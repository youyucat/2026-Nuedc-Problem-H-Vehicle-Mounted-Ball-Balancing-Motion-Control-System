#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""调参会话：车持续跑圈，按窗口分析数据、实时下发参数。

角色分工（单串口由本进程独占，三角色跑在内部线程/主循环）：
  采集线程  持续读串口帧入环形缓冲（永不停）
  主循环    每 --window 秒：切窗口 → 存 CSV → 分析 → 决策 → 下发 gain（运行中）
  安全监督  丢线超阈值立即停车回滚；会话结束自动停车

用法：
  python tools/tune_session.py --port COM9 --session 300 --window 25 --kp 120

流程：
  1. 连串口 → 下发初始 gain → stream → start
  2. 车持续跑；每 window 秒输出报告并按规则调整 kp（运行中生效）
  3. 会话结束（或安全触发）→ stop → 打印 kp 轨迹
"""

import argparse
import csv
import math
import os
import re
import sys
import threading
import time

import serial

RE_HEAD = re.compile(
    r"\[(\w+)\]\s+seq=(\d+)\s+err=(-?\d+)\s+valid=(\d+)\s+mask=([0-9a-fA-F]+)")
RE_BODY = re.compile(
    r"vL=(-?\d+)\s+vR=(-?\d+)\s+mm/s\s+\|\s+pwmL=(-?\d+)\s+pwmR=(-?\d+)"
    r"\s+\|\s+tgtL=(-?\d+)\s+tgtR=(-?\d+)")

FRAME_MS = 100          # 板端 stream 周期（ms）
LOST_STOP_MS = 2500     # 连续丢线达到该值 → 停车回滚
KP_MIN, KP_MAX, KP_STEP = 100, 220, 20


class Collector(threading.Thread):
    """后台采集线程：持续解析帧到共享列表（带锁）。"""

    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser
        self.frames = []          # 共享缓冲（dict 列表）
        self.lock = threading.Lock()
        self.running = True
        self.rx_garbage = 0

    def run(self):
        rx = b""
        while self.running:
            try:
                rx += self.ser.read(512)
            except Exception:
                break
            while b"\n" in rx:
                raw, rx = rx.split(b"\n", 1)
                line = raw.decode("ascii", errors="ignore").strip()
                if not line:
                    continue
                m = RE_HEAD.search(line)
                if m:
                    with self.lock:
                        self.frames.append({
                            "t": time.time(),
                            "state": m.group(1),
                            "seq": int(m.group(2)),
                            "err": int(m.group(3)),
                            "valid": int(m.group(4)),
                            "mask": int(m.group(5), 16),
                            "vL": None, "vR": None,
                        })
                    continue
                m = RE_BODY.search(line)
                if m:
                    with self.lock:
                        if self.frames:
                            last = self.frames[-1]
                            if last["vL"] is None:
                                last["vL"] = int(m.group(1))
                                last["vR"] = int(m.group(2))
                    continue
                self.rx_garbage += 1

    def snapshot_since(self, t_start):
        with self.lock:
            return [f for f in self.frames if f["t"] >= t_start]

    def total(self):
        with self.lock:
            return len(self.frames)


class SimCollector(threading.Thread):
    """合成数据源：直道(8s)→弯道(8s, err 大)→丢线(3s) 循环。
    用于无串口验证全链路（线程/窗口/CSV/决策/下发）。"""

    def __init__(self):
        super().__init__(daemon=True)
        self.frames = []
        self.lock = threading.Lock()
        self.running = True
        self.rx_garbage = 0
        self.seq = 0

    def run(self):
        t0 = time.time()
        while self.running:
            t = time.time() - t0
            phase = t % 19.0
            seq = int(t / 0.1)
            if seq == self.seq:
                time.sleep(0.02)
                continue
            self.seq = seq
            if phase > 16.0:
                f = {"err": 0, "valid": 0, "mask": 0}
            elif phase > 8.0:
                f = {"err": 520 + int(60 * math.sin(t * 3)),
                     "valid": 1, "mask": 0x60}
            else:
                f = {"err": int(120 * math.sin(t * 2.5)),
                     "valid": 1, "mask": 0x18}
            f.update({"t": time.time(), "state": "RUN", "seq": seq,
                      "vL": 200, "vR": 190})
            with self.lock:
                self.frames.append(f)
            time.sleep(0.05)

    def snapshot_since(self, t_start):
        with self.lock:
            return [f for f in self.frames if f["t"] >= t_start]

    def total(self):
        with self.lock:
            return len(self.frames)


def analyze(frames, window_s):
    """窗口指标：有效帧误差统计 + 丢线时长。"""
    valid = [f for f in frames if f["valid"] == 1]
    lost_frames = len(frames) - len(valid)
    out = {
        "frames": len(frames),
        "valid": len(valid),
        "lost_ms": lost_frames * FRAME_MS,
        "mean_ae": 0.0, "rms": 0.0, "zc": 0.0, "corner_frac": 0.0,
    }
    if valid:
        errs = [f["err"] for f in valid]
        abses = [abs(e) for e in errs]
        out["mean_ae"] = sum(abses) / len(abses)
        out["rms"] = math.sqrt(sum(e * e for e in errs) / len(errs))
        signs = [1 if e >= 0 else -1 for e in errs]
        out["zc"] = sum(1 for k in range(1, len(signs))
                        if signs[k] != signs[k - 1]) / window_s
        out["corner_frac"] = (sum(1 for a in abses if a >= 300)
                              / len(abses))
    return out


def decide(metrics, kp, last_kp, history):
    """窗口决策：返回 (action, new_kp, reason)。

    规则：
      lost_ms > 1500            → revert（安全监督会停车）
      corner_frac > 0.4 且
      mean_ae >= 300            → kp+（弯道吃紧，若上次没加过且未到上限）
      mean_ae < 150 且 zc > 2.5 → kp-（直线抖）
      其他                      → hold
    加分项：连续两次弯道吃紧才加，避免单弯误判。
    """
    if metrics["lost_ms"] > 1500:
        return "revert", last_kp, "丢线 %dms" % metrics["lost_ms"]
    if (metrics["corner_frac"] > 0.4 and metrics["mean_ae"] >= 300
            and kp < KP_MAX):
        history.append("corner")
        if len(history) >= 2 and history[-2] == "corner":
            history.clear()
            return "raise", min(kp + KP_STEP, KP_MAX), \
                "弯道吃紧 mean|e|=%.0f(弯占比%.0f%%)" % (
                    metrics["mean_ae"], 100 * metrics["corner_frac"])
        return "hold", kp, "弯道吃紧(第1次观察)"
    if metrics["mean_ae"] < 150 and metrics["zc"] > 2.5 and kp > KP_MIN:
        return "lower", max(kp - KP_STEP, KP_MIN), \
            "直线抖 zc=%.1f/s" % metrics["zc"]
    return "hold", kp, "mean|e|=%.0f zc=%.1f" % (
        metrics["mean_ae"], metrics["zc"])


def save_segment(outdir, idx, frames):
    path = os.path.join(outdir, "seg_%03d.csv" % idx)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=[
            "t", "state", "seq", "err", "valid", "mask", "vL", "vR"])
        w.writeheader()
        for f in frames:
            w.writerow({k: f.get(k) for k in w.fieldnames})
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="COM9 等（--simulate 时可省）")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--session", type=float, default=300,
                    help="总会话秒数（车持续跑）")
    ap.add_argument("--window", type=float, default=25.0,
                    help="分析窗口秒数")
    ap.add_argument("--base", type=int, default=200)
    ap.add_argument("--kp", type=int, default=120)
    ap.add_argument("--kd", type=int, default=130)
    ap.add_argument("--outdir", default="tools/runs/session")
    ap.add_argument("--dry-run", action="store_true",
                    help="不发 start（车原地不动），其余链路照走")
    ap.add_argument("--simulate", action="store_true",
                    help="合成数据源（不需要串口），测全链路逻辑")
    args = ap.parse_args()

    if (not args.simulate) and (not args.port):
        ap.error("--simulate 之外必须给 --port")
    os.makedirs(args.outdir, exist_ok=True)
    sent_log = []
    if args.simulate:
        ser = None

        def send(cmd):
            sent_log.append((time.time(), cmd))
            print("  [下发] %s" % cmd)
    else:
        ser = serial.Serial(args.port, args.baud, timeout=0.15)
        time.sleep(0.5)
        ser.reset_input_buffer()

        def send(cmd):
            ser.write((cmd + "\n").encode("ascii"))
            ser.flush()
            time.sleep(0.1)

    print("== 调参会话 %.0fs / 窗口 %.0fs ==" % (args.session, args.window))
    print("初始 gain: base=%d kp=%d kd=%d" % (args.base, args.kp, args.kd))
    send("gain %d %d %d" % (args.base, args.kp, args.kd))
    send("stream 100")

    if args.simulate:
        collector = SimCollector()
    else:
        collector = Collector(ser)
    collector.start()

    if not args.dry_run:
        send("start")
    t0 = time.time()
    print("%s！持续 %.0f 秒，每 %.0f 秒分析一次\n"
          % ("发车" if not args.dry_run else "干跑（不发 start）",
             args.session, args.window))

    kp = args.kp
    kp_history = [(0.0, kp)]
    hist = []
    window_idx = 0
    win_start = t0
    aborted = False

    try:
        while time.time() - t0 < args.session:
            time.sleep(0.2)
            # ---- 安全监督：连续丢线立即停车 ----
            recent = collector.snapshot_since(time.time() - 3.0)
            lost_run = 0
            for f in reversed(recent):
                if f["valid"] == 0:
                    lost_run += FRAME_MS
                else:
                    break
            if lost_run > LOST_STOP_MS:
                send("stop")
                print("\n!! 连续丢线 %.1fs → 紧急停车，会话结束" % (lost_run / 1000))
                aborted = True
                break

            # ---- 窗口分析 + 下发 ----
            if time.time() - win_start >= args.window:
                window_idx += 1
                frames = collector.snapshot_since(win_start)
                metrics = analyze(frames, args.window)
                path = save_segment(args.outdir, window_idx, frames)
                action, new_kp, reason = decide(
                    metrics, kp, kp_history[-1][1], hist)
                if action == "raise":
                    send("gain %d %d %d" % (args.base, new_kp, args.kd))
                elif action == "lower":
                    send("gain %d %d %d" % (args.base, new_kp, args.kd))
                elif action == "revert":
                    new_kp = kp_history[-1][1]
                    send("gain %d %d %d" % (args.base, new_kp, args.kd))

                print("[窗口%02d t=%3.0fs] 帧%3d 丢%5dms mean|e|=%3.0f "
                      "RMS=%3.0f zc=%.1f 弯占比%3.0f%% | %s → kp=%d (%s)"
                      % (window_idx, time.time() - t0, metrics["frames"],
                         metrics["lost_ms"], metrics["mean_ae"],
                         metrics["rms"], metrics["zc"],
                         100 * metrics["corner_frac"], action, new_kp,
                         reason))
                if new_kp != kp:
                    kp_history.append((time.time() - t0, new_kp))
                kp = new_kp
                win_start = time.time()
    except KeyboardInterrupt:
        print("\n手动中断")
    finally:
        send("stop")
        time.sleep(0.3)
        collector.running = False
        collector.join(timeout=1.0)
        if ser is not None:
            ser.close()

    print("\n== 会话结束（%s）==" % ("紧急停车" if aborted else "正常"))
    print("kp 轨迹: " + " → ".join(
        "%d@%.0fs" % (k, t) for t, k in kp_history))
    print("最终建议: gain %d %d %d" % (args.base, kp, args.kd))
    print("窗口数据: %s/seg_*.csv" % args.outdir)


if __name__ == "__main__":
    main()
