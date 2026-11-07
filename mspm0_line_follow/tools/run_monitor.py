#!/usr/bin/env python3
"""循迹实车监测：自动跑一圈，采数据，出报告 + 曲线图。

用法（车轮落地，车已摆好、灰度已标定）：

    python tools/run_monitor.py --port COM7
    python tools/run_monitor.py --port COM7 --base 200 --kp 120 --kd 130 --duration 20

它做的事：
    1. 连串口，等欢迎语
    2. 可选下发 gain，然后 stream 50ms
    3. 发 start 让车跑
    4. 采 duration 秒（Ctrl-C 可提前结束并照常出报告）
    5. 发 stop，落盘 CSV + PNG + 终端报告

报告重点（都是今天排查出来的关键量）：
    * 直道蛇形：误差过零频率、峰峰值、mask 跳格率
    * 球会不会被甩：车速共模的最大加速度 / 转向差模变化率
    * 丢线事件：时间、持续、位置
"""

import argparse
import csv
import os
import re
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("缺 pyserial：pip install pyserial")
    sys.exit(2)

CTRL_DT_MS = 5.0          # 控制拍
RE_HEAD = re.compile(
    r"\[(\w+)\]\s+seq=(\d+)\s+err=(-?\d+)\s+valid=(\d+)\s+mask=([0-9a-fA-F]+)")
RE_BODY = re.compile(
    r"vL=(-?\d+)\s+vR=(-?\d+)\s+mm/s\s+\|\s+pwmL=(-?\d+)\s+pwmR=(-?\d+)"
    r"\s+\|\s+tgtL=(-?\d+)\s+tgtR=(-?\d+)")


def pick_port(explicit):
    if explicit:
        return explicit
    ports = list(list_ports.comports())
    if not ports:
        print("没找到串口。检查 USB-TTL 是否插好，或用 --port 指定。")
        sys.exit(2)
    print("可用串口：")
    for i, p in enumerate(ports):
        print("  [%d] %s  %s" % (i, p.device, p.description))
    if len(ports) == 1:
        print("只用唯一的 %s" % ports[0].device)
        return ports[0].device
    idx = int(input("选哪个？序号: ").strip() or "0")
    return ports[idx].device


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="COM7 / /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--duration", type=float, default=20.0, help="采集秒数")
    ap.add_argument("--stream", type=int, default=50, help="板端打印周期 ms")
    ap.add_argument("--base", type=int, help="下发起步基线速度")
    ap.add_argument("--kp", type=int, help="下发 kp(千分)")
    ap.add_argument("--kd", type=int, help="下发 kd(千分)")
    ap.add_argument("--outdir", default="tools/runs")
    args = ap.parse_args()

    port = pick_port(args.port)
    os.makedirs(args.outdir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(args.outdir, "run_%s.csv" % stamp)
    png_path = os.path.join(args.outdir, "run_%s.png" % stamp)

    try:
        ser = serial.Serial(port, args.baud, timeout=0.2)
    except Exception as exc:
        print("打开 %s 失败：%s" % (port, exc))
        print("（串口助手占用了？先关掉它。）")
        return 2

    def send(cmd):
        ser.write((cmd + "\n").encode("ascii"))
        ser.flush()
        time.sleep(0.12)

    print("已连 %s，等板子就绪..." % port)
    buf = b""
    t0 = time.time()
    while time.time() - t0 < 3.0:                    # 等欢迎语
        buf += ser.read(256)
        if b"debug ===" in buf:
            break
    print("板子在线")

    if args.base is not None:
        send("gain %d %d %d" % (args.base,
                                args.kp if args.kp is not None else 120,
                                args.kd if args.kd is not None else 130))
    send("stream %d" % args.stream)
    send("start")
    print("发车！采集 %.0f 秒（Ctrl-C 提前结束）" % args.duration)

    rows = []
    cur = None
    start = time.time()
    try:
        while time.time() - start < args.duration:
            line = ser.readline().decode("ascii", errors="ignore").strip()
            if not line:
                continue
            m = RE_HEAD.search(line)
            if m:
                if cur:
                    rows.append(cur)                 # 上一帧只有头，收尾
                cur = {
                    "state": m.group(1), "seq": int(m.group(2)),
                    "err": int(m.group(3)), "valid": int(m.group(4)),
                    "mask": int(m.group(5), 16),
                }
                continue
            m = RE_BODY.search(line)
            if m and cur is not None:
                cur.update({
                    "vL": int(m.group(1)), "vR": int(m.group(2)),
                    "pwmL": int(m.group(3)), "pwmR": int(m.group(4)),
                    "tgtL": int(m.group(5)), "tgtR": int(m.group(6)),
                })
                rows.append(cur)
                cur = None
    except KeyboardInterrupt:
        print("\n提前结束")
        if cur:
            rows.append(cur)

    send("stop")
    ser.close()

    # 补齐缺字段（只收到头的帧）
    for r in rows:
        for k in ("vL", "vR", "pwmL", "pwmR", "tgtL", "tgtR"):
            r.setdefault(k, 0)

    if not rows:
        print("一帧都没收到。检查 stream 命令、波特率、接线。")
        return 1

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print("CSV: %s  (%d 帧)" % (csv_path, len(rows)))

    report_and_plot(rows, args.stream, png_path)
    return 0


def report_and_plot(rows, stream_ms, png_path):
    n = len(rows)
    t = [r["seq"] * CTRL_DT_MS / 1000.0 for r in rows]       # 板载时间 s
    t = [x - t[0] for x in t]
    err = [r["err"] for r in rows]
    val = [r["valid"] for r in rows]
    cm = [(r["tgtL"] + r["tgtR"]) / 2.0 for r in rows]        # 共模车速
    df = [(r["tgtL"] - r["tgtR"]) / 2.0 for r in rows]        # 差模转向
    vv = [(r["vL"] + r["vR"]) / 2.0 for r in rows]            # 实测轮速

    dt = stream_ms / 1000.0
    valid_t = sum(1 for v in val if v) * dt
    lost_events = []
    i = 0
    while i < n:
        if not val[i]:
            j = i
            while j < n and not val[j]:
                j += 1
            lost_events.append((t[i], t[j - 1] - t[i]))
            i = j
        else:
            i += 1

    # ---- 误差统计（只统计有效帧）----
    ve = [err[i] for i in range(n) if val[i]]
    if ve:
        mean = sum(ve) / len(ve)
        rms = (sum((x - mean) ** 2 for x in ve) / len(ve)) ** 0.5
        pp = max(ve) - min(ve)
        # 过零次数（去掉居中偏移后）→ 蛇形频率
        zc = sum(1 for k in range(1, len(ve))
                 if (ve[k - 1] - mean) * (ve[k] - mean) < 0)
        dur = valid_t if valid_t > 0 else dt
    else:
        mean = rms = pp = 0.0
        zc = 0
        dur = dt

    # ---- 加速度（200ms 窗口差分，避开 10mm/s 量化噪声）----
    def accel(series):
        win = max(1, int(0.2 / dt))
        a = []
        for k in range(win, len(series)):
            a.append(abs(series[k] - series[k - win]) / (win * dt))
        return (max(a) if a else 0.0), (sum(a) / len(a) if a else 0.0)

    acc_max, acc_avg = accel(cm)
    jrk_max, jrk_avg = accel(df)

    # ---- mask 跳格率（离散探头抖动）----
    flips = sum(1 for k in range(1, n) if rows[k]["mask"] != rows[k - 1]["mask"])

    print("\n================ 报告 ================")
    print("时长 %.1fs / %d 帧（%.0f%%)  有效 %.1fs" %
          (t[-1], n, 100.0 * n * dt / max(t[-1], 1e-3), valid_t))
    print("误差（有效帧）：均值 %+.0f  RMS %.0f  峰峰 %d" % (mean, rms, pp))
    print("  过零 %.1f 次/秒  → %s" % (
        zc / max(dur, 1e-3),
        "疑似控制/机械振荡" if zc / max(dur, 1e-3) > 0.8 else "无明显周期振荡"))
    print("车速加速度：峰值 %.0f 平均 %.0f mm/s²  %s" % (
        acc_max, acc_avg,
        "⚠ 球可能被前后甩" if acc_max > 400 else "球应能跟上"))
    print("转向差模变化率：峰值 %.0f 平均 %.0f mm/s²  %s" % (
        jrk_max, jrk_avg,
        "⚠ 离心力可能甩球" if jrk_max > 1500 else "正常"))
    print("mask 跳格：%d 次（%.1f 次/秒）%s" % (
        flips, flips / max(t[-1], 1e-3),
        "  ⚠ 探头跳格频繁，直线会抖" if flips / max(t[-1], 1e-3) > 15 else ""))
    if lost_events:
        print("丢线 %d 次：" % len(lost_events))
        for (ts, d) in lost_events[:10]:
            print("   t=%.2fs 持续 %.2fs" % (ts, d))
    else:
        print("丢线：无 ✓")
    print("=====================================")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
        ax[0].plot(t, err, lw=1)
        ax[0].set_ylabel("err")
        ax[0].axhline(0, color="gray", lw=0.5)
        ax[1].plot(t, cm, label="cmd common (target)")
        ax[1].plot(t, vv, label="measured", alpha=0.7)
        ax[1].set_ylabel("speed mm/s")
        ax[1].legend(loc="lower right", fontsize=8)
        ax[2].plot(t, df, color="tab:orange")
        ax[2].set_ylabel("diff (turn)")
        ax[3].plot(t, [r["mask"] for r in rows], lw=0.8)
        ax[3].set_ylabel("mask")
        ax[3].set_xlabel("t (s)")
        for a in ax:
            a.grid(alpha=0.3)
        fig.suptitle("line_follow run")
        fig.tight_layout()
        fig.savefig(png_path, dpi=110)
        print("图: %s" % png_path)
    except Exception as exc:
        print("（画图跳过：%s）" % exc)


if __name__ == "__main__":
    sys.exit(main())
