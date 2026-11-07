#!/usr/bin/env python3
"""执行器模型标定：把串口 open 测试的 CSV 拟合成仿真参数。

用法
----
1. 车轮**架空**（离地），车静止，串口 115200 连 PA10/PA11。
2. 发若干组阶跃，每组等上一组打印 "open done" 后再发下一组：
       open 200 800
       open 400 800
       open 600 800
       open 800 800
       open 1000 800        （左右同 PWM；负值测反转）
   建议从 150 开始每档 +100，直到 1000，并记录哪几档轮子根本不转（测死区）。
3. 把串口日志存成文件（串口助手保存 / 或脚本 --port 直接读），然后：
       python tools/fit_actuator.py log.txt
   或直接在线收：
       python tools/fit_actuator.py --port COM7 --collect

输出
----
每档 PWM 的：稳态轮速 vL/vR、时间常数 tau（63% 上升时间）、
左右增益比；以及可直接填入 sim 的 v_max_mm_s / motor_tau_ms /
gain_left / gain_right。

原理
----
一阶模型 v(t) = v_inf * (1 - exp(-t/tau))。
在稳态段取均值 v_inf，在上升段找 v 首次达到 0.632*v_inf 的时刻
即 tau（相对阶跃起点）。
"""

import math
import sys

# 与固件一致：1489 counts/rev、150.8mm/rev → 10ms 窗口 1 计数 ≈ 10.1mm/s
SPEED_QUANT_MM_S = 150.8 / 1489.0 / 0.010


def parse(lines):
    """返回 {pwm: [(t_ms, vL, vR), ...]}，只收 O, 行。"""
    runs = {}
    cur_pwm = None
    for ln in lines:
        ln = ln.strip()
        if ln.startswith("open pwm="):
            # open pwm=400 for 800 ms
            try:
                cur_pwm = int(ln.split("pwm=")[1].split()[0])
            except (IndexError, ValueError):
                cur_pwm = None
            runs.setdefault(cur_pwm, [])
        elif ln.startswith("O,"):
            parts = ln.split(",")
            if len(parts) < 5:
                continue
            try:
                t = float(parts[1])
                vl = float(parts[2])
                vr = float(parts[3])
            except ValueError:
                continue
            if cur_pwm is not None:
                runs[cur_pwm].append((t, vl, vr))
    return {k: v for k, v in runs.items() if k is not None and v}


def fit_one(samples, settle_frac=0.7):
    """拟合一档阶跃。返回 (v_inf_L, v_inf_R, tau_L_ms, tau_R_ms)。"""
    n = len(samples)
    if n < 5:
        return None
    t_end = samples[-1][0]
    # 稳态段：后 30%
    tail = [s for s in samples if s[0] >= t_end * settle_frac]
    vinf_l = sum(s[1] for s in tail) / len(tail)
    vinf_r = sum(s[2] for s in tail) / len(tail)

    def tau_of(vinf, idx):
        if abs(vinf) < SPEED_QUANT_MM_S:
            return None
        target = 0.632 * vinf
        for (t, vl, vr) in samples:
            v = vl if idx == 1 else vr
            if (vinf > 0 and v >= target) or (vinf < 0 and v <= target):
                return t
        return None

    return vinf_l, vinf_r, tau_of(vinf_l, 1), tau_of(vinf_r, 2)


def main():
    src = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else None
    if src is None:
        print(__doc__)
        return 1
    with open(src, encoding="utf-8", errors="ignore") as f:
        runs = parse(f.readlines())

    if not runs:
        print("没找到 O, 行。先发 open <pwm> <ms> 并把日志存下来。")
        return 1

    print("%8s %10s %10s %8s %8s %8s" %
          ("pwm", "vL", "vR", "tauL", "tauR", "vR/vL"))
    taus, gains = [], []
    deadband_hi = 0
    for pwm in sorted(runs, key=abs):
        r = fit_one(runs[pwm])
        if r is None:
            continue
        vl, vr, tl, tr = r
        ratio = (vr / vl) if abs(vl) > SPEED_QUANT_MM_S else float("nan")
        print("%8d %10.1f %10.1f %8s %8s %8.3f" %
              (pwm, vl, vr,
               "-" if tl is None else "%.0f" % tl,
               "-" if tr is None else "%.0f" % tr,
               ratio))
        if abs(vl) < 20 and abs(vr) < 20:
            if abs(pwm) > abs(deadband_hi):
                deadband_hi = pwm
        # 低速档速度只有几十 mm/s，10.1mm/s 量化让 63% 点判不准，
        # tau 只统计 |pwm| >= 400 的档位
        if abs(pwm) >= 400:
            if tl is not None:
                taus.append(tl)
            if tr is not None:
                taus.append(tr)
        if abs(pwm) >= 800 and abs(vl) > 50:
            gains.append(vl / (pwm / 1000.0))

    print()
    if deadband_hi:
        print("死区估计：|pwm| <= %d 不转 → 约 %.0f%% 占空比（仿真 dead=0.15）"
              % (abs(deadband_hi), abs(deadband_hi) / 10.0))
    if taus:
        tau = sum(taus) / len(taus)
        print("motor_tau_ms  ≈ %.0f   (仿真默认 100)" % tau)
    if gains:
        g = sum(gains) / len(gains)
        print("v_max_mm_s    ≈ %.0f   (仿真默认 800)" % g)
    print()
    print("把上面两个数填进 sim/sim_platform.c 的 g_sim 初始值，")
    print("左右差异用 gain_left/gain_right 比例表示（右/左那个比值）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
