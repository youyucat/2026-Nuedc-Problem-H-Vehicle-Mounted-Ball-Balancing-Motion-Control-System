/**
  ******************************************************************************
  * @file    sim_main.c
  * @brief   PC 仿真：球-导轨 + 静摩擦 + 视觉延迟 + 量化噪声
  *
  * 目的：在电脑上先把闭环跑稳，再上机。参数在这里调，不靠台架盲试。
  *
  * 编译：make        运行：./sim_ctrl [初始偏移mm]
  ******************************************************************************
  */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "ball_ctrl_v2.h"

/* ---------------- 被控对象参数（可按实测改） ---------------- */
typedef struct {
    double gain;          /* mm/s² per deg */
    double a_break;       /* 静摩擦突破阈值 mm/s² */
    double a_roll;        /* 滚动阻力 mm/s² */
    double rail_half_mm;  /* 导轨半长 125mm */
} plant_t;

/* ---------------- 视觉链路参数 ---------------- */
typedef struct {
    double   period_ms;   /* 出帧周期 */
    double   delay_ms;    /* 传输延迟 */
    double   noise_mm;    /* 测量噪声标准差 */
} vision_t;

static unsigned int g_seed = 12345U;
static double rnd_uniform(void)
{
    g_seed = g_seed * 1103515245U + 12345U;
    return (double)((g_seed >> 16) & 0x7FFF) / 32767.0;   /* 0..1 */
}
static double rnd_noise(double sigma)
{
    /* 近似高斯：两个均匀分布相加 */
    return (rnd_uniform() + rnd_uniform() - 1.0) * sigma * 1.5;
}

int main(int argc, char **argv)
{
    ball_v2_cfg_t cfg;
    ball_v2_t     ctrl;
    plant_t       plant;
    vision_t      vis;

    double x_mm, v_mms;           /* 真实状态 */
    double x0_mm = 40.0;
    double t_ms  = 0.0;
    const double dt_ms = 1.0;

    /* 视觉延迟队列：采样侧 20ms 入队，释放侧延迟 delay_ms 后出队 */
    enum { FIFO_MAX = 16 };
    double meas_fifo[FIFO_MAX];   /* 延迟队列，单位 mm */
    int    fifo_n = 0;
    double next_sample_ms  = 0.0;
    double next_release_ms = 0.0;
    int    release_started = 0;

    /* 控制器节拍 */
    double next_ctrl_ms = 0.0;
    int32_t angle_cmd = 0;

    /* 执行器实际角度（deg）。
     * 早期仿真把 angle_cmd 直接当摆杆角度用，等于假设电机瞬间到位；
     * 实测 STEPPER_SERVO_SPEED=2400pps、66.67 步/度 → 摆杆最大 36°/s，
     * 要摆到 5° 需要 139ms。这个滞后和视觉延迟同量级，必须建模。 */
    double angle_act = 0.0;
    const double ACT_SLEW_DEG_S = 36.0;

    /* 静摩擦突破阈值（mm/s²）。
     * 第一版按 T1 数据推得 8，但台架实测稳态输出 ±0.8°（≈144 mm/s²）
     * 球仍不动 —— T1 那次「1° 让球动 2.6mm」反算只有 0.83 mm/s²，
     * 是标定漂移而非滚动。实测 M,1000（1.0°）会让球滚 → 阈值 ≤1.0°，取 180（1.0°），
     * 并用命令行参数扫 50~200 检查鲁棒性。 */
    double g_a_break = 180.0;

    /* 指标 */
    double max_overshoot_mm = 0.0;
    double max_angle_mdeg = 0.0;      /* 全程 |下发角度| 峰值 */
    long   n_angle = 0;
    long   n_big_angle = 0;           /* |>0.3°| 的拍数，静置时应为 0 */
    double last_out_of_band_ms = 0.0;
    int    crossed = 0;
    double x_start;

    if (argc > 1) x0_mm = atof(argv[1]);

    ball_v2_default_cfg(&cfg);

    /* 可选命令行调参：
     * ./sim_ctrl [x0] [kp] [kd] [delay_ms] [pulse_mdeg] [gain] [a_break] */
    if (argc > 2) cfg.kp_mdeg_per_mm   = atoi(argv[2]);
    if (argc > 3) cfg.kd_mdeg_per_mm_s = atoi(argv[3]);
    if (argc > 4) cfg.loop_delay_ms    = atoi(argv[4]);
    if (argc > 5) cfg.stick_pulse_mdeg = atoi(argv[5]);
    if (argc > 6) cfg.gain_mm_s2_per_deg = atoi(argv[6]);
    if (argc > 7) g_a_break = atof(argv[7]);

    plant.gain         = (double)cfg.gain_mm_s2_per_deg;
    /* 静摩擦阈值：2026-10-25 角度标定后重算。
     * 早期 T1 扫描时角度虚高 15 倍（命令 1° 实际只有 0.067°），
     * 那时命令 1° 就能让球从静止滚动 → 真实阈值 < 0.067° × 180 ≈ 12 mm/s²。
     * 所以原来的 400（≈2.2°）是错的，且「需要 5° 脱困脉冲」这个结论也作废。
     * 取 8 mm/s²（≈0.044°），并可扫 5/8/20 做敏感性。 */
    plant.a_break      = g_a_break;
    /* 滚动阻力：钢球在钢/铝上的滚动阻力系数约 0.001~0.002 → 10~20 mm/s² */
    plant.a_roll       = 10.0;
    plant.rail_half_mm = 125.0;

    /* 视觉链路（实测 29fps / 480x360）：
     *   period 34.5ms，1px = 0.64mm，检测耗时 24.5ms 已计入延迟 */
    vis.period_ms = (double)cfg.frame_period_ms;
    vis.delay_ms  = 50.0;
    vis.noise_mm  = 0.15;

    ball_v2_init(&ctrl, &cfg);
    ball_v2_set_target(&ctrl, 0);

    x_mm  = x0_mm;
    v_mms = 0.0;
    x_start = x_mm;

    printf("# 初始偏移 %.1f mm, gain=%.0f, delay=%dms, kp=%d, kd=%d\n",
           x0_mm, plant.gain, cfg.loop_delay_ms, cfg.kp_mdeg_per_mm, cfg.kd_mdeg_per_mm_s);
    printf("#   t_ms   x_true   x_meas    v_true   v_est   angle_cmd  act_deg  pulse\n");

    while (t_ms < 8000.0) {

        /* ---------- 0. 执行器：36°/s 斜率限幅 ---------- */
        {
            double target_deg = (double)angle_cmd / 1000.0;
            double max_step   = ACT_SLEW_DEG_S * dt_ms / 1000.0;
            double diff       = target_deg - angle_act;

            if (diff >  max_step) diff =  max_step;
            if (diff < -max_step) diff = -max_step;
            angle_act += diff;
        }

        /* ---------- 1. 被控对象积分 ---------- */
        {
            /* 符号约定：正角抬高 +x 端 → 球向 -x 加速 */
            double a_drive = -plant.gain * angle_act;                     /* mm/s² */
            double a = 0.0;

            if (fabs(v_mms) < 1.0 && fabs(a_drive) < plant.a_break) {
                /* 静摩擦锁死 */
                v_mms = 0.0;
            } else {
                a = a_drive;
                if (fabs(v_mms) > 0.5) {
                    a -= plant.a_roll * ((v_mms > 0) ? 1.0 : -1.0);   /* 滚动阻力 */
                }
                v_mms += a * (dt_ms / 1000.0);
            }
            x_mm += v_mms * (dt_ms / 1000.0);

            if (x_mm >  plant.rail_half_mm) { x_mm =  plant.rail_half_mm; v_mms = 0; }
            if (x_mm < -plant.rail_half_mm) { x_mm = -plant.rail_half_mm; v_mms = 0; }
        }

        /* ---------- 2. 视觉采样与延迟 ---------- */
        if (t_ms >= next_sample_ms) {
            double meas = x_mm + rnd_noise(vis.noise_mm);   /* mm */
            /* 480x360 下 1 像素 = 0.64mm：量化到像素网格，而不是理想的 0.1mm */
            meas = floor(meas / 0.64 + 0.5) * 0.64;
            if (fifo_n < FIFO_MAX) meas_fifo[fifo_n++] = meas;
            next_sample_ms += vis.period_ms;
        }

        /* 释放侧：比采样晚 delay_ms，逐个吐出（保持队列长度恒定） */
        if (!release_started && t_ms >= vis.delay_ms + vis.period_ms) {
            release_started  = 1;
            next_release_ms  = t_ms;
        }
        if (release_started && t_ms >= next_release_ms && fifo_n > 0) {
            double out_mm = meas_fifo[0];
            int i;
            for (i = 1; i < fifo_n; i++) meas_fifo[i-1] = meas_fifo[i];
            fifo_n--;
            ball_v2_feed(&ctrl, (int32_t)(out_mm * 10.0), (uint32_t)t_ms);
            next_release_ms = t_ms + vis.period_ms;
        }

        /* ---------- 3. 控制器 100Hz ---------- */
        if (t_ms >= next_ctrl_ms) {
            angle_cmd = ball_v2_step(&ctrl, (uint32_t)t_ms);
            next_ctrl_ms += 10.0;
        }

        /* ---------- 4. 指标统计 ---------- */
        if (!crossed && (x_start > 0 ? x_mm < 0.0 : x_mm > 0.0)) crossed = 1;
        if (crossed) {
            double o = (x_start > 0) ? -x_mm : x_mm;
            if (o > max_overshoot_mm) max_overshoot_mm = o;
        }
        if (fabs(x_mm) >= 10.0) last_out_of_band_ms = t_ms;

        /* 角度活动量：静置测试用。若控制器在球已稳住时还发脉冲，
         * 这里会明显变大（一次 0.6° 脉冲就会拉到 600 以上）。 */
        {
            double aa = fabs((double)angle_cmd);
            if (aa > max_angle_mdeg) max_angle_mdeg = aa;
            n_angle++;
            if (aa > 300.0) n_big_angle++;     /* 统计「大动作」拍数 */
        }

        /* ---------- 5. 打印 ---------- */
        if (fmod(t_ms, 50.0) < 0.5) {
            printf("%7.0f %8.2f %8.2f %8.2f %7.1f %10d %8.3f %6d\n",
                   t_ms, x_mm, (double)ball_v2_pos(&ctrl) / 10.0,
                   v_mms, (double)ball_v2_vel(&ctrl),
                   angle_cmd, angle_act, ball_v2_pulse(&ctrl));
        }

        t_ms += dt_ms;
    }

    printf("\n================ 指标 ================\n");
    printf("初始偏移        : %.1f mm\n", x0_mm);
    printf("最大反向过冲    : %.2f mm\n", max_overshoot_mm);
    printf("进入 ±10mm 时间 : %.0f ms\n", last_out_of_band_ms);
    printf("终值误差        : %.2f mm\n", x_mm);
    printf("峰值角度输出    : %.0f mdeg\n", max_angle_mdeg);
    printf("大动作(>0.3°)拍数: %ld / %ld\n", n_big_angle, n_angle);
    printf("是否收敛        : %s\n",
           (fabs(x_mm) < 3.0 && max_overshoot_mm < 15.0) ? "是" : "否");
    /* 判据按题目指标：球稳在 ±10mm 内即达标 */
    printf("达标(±10mm)     : %s\n",
           (fabs(x_mm) < 10.0 && max_overshoot_mm < 10.0) ? "是" : "否");
    printf("======================================\n");

    return 0;
}
