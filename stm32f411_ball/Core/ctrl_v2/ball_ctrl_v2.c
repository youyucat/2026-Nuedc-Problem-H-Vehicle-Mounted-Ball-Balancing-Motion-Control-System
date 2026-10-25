/**
  ******************************************************************************
  * @file    ball_ctrl_v2.c
  * @brief   摆杆稳球控制 —— 从零重写版实现（纯 C，无 HAL 依赖）
  ******************************************************************************
  */

#include "ball_ctrl_v2.h"

#include <string.h>

/* ==========================================================================
 *                                工具
 * ========================================================================== */

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int32_t abs_i32(int32_t v)
{
    return (v < 0) ? -v : v;
}

static int32_t sign_i32(int32_t v)
{
    if (v > 0) return 1;
    if (v < 0) return -1;
    return 0;
}

/* ==========================================================================
 *                                默认参数
 * ========================================================================== */

void ball_v2_default_cfg(ball_v2_cfg_t *cfg)
{
    if (cfg == 0) return;

    /* 机械标定：旧工程实测 180 mm/s²/deg；理论 (5/7)g ≈ 122。
     * 两者差 47%，说明这个值必须实测，不能抄。 */
    cfg->gain_mm_s2_per_deg = 180;

    /* 回路延迟（2026-10-25 实测重估）：
     * 视觉 29fps → 帧周期 34.5ms（等帧平均 17）+ 检测耗时 24.5
     * + 串口 1 + 控制周期平均 5 ≈ 48ms，取 50ms。 */
    cfg->loop_delay_ms   = 50;
    /* 执行器真实上限：STEPPER_SERVO_SPEED=2400pps ÷ 66.67 步/度 ≈ 36°/s，
     * 摆到 5° 需 139ms。原来写 60000（60°/s）是假设 1:1 直连时的值，
     * 标定后必须按真实减速比修正，否则控制器会高估动作速度而超调。 */
    cfg->slew_mdeg_per_s = 36000;   /* 36°/s（=2400pps / 66.67） */
    cfg->angle_max_mdeg  = 10000;   /* ±10° */

    /* 增益（2026-10-25 按实测参数重扫）：
     * kp=30 kd=20 在 4 个初始偏移 × 3 档延迟 × 3 档 gain(122/180/250)
     * 共 36 组的组合里 35 组收敛，过冲 ≤3.6mm（指标 10mm）。
     *
     * kd 为什么只能到 20：位置量化 0.64mm、帧间隔 34.5ms，
     * 帧差测速的量化噪声就有 0.64/0.0345 ≈ 18.6 mm/s。kd=70 时
     * 噪声被放大成 1.3° 的角度抖动，仿真里直接发散。
     * 提高 kd 的前提是先把测速噪声压下去（多帧拟合而非两点差分）。
     *
     * 旧默认 80/70 是在「角度虚高 15 倍 + 无执行器滞后」的假设下调的，作废。 */
    cfg->kp_mdeg_per_mm   = 30;
    cfg->kd_mdeg_per_mm_s = 20;

    /* 静摩擦管理（2026-10-25 实机数据二次修正）
     *
     * 第一版按 T1 数据推得阈值 <12 mm/s²，取 pulse=0.2°。
     * 但台架实测：稳态时输出 ±0.8°（≈144 mm/s²）球仍纹丝不动，
     * 说明真实静摩擦远大于 12 —— T1 那次「1° 命令让球动 2.6mm」
     * 反算加速度只有 0.83 mm/s²，那是标定漂移，不是滚动。
     *
     * 静摩擦还会随位置/清洁度变化，固定脉冲要么推不动、要么踢飞，
     * 因此改成自适应：从 0.5° 起，连续推不动就 +0.4°，球一动立刻归零。
     * 0.5/0.9/1.3/1.7/2.1° 依次覆盖 0.56~2.3° 的阈值范围。 */
    cfg->stick_v_thresh_mms   = 8;
    cfg->stick_hold_ms        = 150;
    cfg->stick_pulse_mdeg     = 600;    /* 起始 0.6° */
    cfg->stick_pulse_step_mdeg = 500;   /* 每次没推动 +0.5° */
    cfg->stick_ramp_ms        = 80;
    cfg->stick_hold_plateau_ms = 120;
    cfg->stick_relax_ms       = 100;

    /* 死区与偏置 */
    cfg->deadband_01mm = 8;
    cfg->bias_max_mdeg = 2000;
    cfg->bias_learn_ms = 1500;

    cfg->frame_period_ms = 34;      /* 实测 29fps → 34.5ms，测速按此帧间隔 */
}

/* ==========================================================================
 *                                生命周期
 * ========================================================================== */

void ball_v2_init(ball_v2_t *c, const ball_v2_cfg_t *cfg)
{
    if (c == 0) return;
    memset(c, 0, sizeof(*c));
    c->cfg = cfg;
}

void ball_v2_reset(ball_v2_t *c)
{
    const ball_v2_cfg_t *cfg;

    if (c == 0) return;
    cfg = c->cfg;
    memset(c, 0, sizeof(*c));
    c->cfg = cfg;
}

/* ==========================================================================
 *                                输入
 * ========================================================================== */

void ball_v2_feed(ball_v2_t *c, int32_t x_01mm, uint32_t now_ms)
{
    if (c->have_x) {
        uint32_t dt = now_ms - c->last_x_ms;
        /* 只在合理间隔内做帧差，避免首帧/重捕获踢出一个假速度 */
        if ((dt >= 5U) && (dt <= 200U)) {
            /* dx 单位是 0.1mm：v[mm/s] = dx*0.1 / (dt/1000) = dx*100/dt */
            int32_t dv = (int32_t)(((int64_t)(x_01mm - c->x_raw_01mm) * 100) / (int32_t)dt);
            if (dv > 3000)  dv = 3000;
            if (dv < -3000) dv = -3000;
            c->v_est_mms += (dv - c->v_est_mms) / 4;      /* EMA 1/4 */
        } else {
            c->v_est_mms = 0;
        }
    }

    c->x_raw_01mm = x_01mm;
    c->last_x_ms  = now_ms;

    if (!c->have_x || !c->est_ready) {
        c->x_filt_01mm = x_01mm;
        c->est_ready   = 1;
        c->have_x      = 1;
    } else {
        c->x_filt_01mm += (x_01mm - c->x_filt_01mm) / 2;  /* EMA 1/2 */
    }
}

void ball_v2_set_target(ball_v2_t *c, int32_t xref_01mm)
{
    c->xref_01mm = xref_01mm;
}

/* ==========================================================================
 *                                控制
 * ========================================================================== */

/* 静摩擦脉冲：爬升 → 保持 → 卸力。返回当前脉冲角（带方向） */
static int32_t pulse_profile(ball_v2_t *c, uint32_t dt_ms)
{
    const ball_v2_cfg_t *k = c->cfg;
    int32_t a;

    if (c->pulse_dir == 0) return 0;

    c->pulse_ms += dt_ms;

    if (c->pulse_ms <= (uint32_t)k->stick_ramp_ms) {
        /* 爬升 */
        a = (int32_t)(((int64_t)c->pulse_peak_mdeg * (int32_t)c->pulse_ms) /
                      (k->stick_ramp_ms > 0 ? k->stick_ramp_ms : 1));
    } else if (c->pulse_ms <= (uint32_t)(k->stick_ramp_ms + k->stick_hold_plateau_ms)) {
        a = c->pulse_peak_mdeg;
    } else if (c->pulse_ms <= (uint32_t)(k->stick_ramp_ms + k->stick_hold_plateau_ms +
                                         k->stick_relax_ms)) {
        /* 卸力：从峰值线性回到 0 */
        int32_t over = (int32_t)c->pulse_ms - k->stick_ramp_ms - k->stick_hold_plateau_ms;
        a = (int32_t)(((int64_t)c->pulse_peak_mdeg * (k->stick_relax_ms - over)) /
                      (k->stick_relax_ms > 0 ? k->stick_relax_ms : 1));
    } else {
        c->pulse_dir  = 0;
        c->pulse_ms   = 0;
        c->pulse_mdeg = 0;
        return 0;
    }

    c->pulse_mdeg = a * c->pulse_dir;
    return c->pulse_mdeg;
}

int32_t ball_v2_step(ball_v2_t *c, uint32_t now_ms)
{
    const ball_v2_cfg_t *k = c->cfg;
    uint32_t dt_ms;
    int32_t  err_pred, fb, req, step_lim, out;
    int32_t  a_est, x_pred;
    int32_t  dir;

    if (c->now_ms == 0U) {
        c->now_ms = now_ms;
    }
    dt_ms = now_ms - c->now_ms;
    if (dt_ms == 0U) return c->angle_cmd_mdeg;
    if (dt_ms > 100U) dt_ms = 100U;      /* 长时间没跑，防积分式累积跳变 */
    c->now_ms = now_ms;

    if (!c->est_ready) return c->angle_cmd_mdeg;

    /* ---------- 1. 延迟补偿：把状态前推到「动作真正生效」的时刻 ---------- */
    /* 符号约定：正角抬高 +x 端 → 球向 -x 加速，故加速度取负 */
    a_est = (int32_t)(-((int64_t)k->gain_mm_s2_per_deg * c->angle_cmd_mdeg) / 1000);
    x_pred = c->x_filt_01mm
           + (int32_t)(((int64_t)c->v_est_mms * k->loop_delay_ms) / 100)
           + (int32_t)(((int64_t)a_est * k->loop_delay_ms * k->loop_delay_ms) / 200000);
    c->x_pred_01mm = x_pred;

    err_pred = x_pred - c->xref_01mm;
    if (abs_i32(err_pred) <= k->deadband_01mm) err_pred = 0;

    /* ---------- 2. 状态反馈（本工程 +角抬高 +x 端，正偏差需正角） ---------- */
    fb  = (int32_t)(((int64_t)k->kp_mdeg_per_mm * err_pred) / 10);
    fb += (int32_t)((int64_t)k->kd_mdeg_per_mm_s * c->v_est_mms);
    c->fb_mdeg = fb;

    /* ---------- 3. 机械零位慢学习（只修常驻不水平，不做积分） ---------- */
    if ((abs_i32(c->v_est_mms) <= k->stick_v_thresh_mms) &&
        (abs_i32(err_pred) > k->deadband_01mm)) {
        int32_t dv = (int32_t)(((int64_t)k->bias_max_mdeg * (int32_t)dt_ms) /
                               (k->bias_learn_ms > 0 ? k->bias_learn_ms : 1));
        c->bias_acc_ms += dt_ms;
        if (c->bias_acc_ms >= (uint32_t)k->bias_learn_ms) {
            c->bias_acc_ms = 0;
            c->bias_mdeg = clamp_i32(c->bias_mdeg + dv * sign_i32(err_pred),
                                     -k->bias_max_mdeg, k->bias_max_mdeg);
        }
    } else {
        c->bias_acc_ms = 0;
    }

    /* ---------- 4. 静摩擦状态机 ---------- */
    if (c->pulse_dir != 0) {
        /* 球已经动起来就立刻收脉冲，不等它跑完 */
        if (abs_i32(c->v_est_mms) > k->stick_v_thresh_mms) {
            c->pulse_dir  = 0;
            c->pulse_ms   = 0;
            c->pulse_mdeg = 0;
            c->stick_tries = 0;     /* 推起来了，下次回到起始脉冲 */
        }
    }

    if (c->pulse_dir == 0) {
        if (abs_i32(c->v_est_mms) <= k->stick_v_thresh_mms) {
            c->quiet_ms += dt_ms;
        } else {
            c->quiet_ms = 0;
        }

        if ((c->quiet_ms >= (uint32_t)k->stick_hold_ms) &&
            (abs_i32(err_pred) > k->deadband_01mm)) {
            dir = sign_i32(err_pred);
            if (dir != 0) {
                /* 脉冲起点随「连续推不动次数」递增：
                 * 静摩擦随位置/清洁度变化，固定值要么推不动、要么踢飞。
                 * 每次没推动就加大一档，球一动立刻回到起始值。 */
                int32_t base = k->stick_pulse_mdeg +
                               (int32_t)c->stick_tries * k->stick_pulse_step_mdeg;

                if (c->stick_tries < 20U) c->stick_tries++;

                c->pulse_dir  = dir;
                c->pulse_ms   = 0;
                /* 行程越远，脉冲略大一些，但不超过上限的 60% */
                c->pulse_peak_mdeg = clamp_i32(
                    base + abs_i32(err_pred) / 4,
                    base,
                    (k->angle_max_mdeg * 6) / 10);
                c->quiet_ms = 0;
            }
        }
    }
    (void)pulse_profile(c, dt_ms);

    /* ---------- 5. 合成 + 限幅 + 斜率限幅 ---------- */
    req = fb + c->bias_mdeg + c->pulse_mdeg;
    c->angle_req_mdeg = req;

    out = clamp_i32(req, -k->angle_max_mdeg, k->angle_max_mdeg);
    c->saturated = (out != req) ? 1U : 0U;

    step_lim = (int32_t)(((int64_t)k->slew_mdeg_per_s * (int32_t)dt_ms) / 1000);

    if (out > c->angle_cmd_mdeg + step_lim) {
        out = c->angle_cmd_mdeg + step_lim;
    } else if (out < c->angle_cmd_mdeg - step_lim) {
        out = c->angle_cmd_mdeg - step_lim;
    }

    c->angle_cmd_mdeg = out;
    return out;
}

/* ==========================================================================
 *                                遥测
 * ========================================================================== */

int32_t ball_v2_angle(const ball_v2_t *c) { return c->angle_cmd_mdeg; }
int32_t ball_v2_pos(const ball_v2_t *c)   { return c->x_filt_01mm; }
int32_t ball_v2_vel(const ball_v2_t *c)   { return c->v_est_mms; }
int32_t ball_v2_pred(const ball_v2_t *c)  { return c->x_pred_01mm; }
int32_t ball_v2_fb(const ball_v2_t *c)    { return c->fb_mdeg; }
int32_t ball_v2_pulse(const ball_v2_t *c) { return c->pulse_mdeg; }
uint8_t ball_v2_sat(const ball_v2_t *c)   { return c->saturated; }
