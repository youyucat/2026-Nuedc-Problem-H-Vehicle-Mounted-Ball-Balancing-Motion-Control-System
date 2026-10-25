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

    /* 回路延迟：视觉 20ms 出帧 + 传输 5~10 + 控制 10 → 取 50ms 量级 */
    cfg->loop_delay_ms   = 50;
    cfg->slew_mdeg_per_s = 60000;   /* 60°/s */
    cfg->angle_max_mdeg  = 10000;   /* ±10° */

    /* 增益：按「临界阻尼 + 延迟补偿后」给初值，仿真里再细化 */
    cfg->kp_mdeg_per_mm   = 80;
    cfg->kd_mdeg_per_mm_s = 70;

    /* 静摩擦管理 */
    cfg->stick_v_thresh_mms   = 8;
    cfg->stick_hold_ms        = 150;
    cfg->stick_pulse_mdeg     = 2500;
    cfg->stick_ramp_ms        = 80;
    cfg->stick_hold_plateau_ms = 120;
    cfg->stick_relax_ms       = 100;

    /* 死区与偏置 */
    cfg->deadband_01mm = 8;
    cfg->bias_max_mdeg = 2000;
    cfg->bias_learn_ms = 1500;

    cfg->frame_period_ms = 20;
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
                c->pulse_dir  = dir;
                c->pulse_ms   = 0;
                /* 行程越远，脉冲略大一些，但不超过上限的 60% */
                c->pulse_peak_mdeg = clamp_i32(
                    k->stick_pulse_mdeg + abs_i32(err_pred) / 4,
                    k->stick_pulse_mdeg,
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
