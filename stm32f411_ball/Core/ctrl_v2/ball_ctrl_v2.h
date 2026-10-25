/**
  ******************************************************************************
  * @file    ball_ctrl_v2.h
  * @brief   摆杆稳球控制 —— 从零重写版（不依赖任何 HAL，可在 PC 编译）
  *
  * 设计出发点：旧实现卡在「静摩擦推不动 / 推大了过冲」的极限环里，根因是
  * 回路延迟（视觉 20ms + 传输 + 控制 10ms + 步进斜率限幅）对双积分器伤害很大。
  * 于是本版把三件事显式建模，而不是当参数试：
  *
  *   1. 延迟补偿：用 (位置 + 速度·Td + ½·a·Td²) 把状态前推到「动作真正生效」的时刻
  *   2. 执行器可达性：角度命令始终经过斜率限幅，控制器知道自己在要一个够不到的角度
  *   3. 静摩擦管理：不是靠加大增益硬推，而是「准静止判定 → 最小可信脉冲 → 立即卸力」
  *
  * 单位约定（全整数，和旧工程一致，便于直接对接视觉帧）：
  *   位置 x     : 0.1 mm
  *   速度 v     : mm/s
  *   角度       : 0.001 度（mdeg）
  *   时间 dt    : ms
  *
  * 注意：本文件不 include 任何 HAL/STM32 头，PC 与目标平台共用同一份源码。
  ******************************************************************************
  */

#ifndef BALL_CTRL_V2_H
#define BALL_CTRL_V2_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 *                                配置
 * ========================================================================== */

typedef struct {
    /* ---- 机械与标定（必须先实测，不能抄理论值）----
     * 理论值 (5/7)·g ≈ 122 mm/s²/deg，旧工程实测标定为 180，两者差 47%，
     * 因此这里必须填系统辨识的实测值。 */
    int32_t gain_mm_s2_per_deg;

    /* ---- 回路延迟与执行器 ---- */
    int32_t loop_delay_ms;      /* 视觉出帧→动作生效 的总延迟估计 */
    int32_t slew_mdeg_per_s;    /* 步进斜率限幅，旧工程 60°/s = 60000 */
    int32_t angle_max_mdeg;     /* 近端角度上限 */

    /* ---- 状态反馈增益（由 gain/延迟/阻尼比反算，不是试出来的）---- */
    int32_t kp_mdeg_per_mm;
    int32_t kd_mdeg_per_mm_s;

    /* ---- 静摩擦管理 ---- */
    int32_t stick_v_thresh_mms;    /* |v| 低于此值视为「可能顶住」 */
    int32_t stick_hold_ms;         /* 准静止持续多久后发脉冲 */
    int32_t stick_deadband_01mm;   /* 懒区：误差小于此值时一律不动手（不发脉冲、不学偏置） */
    int32_t stick_pulse_mdeg;      /* 破静摩擦的**起始**可信角度（实测） */
    /* 自适应递增：同一个位置连续几次都推不动，说明这次遇到的静摩擦比
     * 预估大，下一次把脉冲加大这么多。球一动就清零回到起始值。
     * 实机上静摩擦随位置/清洁度变化，固定脉冲要么推不动、要么一脚踢飞。 */
    int32_t stick_pulse_step_mdeg;
    int32_t stick_ramp_ms;         /* 脉冲爬升时间 */
    int32_t stick_hold_plateau_ms; /* 脉冲保持时间 */
    int32_t stick_relax_ms;        /* 卸力时间 */

    /* ---- 死区与限幅 ---- */
    int32_t deadband_01mm;      /* 误差死区，避免端点抖动 */
    int32_t bias_max_mdeg;      /* 机械零位偏置上限（慢学习） */
    int32_t bias_learn_ms;      /* 偏置学习时间常数（越长越稳，越短越快） */

    /* ---- 视觉 --- */
    int32_t frame_period_ms;    /* 视觉标称帧周期，用于测速 */
} ball_v2_cfg_t;

/* 出厂默认参数（仿真里调出来的初值，上车需按实测 gain 重算 K）*/
void ball_v2_default_cfg(ball_v2_cfg_t *cfg);

/* ==========================================================================
 *                                状态
 * ========================================================================== */

typedef struct {
    const ball_v2_cfg_t *cfg;

    /* 输入侧 */
    int32_t  x_raw_01mm;        /* 最近一次视觉位置 */
    uint32_t last_x_ms;         /* 该位置的时刻 */
    int32_t  x_prev_01mm;       /* 前一帧视觉位置（跨帧差分测速用） */
    uint32_t prev_x_ms;         /* 前一帧时刻 */
    uint8_t  have_prev;         /* 前一帧有效 */
    uint32_t now_ms;
    uint8_t  have_x;

    /* 估计 */
    int32_t  x_filt_01mm;       /* 位置 EMA */
    int32_t  v_est_mms;         /* 帧差测速 + EMA */
    uint8_t  est_ready;

    /* 目标与偏置 */
    int32_t  xref_01mm;
    int32_t  bias_mdeg;         /* 机械零位慢学习 */
    uint32_t bias_acc_ms;

    /* 静摩擦状态机 */
    uint32_t quiet_ms;          /* 准静止累计 */
    uint32_t pulse_ms;          /* 脉冲已运行时长 */
    int32_t  pulse_dir;         /* +1 / -1 / 0 */
    int32_t  pulse_peak_mdeg;
    uint16_t stick_tries;       /* 连续没推动的次数，用于脉冲自适应递增 */

    /* 输出 */
    int32_t  angle_req_mdeg;    /* 反馈+脉冲+偏置 求和后（未限速）*/
    int32_t  angle_cmd_mdeg;    /* 斜率限幅后，真正下发 */
    uint8_t  saturated;

    /* 遥测 */
    int32_t  fb_mdeg;           /* 纯反馈项 */
    int32_t  pulse_mdeg;        /* 当前脉冲项 */
    int32_t  x_pred_01mm;       /* 延迟补偿后的预测位置 */
} ball_v2_t;

/* ==========================================================================
 *                                接口
 * ========================================================================== */

void    ball_v2_init(ball_v2_t *c, const ball_v2_cfg_t *cfg);
void    ball_v2_reset(ball_v2_t *c);     /* 一次清零，不怕漏字段 */

/* 喂一帧视觉结果（0.1mm）。未检出时不要调用，控制器会保持上次值。 */
void    ball_v2_feed(ball_v2_t *c, int32_t x_01mm, uint32_t now_ms);

/* 100Hz 调用。返回本次下发角度（mdeg）。 */
int32_t ball_v2_step(ball_v2_t *c, uint32_t now_ms);

/* 设定球目标位置（0.1mm） */
void    ball_v2_set_target(ball_v2_t *c, int32_t xref_01mm);

/* ---- 遥测 ---- */
int32_t ball_v2_angle(const ball_v2_t *c);
int32_t ball_v2_pos(const ball_v2_t *c);
int32_t ball_v2_vel(const ball_v2_t *c);
int32_t ball_v2_pred(const ball_v2_t *c);
int32_t ball_v2_fb(const ball_v2_t *c);
int32_t ball_v2_pulse(const ball_v2_t *c);
uint8_t ball_v2_sat(const ball_v2_t *c);

#ifdef __cplusplus
}
#endif

#endif /* BALL_CTRL_V2_H */
