/**
  ******************************************************************************
  * @file    ball_lqr.c
  * @brief   H 题要求4：摆杆稳球 LQR 控制律（整数定点实现）
  *
  * 控制律：u = K1·e + K2·v + KI·∫e
  *   对象模型（球在杆上滚动，小角度）：ė = v，v̇ = -(5/7)·g·sin(u) ≈ -b·u
  *   b = (5/7)·g·π/180 ≈ 122.3 mm/s²/°（g=9810mm/s²）
  *
 * 增益来源（详见 README_LQR.md）：
 *   K1/K2：双积分器 LQR 的 Riccati 闭式最优解（Q=[625,25], R=1）：
 *     k1 = √Q_e / b = 25/122.3     = 0.2044 °/mm
 *     k2 = √(Q_v + 2√Q_e) / b = √75/122.3 = 0.0708 °/(mm/s)
 *     等效二阶：ωn = √(b·k1) = 5 rad/s，ζ = b·k2/(2ωn) = 0.866
 *     （对比滑模 PD 等效 ζ≈0.59，阻尼显著更优）
  *   KI：积分增广 ki = 0.06 °/(mm·s)。3 状态完整 LQI 在本对象上存在
  *     "积分通道拉低主阻尼"的结构性权衡（网格扫描 500+ 组无合格点），
  *     故采用 2 态 LQR 最优解 + 低权重外部积分（工程标准做法）。
  *
  * 抗饱和：条件积分——冻结输出仍在限幅内且误差非零才积累；积分项另有
  * ±8° 输出限幅。重捕获/丢球回平/重新启动时 ball_lqr_reset() 清积分。
  *
  * 本文件属于用户代码，CubeMX 重新生成工程时不会被覆盖。
  ******************************************************************************
  */

#include "ball_lqr.h"

/* ===================== 内部状态 ===================== */

/* 积分累加器，单位 0.1mm·ms（err01 × dt_ms 累加）。
 * 量级参考：偏差 20mm 持续 1s ≈ 2e6？——err01=200 × 1000ms = 2e5，
 * 产生积分输出 6×2e5/1000 = 1200 mdeg = 1.2°。 */
static int32_t s_i_acc = 0;

/* ===================== 内部小工具 ===================== */

static int32_t lqr_abs32(int32_t v)
{
    return (v >= 0) ? v : -v;
}

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ===================== 接口实现 ===================== */

void ball_lqr_reset(void)
{
    s_i_acc = 0;
}

int32_t ball_lqr_update(int32_t err01, int32_t v_mms, uint32_t dt_ms,
                        int32_t limit_mdeg)
{
    int32_t u_p, u_d, u_i, u_frozen;

    /* P 项：u1 = 2044 × err01 / 100  mdeg（k1=0.2044°/mm） */
    u_p = (int32_t)(((int64_t)BALL_LQR_KP_NUM * err01) / 100);

    /* D 项：u2 = 708 × v / 10  mdeg（k2=0.0708°/(mm/s)） */
    u_d = (int32_t)(((int64_t)BALL_LQR_KD_NUM * v_mms) / 10);

    /* 抗饱和判定：用冻结积分的总输出判断是否允许继续积分 */
    u_i      = (int32_t)(((int64_t)BALL_LQR_KI_NUM * s_i_acc) / 1000);
    u_frozen = u_p + u_d + u_i;

    /* 条件积分：误差非零（死区内冻结，驻留防抖）且输出未贴限幅才积累 */
    if ((err01 != 0) && (lqr_abs32(u_frozen) < limit_mdeg)) {
        s_i_acc = (int32_t)((int64_t)s_i_acc + (int64_t)err01 * (int64_t)dt_ms);
        s_i_acc = clamp_i32(s_i_acc, -BALL_LQR_I_MAX, BALL_LQR_I_MAX);
    }

    /* 重算积分项（本拍刚更新过）并限幅后合成输出 */
    u_i = (int32_t)(((int64_t)BALL_LQR_KI_NUM * s_i_acc) / 1000);
    u_i = clamp_i32(u_i, -BALL_LQR_KI_MAX_MDEG, BALL_LQR_KI_MAX_MDEG);

    return u_p + u_d + u_i;
}

int32_t ball_lqr_integral_mdeg(void)
{
    return clamp_i32((int32_t)(((int64_t)BALL_LQR_KI_NUM * s_i_acc) / 1000),
                     -BALL_LQR_KI_MAX_MDEG, BALL_LQR_KI_MAX_MDEG);
}
