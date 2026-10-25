/**
  ******************************************************************************
  * @file    ball_lqr.h
  * @brief   H 题要求4：摆杆稳球 LQR 控制律（可选，BALL_USE_LQR=1 启用）
  *
  * 控制律：u = K1·e + K2·v + KI·∫e（LQR 状态反馈 + 积分增广）
  *   - K1/K2 为双积分器模型 LQR 的 Riccati 闭式最优解（ζ=0.866）
  *   - KI 为低权重积分增广，条件积分抗饱和，替代回零偏学习消稳态差
  *   - 输入原始误差（真实状态反馈），静摩擦脱困脉冲仍由 ball_ctrl.c 叠加
  *   - 原理 / 增益推导 / 仿真验证 / 整定指引见同目录 README_LQR.md
  *
  * 定点约定（全整数，无浮点）：
  *   err01  球偏差，0.1mm；v  球速，mm/s；dt_ms 控制周期，ms
  *   积分累加器 s_i_acc 单位 0.1mm·ms（= err01 × dt_ms 累加）
  *   输出 mdeg：u = KP·err01/100 + KD·v/10 + KI_NUM·s_i_acc/1000
  *
  * 本文件属于用户代码，CubeMX 重新生成工程时不会被覆盖。
  ******************************************************************************
  */

#ifndef __BALL_LQR_H__
#define __BALL_LQR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ==========================================================================
 *                  LQR 增益配置区（整定改这里，推导见 README_LQR.md）
 * ========================================================================== */

/* K1：位置增益。k1 = 0.2044 °/mm（Q_e=625, R=1 的 LQR 闭式解）。
 * 定点：u1[mdeg] = 2044 × err01 / 100。回中慢 → 加大；振荡 → 减小。 */
#define BALL_LQR_KP_NUM         (2044)   /* = k1 × 10000 */

/* K2：速度（阻尼）增益。k2 = 0.0708 °/(mm/s)（同组 LQR 闭式解）。
 * 定点：u2[mdeg] = 708 × v / 10。过冲加大；反应迟钝减小。 */
#define BALL_LQR_KD_NUM         (708)    /* = k2 × 10000 */

/* KI：积分增益。ki = 0.06 °/(mm·s)（低权重积分增广，Routh 裕度 24）。
 * 定点：u3[mdeg] = 6 × s_i_acc / 1000，s_i_acc 单位 0.1mm·ms。
 * 稳态差消得慢 → 加大；超调变大 → 减小。 */
#define BALL_LQR_KI_NUM         (6)      /* = ki × 100 */

/* 积分项输出限幅：等效 ±8°。盖过常驻机械偏差所需的最大积分输出。 */
#define BALL_LQR_KI_MAX_MDEG    (8000)

/* 积分累加器限幅（0.1mm·ms）：= KI_MAX_MDEG × 1000 / KI_NUM，保持一致 */
#define BALL_LQR_I_MAX          (1333333)

/* ==========================================================================
 *                                 接口
 * ========================================================================== */

/* 清积分状态（启动/重捕获/丢球回平时由 ball_ctrl.c 调用） */
void    ball_lqr_reset(void);

/* LQR 控制律一步更新。
 * @param err01mm    球偏差（球位置-目标），0.1mm（中心死区已处理，可为 0）
 * @param v_mms      球速度估计，mm/s
 * @param dt_ms      距上次更新的实测间隔，ms（积分按实际间隔折算）
 * @param limit_mdeg 当前角度限幅（抗饱和判定用，如 ±10°/±14°）
 * @return 控制输出 mdeg（含积分项；外层仍需按 limit_mdeg 限幅） */
int32_t ball_lqr_update(int32_t err01mm, int32_t v_mms, uint32_t dt_ms,
                        int32_t limit_mdeg);

/* 遥测：当前积分项输出，mdeg（调试用） */
int32_t ball_lqr_integral_mdeg(void);

#ifdef __cplusplus
}
#endif

#endif /* __BALL_LQR_H__ */
