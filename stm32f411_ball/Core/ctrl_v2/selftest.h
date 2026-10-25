/**
  ******************************************************************************
  * @file    selftest.h
  * @brief   命令角 → 真实倾角 标定测试（临时诊断代码，不参与正常控制）
  *
  * 用法：把 BALL_SELFTEST_ENABLE 改成 1 → 重新编译烧录 → 打开 PC6/PC7 串口
  *       上电后自动扫一遍命令角，每档停顿 4 秒供读水平仪。
  *       跑完把 BALL_SELFTEST_ENABLE 改回 0，恢复正常控制。
  *
  * 测试内容（全程开环，不跑控制律）：
  *   命令角 25/40/55/70/85 度五档，每档 摆到位 → 保持 4 秒 → 回平，
  *   由人工读水平仪记录真实倾角，事后拟合出「命令角 : 真实角」比例。
  *
  * 为什么不做摩擦/加速度/延迟辨识：
  *   T4 实测证明步进严格跟随命令，但水平仪显示真实倾角只有命令的约 1/12.5。
  *   在角度换算错 12.5 倍的前提下，所有基于该换算的参数都不可信，
  *   必须先把这个比例标定准，其余辨识才有意义。
  ******************************************************************************
  */

#ifndef BALL_SELFTEST_H
#define BALL_SELFTEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 1 = 启用自检（正常控制让位）；0 = 完全不编译进固件
 * 标定已完成，已关闭。需要复核标定时再改回 1。 */
#ifndef BALL_SELFTEST_ENABLE
#define BALL_SELFTEST_ENABLE   (0)
#endif

void selftest_init(void);
void selftest_poll(void);      /* 主循环里每轮调用，非阻塞 */

#ifdef __cplusplus
}
#endif

#endif /* BALL_SELFTEST_H */
