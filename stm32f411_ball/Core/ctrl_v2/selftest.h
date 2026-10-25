/**
  ******************************************************************************
  * @file    selftest.h
  * @brief   开环参数辨识自检（临时诊断代码，不参与正常控制）
  *
  * 用法：把 BALL_SELFTEST_ENABLE 改成 1 → 重新编译烧录 → 打开 PA9 串口
  *       上电后自动跑三段实验，把串口输出存成 CSV。
  *       跑完把 BALL_SELFTEST_ENABLE 改回 0，恢复正常控制。
  *
  * 三段实验（全程开环，不跑控制律）：
  *   阶段1  静摩擦阈值扫描：0.5°→6.0° 逐档，找球刚动的最小角度
  *   阶段2  加速度辨识：固定角度保持 5 秒，输出 20ms 位置序列
  *   阶段3  延迟辨识：角度阶跃，输出 20ms 位置序列 + 首次位移时刻
  ******************************************************************************
  */

#ifndef BALL_SELFTEST_H
#define BALL_SELFTEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 1 = 启用自检（正常控制让位）；0 = 完全不编译进固件 */
#ifndef BALL_SELFTEST_ENABLE
#define BALL_SELFTEST_ENABLE   (0)
#endif

void selftest_init(void);
void selftest_poll(void);      /* 主循环里每轮调用，非阻塞 */

#ifdef __cplusplus
}
#endif

#endif /* BALL_SELFTEST_H */
