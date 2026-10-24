/**
  ******************************************************************************
  * @file    key.h
  * @brief   唯一按键（PA0）：短按 = 启动，运行中长按 = 停止
  *          按下 = 低电平（引脚内部上拉，按键另一端接 GND）
  *
  * 本文件属于用户代码，CubeMX 重新生成工程时不会被覆盖。
  ******************************************************************************
  */

#ifndef __KEY_H__
#define __KEY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define KEY_DEBOUNCE_MS   20U   /* 消抖时间 */
#define KEY_LONG_MS       1000U /* 长按判定时间（运行中长按 = 停止回平） */

void    key_init(void);
void    key_scan(void);            /* 每 1ms 调用一次 */

/* 当前是否按住 */
uint8_t key1_pressed(void);

/* 按下瞬间的事件，读一次就清掉 */
uint8_t key1_clicked(void);

/* 松手瞬间的事件，读一次就清掉 */
uint8_t key1_released(void);

#ifdef __cplusplus
}
#endif

#endif /* __KEY_H__ */
