/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   编码器 A/B/Z 读取（TIM2 硬件正交解码 + Z 相外部中断）
  *
  * 本文件属于用户代码，CubeMX 重新生成工程时不会被覆盖。
  ******************************************************************************
  */

#ifndef __ENCODER_H__
#define __ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* 编码器一圈对应多少计数。TIM2 是 4 倍频，所以 = 编码器线数 × 4
 * 常见步进闭环编码器 1000 线 → 4000 计数/圈
 * 不确定就先按默认跑，看串口打印的实测角度再回来改。
 */
#define ENCODER_COUNTS_PER_REV    4000

void     encoder_init(void);
void     encoder_reset(void);
int32_t  encoder_get_count(void);

/* Z 相（零位）信号 */
uint8_t  encoder_index_flag(void);    /* 读到 1 表示 Z 相来过一次（读后自动清零） */
uint32_t encoder_index_count(void);   /* Z 相累计来了多少次 */

void     encoder_z_isr(void);         /* 供 EXTI2 中断里调用，用户不用管 */

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H__ */
