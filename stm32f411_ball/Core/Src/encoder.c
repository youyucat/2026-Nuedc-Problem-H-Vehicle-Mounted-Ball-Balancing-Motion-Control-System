/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   TIM2 硬件正交解码 + Z 相外部中断
  *
  * TIM2 是 32 位定时器，直接当计数器用，几乎不会溢出。
  * A/B 两相接 PA0 / PA1，走 TIM2_CH1 / TIM2_CH2，硬件 4 倍频解码。
  * Z 相接 PA2，走 EXTI2。默认按下降沿触发，编码器 Z 是高有效就改 main.c 里
  * MX_GPIO_Init 的 GPIO_MODE_IT_RISING。
  *
  * 本文件属于用户代码，CubeMX 重新生成工程时不会被覆盖。
  ******************************************************************************
  */

#include "encoder.h"

extern TIM_HandleTypeDef htim2;

static volatile uint8_t  s_z_flag = 0;
static volatile uint32_t s_z_cnt  = 0;
static volatile uint32_t s_z_last = 0;    /* 上一次 Z 触发的时间戳（消抖用） */

void encoder_init(void)
{
    s_z_flag = 0;
    s_z_cnt  = 0;
    s_z_last = 0;

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
}

void encoder_reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
}

int32_t encoder_get_count(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
}

uint8_t encoder_index_flag(void)
{
    uint8_t f = s_z_flag;
    s_z_flag = 0;
    return f;
}

uint32_t encoder_index_count(void)
{
    return s_z_cnt;
}

/* 由 EXTI2 中断调用 */
void encoder_z_isr(void)
{
    uint32_t now = HAL_GetTick();

    /* Z 相光耦过零时边沿会抖，一次过零可能弹出好几个中断把串口刷爆。
     * 一圈才一个 Z，100ms 内的重复触发一律当抖动丢弃。 */
    if ((now - s_z_last) < 100U) {
        return;
    }
    s_z_last = now;
    s_z_flag = 1;
    s_z_cnt++;
}
