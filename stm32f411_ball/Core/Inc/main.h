/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* 唯一按键 KEY1：PA0（短按 = 启动稳球；运行中长按 1 秒 = 停止回平）
   —— PA0 原来是编码器 A 相（TIM2_CH1），编码器随之退役：
      闭环稳球不依赖编码器（它只做过细分校准和 Z 相打印）。
      PA1（B相）/PA2（Z相）/PA3（原KEY2）引脚空出，硬件可拆。
   按下 = 低电平（内部上拉，按键另一端接 GND） */
#define KEY_FWD_Pin GPIO_PIN_0
#define KEY_FWD_GPIO_Port GPIOA
#define Z_Pin GPIO_PIN_2
#define Z_GPIO_Port GPIOA
#define DIR1_Pin GPIO_PIN_1
#define DIR1_GPIO_Port GPIOB
#define EN1_Pin GPIO_PIN_0
#define EN1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* ST1 / PWM / 编码器 A、B 是复用功能引脚，CubeMX 不会给它们生成宏，手动补上
   注意：编码器已退役（PA0 让给 KEY1），ENC_A/B 宏仅供 msp 里的旧配置引用，
   main.c 不再调用 encoder_init()，TIM2 编码器不会真正启动 */
#define ST1_Pin              GPIO_PIN_8     /* TIM1_CH1 步进脉冲 */
#define ST1_GPIO_Port        GPIOA
#define PWM_Pin              GPIO_PIN_6     /* TIM4_CH1 速度参考 PWM */
#define PWM_GPIO_Port        GPIOB
#define ENC_A_Pin            GPIO_PIN_0     /* 旧编码器 A（PA0 已让给 KEY1） */
#define ENC_A_GPIO_Port      GPIOA
#define ENC_B_Pin            GPIO_PIN_1     /* 旧编码器 B */
#define ENC_B_GPIO_Port      GPIOA

/* 板载 LED 心跳灯：默认关（PA5 没在 CubeMX 里配）。要用就改 1 并自己加一个 GPIO 输出脚 */
#define USE_STATUS_LED       0
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
