/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "stepper.h"
#include "key.h"
#include "ball_ctrl.h"
#include "selftest.h"          /* 开环参数辨识（BALL_SELFTEST_ENABLE=0 时为空） */
/* 编码器已退役（PA0 让给 KEY1），不再 include/调用 encoder 模块 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define USE_DEBUG_UART   1     /* 不用串口就改成 0 */
#define USE_DEBUG_UART6  1     /* 板载调试口在 PC6/PC7(USART6) 时置 1 */

/* 第二路调试串口：实际板子的调试口接在 PC6/PC7（USART6，AF8），
 * 而 USART1 的 PA9/PA10 是与 MaixCam + 车主控的协议链路。
 *
 * 调试口既发日志也收命令：收到的字节注入 ball_ctrl 的解析环形缓冲，
 * 这样台架调试时直接从这个口发 M/Z/R 命令即可，不用挪线到 PA10。 */
UART_HandleTypeDef huart6;
uint8_t            dbg_rx_byte;   /* USART6 单字节中断接收缓冲 */

/* ==================== H题 要求4：行驶稳球任务 ====================
 * 小车置于 A 点、钢球置于中心 O，按键启动后沿黑线顺时针行驶通过 B
 * （AB 时间 <= 8s 由车主控负责），行驶全程摆杆把球稳定在中心 O，
 * 误差绝对值 <= 1cm（本工程负责摆杆这一半）。
 *
 * 控制方案（移植自 MSPM0 成熟参考工程 ball_balance）：滑模 PD
 *   + 回零偏学习 + 静摩擦脱困脉冲 + 车体加速度前馈，
 *   丢球保持 600ms 后限速回平（不退出任务，球回视野自动继续）
 *   W1 减速告警 / W2 回平停车告警
 *
 *  唯一按键 PA0：短按 = 启动稳球闭环；运行中长按 1 秒 = 停止回平
 *  USART1 RX   收视觉模块 B 帧（球位置/速度/置信度）和车主控 A 帧
 *  USART1 TX   调试打印 + 给车主控的 W 告警帧（可选监听）
 */
typedef enum {
    MISSION_IDLE = 0,       /* 等待按键启动，摆杆保持在水平零位 */
    MISSION_RUN           /* 要求4进行中：稳球闭环工作 */
} mission_t;

static mission_t s_mission = MISSION_IDLE;

static uint32_t s_last_tick = 0;    /* 1ms 任务节拍（按键扫描 + 电机斜坡） */
static uint32_t s_last_ctrl = 0;    /* 10ms 任务节拍（稳球控制律 + 帧解析） */
static uint32_t s_last_log  = 0;    /* 500ms 遥测节拍 */

/* 唯一按键 PA0 的按下/松开计时（长按判定用） */
static uint32_t s_key_down_tick = 0;
static uint8_t  s_key_holding   = 0;

static uint32_t s_last_hb = 0;      /* 1s 待机心跳节拍 */

/* 按键确认"点头"：短按启动前摆杆抬 3° 再回平，肉眼可见 = 按键/使能/脉冲链路全通。
 * 没接 MaixCam 时闭环本身不推动摆杆（无球数据只保持水平），
 * 如果没有这个动作，台架上"按了没反应"其实可能就是正常现象——用它一按便知。 */
typedef enum {
    ACK_NONE = 0,       /* 无动作 */
    ACK_UP,             /* 抬起 3° 进行中 */
    ACK_BACK            /* 回平进行中 */
} ack_state_t;
static ack_state_t s_ack = ACK_NONE;
static uint8_t     s_start_after_ack = 0;   /* 点头完成后要启动稳球闭环 */

/* 上电自检：上电 1 秒后摆杆自动点头一次（不需要按键），用来把
 * "脉冲链路不通"和"按键信号没进来"两种故障区分开——保持力矩只证明
 * 使能有效，不证明 STEP 脉冲有效，必须亲眼看摆杆动一次才算数。 */
static uint32_t s_boot_tick     = 0;        /* 自检点头的时间基准 */
static uint8_t  s_boot_nod_done = 0;        /* 自检只做一次 */
static int32_t  s_ack_steps     = 0;        /* 当前点头步数：启动确认 3°，运行/退出示意 2° */
static uint8_t  s_exit_nod_req  = 0;        /* 任务退出（超时/长按）后要点头示意一次 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
static void dbg_printf(const char *fmt, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void dbg_printf(const char *fmt, ...)
{
#if USE_DEBUG_UART
    char    buf[160];
    int     n;
    va_list ap;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n > 0) {
        if (n > (int)sizeof(buf) - 1) n = (int)sizeof(buf) - 1;
#if USE_DEBUG_UART6
        /* 调试日志只走 PC6/PC7（USART6）。
         * PA9/PA10 是与 MaixCam + 主控的协议链路，绝不能混入 ASCII 调试文本，
         * 否则会污染 12 字节二进制帧。协议帧（W1/W0/W2）由 ball_ctrl 单独发。 */
        HAL_UART_Transmit(&huart6, (uint8_t *)buf, (uint16_t)n, 100);
#else
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)n, 100);
#endif
    }
#else
    (void)fmt;
#endif
}

#if USE_DEBUG_UART6
/* 板载调试口在 PC6/PC7 = USART6_TX/RX（AF8）。
 * 手动配 GPIO+时钟，再交给 HAL_UART_Init；MspInit 是弱函数，为空即可。 */
static void dbg_uart6_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_USART6_CLK_ENABLE();

    gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &gpio);

    huart6.Instance          = USART6;
    huart6.Init.BaudRate     = 115200;
    huart6.Init.WordLength   = UART_WORDLENGTH_8B;
    huart6.Init.StopBits     = UART_STOPBITS_1;
    huart6.Init.Parity       = UART_PARITY_NONE;
    huart6.Init.Mode         = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    (void)HAL_UART_Init(&huart6);

    /* 与 USART1 同优先级，开启接收中断，才能从调试口发命令 */
    HAL_NVIC_SetPriority(USART6_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
    (void)HAL_UART_Receive_IT(&huart6, &dbg_rx_byte, 1U);
}
#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
#if USE_DEBUG_UART6
  dbg_uart6_init();                     /* 调试口 PC6/PC7，必须最先初始化 */
#endif
  stepper_init();
  key_init();
  ball_ctrl_init();                     /* 含挂起 USART1 接收（摄像头/车主控数据） */

  /* KEY1 已从 PC13 挪到 PA0（原编码器 A 相）。
   * MX_TIM2_Init 的编码器 MspInit 会把 PA0/PA1 配成 TIM2 复用，
   * 这里在所有 MX 初始化之后把 PA0 改回"输入上拉"按键，保证按键可靠；
   * TIM2 编码器不再 Start（encoder_init 已移除），计数器闲置无影响。 */
  {
      GPIO_InitTypeDef key_gpio = {0};
      key_gpio.Pin  = KEY_FWD_Pin;
      key_gpio.Mode = GPIO_MODE_INPUT;
      key_gpio.Pull = GPIO_PULLUP;
      key_gpio.Speed = GPIO_SPEED_FREQ_LOW;
      HAL_GPIO_Init(KEY_FWD_GPIO_Port, &key_gpio);
  }

  /* USART1 接收中断：视觉模块 B 帧 / 车主控 A 帧从这里进来 */
  HAL_NVIC_SetPriority(USART1_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  stepper_set_run_speed(STEPPER_SERVO_SPEED);   /* 稳球微动速度 */
  stepper_enable(1);                    /* 使能驱动器，电机上电保持力矩 */

  dbg_printf("\r\n============================================\r\n");
  dbg_printf("[STEPPER] MOTER / STM32F411RET6 ready\r\n");
  dbg_printf("[TASK ] H题 要求4：滑模PD+前馈 行驶稳球（误差<=1cm）\r\n");
  dbg_printf("[KEY  ] PA0 唯一按键：短按 = 启动（先点头3°确认）\r\n");
  dbg_printf("        运行中短按 = 点头2°示意在跑 | 长按1秒 = 停止回平\r\n");
  dbg_printf("        任务结束（超时/停止）后自动小点头示意\r\n");
  dbg_printf("[UART1] 115200 8N1  RX = MaixCam 二进制帧(0xAA55,12B,CRC16)\r\n");
  dbg_printf("        接线: MaixCam A19(TX) -> PA10(RX)；帧含 0.1mm位置/\r\n");
  dbg_printf("        mm/s速度/置信度/标志位，50Hz 恒发\r\n");
  dbg_printf("        车主控 A,<mm/s2> 帧共线（串1k电阻）或台架单用\r\n");
  dbg_printf("        M,<mdeg>手动摆角 Z,水平清零 R,<x0.1mm>球目标(要求6)\r\n");
  dbg_printf("        TX告警帧: W1丢球减速 W0恢复 W2超时停车\r\n");
  dbg_printf("[TIP  ] 上电前先目视调平摆杆：上电位置=水平零位\r\n");
  dbg_printf("        也可运行中发 Z 重新定义零位\r\n");
  dbg_printf("[TEST ] 上电1秒摆杆自动点头 = 脉冲链路自检\r\n");
  dbg_printf("        再短按PA0有第二次点头 = 按键也正常\r\n");
  dbg_printf("============================================\r\n");

  s_last_tick = HAL_GetTick();
  s_last_ctrl = s_last_tick;
  s_last_log  = s_last_tick;
  s_boot_tick = s_last_tick;

  /* 自检放在横幅之后：横幅出来了说明固件已跑起来，
   * 再看下面有没有 SELFTEST START，就能区分「固件没跑」和「自检起不来」 */
#if BALL_SELFTEST_ENABLE
  selftest_init();                      /* 开环参数辨识：上电自动跑三段实验 */
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* ---------------- 1ms 任务：按键扫描 + 电机速度斜坡 ---------------- */
    if ((uint32_t)(now - s_last_tick) >= 1U) {
        s_last_tick = now;
        key_scan();
        stepper_tick_1ms();

#if BALL_SELFTEST_ENABLE
        selftest_poll();              /* 开环参数辨识状态机（非阻塞） */
#endif

        /* 按键点头状态机：抬起到位 -> 回平 -> （按按键意图）启动闭环 */
        if (s_ack == ACK_UP && !stepper_is_busy()) {
            s_ack = ACK_BACK;
            stepper_servo_update(stepper_position() - s_ack_steps);
        } else if (s_ack == ACK_BACK && !stepper_is_busy()) {
            s_ack = ACK_NONE;
            stepper_set_run_speed(STEPPER_SERVO_SPEED);     /* 恢复稳球微动速度 */
            if (s_start_after_ack) {
                s_start_after_ack = 0;
                ball_ctrl_start();      /* 记录水平零位、清控制状态、开启闭环 */
                s_mission = MISSION_RUN;
                dbg_printf("[MISSION] 要求4 启动：稳球闭环开启\r\n");
                if (ball_ctrl_frame_count() == 0) {
                    dbg_printf("[MISSION] 提示：尚未收到 MaixCam 帧(fr=0)，"
                               "摆杆先保持水平，球回视野自动开始稳球\r\n");
                }
            }
        }

        /* 上电自检点头：1 秒后自动点一下，不按键也能验证"使能+脉冲链路"。
         * 必须放在点头状态机之后：刚排队动作时 busy 仍为 0（脉冲要到下一
         * tick 才起步），放前面会被同一 tick 误判成"抬到位"直接翻到回平。 */
        if (!s_boot_nod_done && s_mission == MISSION_IDLE && s_ack == ACK_NONE &&
            (uint32_t)(now - s_boot_tick) >= 1000U && !stepper_is_busy()) {
            s_boot_nod_done = 1;
            s_ack_steps = STEPPER_ACK_STEPS;
            s_ack = ACK_UP;             /* s_start_after_ack 保持 0：点完头不启动 */
            stepper_set_run_speed(STEPPER_ACK_SPS);
            stepper_servo_update(stepper_position() + s_ack_steps);
            dbg_printf("[SELFTEST] 摆杆点头自检：能动 = 使能+脉冲链路正常\r\n");
        }

        /* 任务退出示意：长按停止 之后小点头一次，
         * 让"运行中 -> 空闲"的状态切换肉眼可见（stop_level 的回平动作走完才触发）
         * （丢球不再退出任务，只回平待球，所以点头只在手动停止时出现） */
        if (s_exit_nod_req && s_mission == MISSION_IDLE && s_ack == ACK_NONE &&
            !stepper_is_busy()) {
            s_exit_nod_req = 0;
            s_ack_steps = STEPPER_ACK_RUN_STEPS;
            s_ack = ACK_UP;             /* s_start_after_ack 保持 0 */
            stepper_set_run_speed(STEPPER_ACK_SPS);
            stepper_servo_update(stepper_position() + s_ack_steps);
            dbg_printf("[MISSION] 已退出任务（点头示意，短按可重新启动）\r\n");
        }
    }

    /* ---------------- 10ms 任务：要求4 稳球控制律 ---------------- */
    if ((uint32_t)(now - s_last_ctrl) >= 10U) {
        s_last_ctrl = now;
        ball_ctrl_uart_poll();      /* 解析 B/A/M/Z 帧 */
        ball_ctrl_tick_10ms();      /* 滑模PD + 偏置/脉冲/前馈 -> 步进伺服微动 */
    }

    /* ---------------- 故障转移（保留）：参考设计丢球不退出任务，
     * ball_ctrl_fault() 恒 0，此块正常不会触发，留作兜底 ---------- */
    if (s_mission == MISSION_RUN && ball_ctrl_fault()) {
        s_mission = MISSION_IDLE;
        s_exit_nod_req = 1;             /* 回平完成后点头示意"已退出" */
        dbg_printf("[MISSION] 丢球超时：已回平退出任务（W2 已发，建议车主控停车）\r\n");
    }

    /* ---------------- 按键任务：PA0 短按启动 / 长按1秒停止 ---------------- */
    if (key1_clicked()) {                          /* 按下瞬间：记录起点 */
        s_key_down_tick = now;
        s_key_holding   = 1;
    }
    if (s_key_holding && key1_released()) {        /* 松手：按住时长判定意图 */
        uint32_t held = now - s_key_down_tick;

        s_key_holding = 0;

        if (s_mission == MISSION_IDLE) {
            /* 空闲：短按/长按都 = 先点头确认，再启动闭环 */
            if (stepper_is_busy() || s_ack != ACK_NONE) {
                dbg_printf("[MISSION] 电机未停稳，稍后再按\r\n");
            } else {
                s_start_after_ack = 1;
                s_ack_steps = STEPPER_ACK_STEPS;
                s_ack = ACK_UP;
                stepper_set_run_speed(STEPPER_ACK_SPS);
                stepper_servo_update(stepper_position() + s_ack_steps);
                dbg_printf("[MISSION] 按键确认：摆杆点头...\r\n");
            }
        } else if (held >= KEY_LONG_MS) {
            /* 运行中长按：停止回平（短按只示意，防比赛误触停机） */
            ball_ctrl_stop_level();
            s_mission = MISSION_IDLE;
            s_exit_nod_req = 1;         /* 回平完成后点头示意"已退出" */
            dbg_printf("[MISSION] 停止：摆杆回平\r\n");
        } else if (s_ack == ACK_NONE && !stepper_is_busy()) {
            /* 运行中短按：点头 2° 示意"任务在跑"——没有它，运行态和空闲态
             * 肉眼无法区分，会被当成"按了没反应"（比赛时不要按，会轻微扰动球） */
            s_ack_steps = STEPPER_ACK_RUN_STEPS;
            s_ack = ACK_UP;             /* s_start_after_ack 保持 0 */
            stepper_set_run_speed(STEPPER_ACK_SPS);
            stepper_servo_update(stepper_position() + s_ack_steps);
            dbg_printf("[MISSION] 运行中：点头示意（长按1秒停止）\r\n");
        } else {
            dbg_printf("[MISSION] 运行中：长按1秒停止\r\n");
        }
    }

    /* ---------------- 500ms 遥测（运行时打印控制各项，方便现场整定） ------- */
    if ((uint32_t)(now - s_last_log) >= 500U) {
        s_last_log = now;

        if (s_mission == MISSION_RUN) {
            int32_t b  = ball_ctrl_ball_01mm();
            int32_t v  = ball_ctrl_vel_mms();
            int32_t rf = ball_ctrl_xref_01mm();
            int32_t t  = ball_ctrl_angle_mdeg();
            int32_t fb = ball_ctrl_fb_mdeg();
            int32_t ff = ball_ctrl_ff_mdeg();
            int32_t a  = ball_ctrl_accel();
            uint8_t c  = ball_ctrl_conf();

            dbg_printf("[RUN] x=%+ld.%01ld v=%+ldmm/s c=%d ref=%+ld.%01ld | "
                       "fb=%+ld ff=%+ld cmd=%+ld.%03lddeg acc=%+ld "
                       "sat=%d ok=%d cal=%d fr=%lu ce=%lu pos=%ld\r\n",
                       (long)(b / 10), (long)labs(b % 10),
                       (long)v, (int)c,
                       (long)(rf / 10), (long)labs(rf % 10),
                       (long)fb, (long)ff,
                       (long)(t / 1000), (long)labs(t % 1000),
                       (long)a,
                       (int)ball_ctrl_sat(),
                       (int)ball_ctrl_ball_ok(),
                       (int)ball_ctrl_vision_cal(),
                       (unsigned long)ball_ctrl_frame_count(),
                       (unsigned long)ball_ctrl_crc_err(),
                       (long)stepper_position());
        }
    }

    /* ---------------- 1s 待机心跳（排查"没力矩/按键没反应"用） ------------- */
    /* en_pin  : PB0 实际电平，1=固件已使能驱动器（若仍无力矩查驱动器侧）
     * key_raw : PA0 原始电平，1=未按(上拉高) 0=按下接地 —— 按键不动它说明接法有问题
     * key     : 消抖后的按键状态；fr/ce : 视觉帧计数（接了 MaixCam 才会涨）   */
    if ((uint32_t)(now - s_last_hb) >= 1000U) {
        s_last_hb = now;

        if (s_mission == MISSION_IDLE) {
            dbg_printf("[IDLE] en_pin=%d key_raw=%d key=%d busy=%d fr=%lu ce=%lu\r\n",
                       (int)HAL_GPIO_ReadPin(EN1_GPIO_Port, EN1_Pin),
                       (int)HAL_GPIO_ReadPin(KEY_FWD_GPIO_Port, KEY_FWD_Pin),
                       (int)key1_pressed(),
                       (int)stepper_is_busy(),
                       (unsigned long)ball_ctrl_frame_count(),
                       (unsigned long)ball_ctrl_crc_err());
        }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  *
  * HSI 16MHz -> PLLM=8, PLLN=100, PLLP=2 -> SYSCLK 100MHz
  * APB1 = 50MHz, APB2 = 100MHz；两个 APB 上的定时器时钟都是 100MHz
  * 板子上有晶振的话，改成 HSE 再调 PLLM 即可（stepper.c 会自动适配时钟）
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  *
  * STEP 脉冲发生器：PSC=99 -> 计数频率 1MHz，改 ARR 就是改脉冲频率
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 99;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  *
  * 编码器接口模式：A/B 两相都计数 -> 4 倍频；32 位计数器基本不会溢出
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  *
  * "PWM" 脚输出：10kHz 固定频率，占空比随速度变
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 99;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EN1_Pin DIR1_Pin */
  GPIO_InitStruct.Pin = EN1_Pin|DIR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Z_Pin */
  GPIO_InitStruct.Pin = Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Z_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_FWD_Pin (PA0, 唯一按键：短按启动/长按停止) */
  GPIO_InitStruct.Pin = KEY_FWD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_FWD_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
