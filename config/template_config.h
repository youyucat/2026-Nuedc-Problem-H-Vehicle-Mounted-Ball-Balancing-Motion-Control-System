#ifndef TEMPLATE_CONFIG_H
#define TEMPLATE_CONFIG_H

#include <stdint.h>

/*
 * 循迹专用工程的唯一行为参数表（双芯架构中的「循迹芯片」）。
 *
 * 全部数值都是源工程 empty_LP_MSPM0G3507_nortos_ticlang 上跑通的那一套，
 * 逐字保留，只删掉了属于「另一颗芯片」的 IMU / OLED / 步进 / 球平衡 /
 * 上位机协议参数。改行为就改这里，别去改 bsp/ 里的引脚表。
 *
 * 固件标识（源工程打在串口横幅上，本工程没有串口，仅作版本备注）：
 *   LF_ONLY_V1_20261024  —— 由 LF_V5_IRQ_80M_20260811 抽出的循迹分支
 */

/* ---------- 灰度 8 路（MUX + 单路 ADC） ---------- */
#define TEMPLATE_SENSOR_CHANNELS        8U
#define TEMPLATE_SENSOR_MAX_CHANNELS    32U
#define TEMPLATE_ADC_MAXIMUM            4095U
#define TEMPLATE_ADC_SAMPLE_TIME_CYCLES 64U
/* 同步兜底路径的平均样本数（启动/诊断用；控制环走中断异步帧，见 board.c） */
#define TEMPLATE_ADC_AVERAGE_SAMPLES    8U
/* MUX 地址线切换后的建立等待（空转周期）。 */
#define TEMPLATE_MUX_SETTLE_CYCLES      4000U
/* 控制拍 5 ms = 200 Hz：外环（循迹 PD）与内环（轮速 PI）同一个节拍。 */
#define TEMPLATE_CONTROL_PERIOD_MS      5U

/* ---------- 时钟 / PWM ---------- */
/* MCLK 80 MHz（SYSOSC -> SYSPLL），见 line_follow_mspm0g3507.syscfg。
 * BUS_CLOCK_HZ 是忙等延时用的 CPU 频率；TIMER_CLOCK_HZ 是定时器实际
 * 时钟（BUSCLK = ULPCLK = MCLK/2 = 40 MHz）。 */
#define TEMPLATE_BUS_CLOCK_HZ           80000000U
#define TEMPLATE_TIMER_CLOCK_HZ         40000000U
#define TEMPLATE_PWM_FREQUENCY_HZ       8000U
#define TEMPLATE_PWM_PERIOD_COUNTS      5000U

/* ---------- 灰度标定 ---------- */
/* 黑白场最小差值：低于它判为该通道校准失败。本工程没有串口/按键校准界面，
 * 未标定时 line_processing 用缺省阈值 2047（黑比白低）。要写死实测值，
 * 在 app/line_task.c 里调 line_processing_apply_points(black, white)。 */
#define TEMPLATE_CAL_MIN_SPAN           100U

/* ---------- 循迹外环（PD）默认参数 ---------- */
/* 已验证的一组：基础速度 260、Kp 0.12、Kd 0.13（千分之一为单位）。 */
#define TEMPLATE_DEFAULT_BASE_OUTPUT    260
#define TEMPLATE_DEFAULT_KP_MILLI       120
#define TEMPLATE_DEFAULT_KD_MILLI       130
/* 打角限幅：左右目标速度之差的一半，最大 120 mm/s。 */
#define TEMPLATE_LINE_MAX_CORRECTION_MM_S 120

/* ---------- 丢线恢复 ---------- */
/* 先保持最后有效目标 HOLD_MS，然后进入左右交替摇头扫描（每 SWEEP_MS 换向，
 * 幅度随失线时间线性增大），直到重新捕线或超过 STOP_MS 超时停车。 */
#define TEMPLATE_LINE_LOST_HOLD_MS       80U
#define TEMPLATE_LINE_LOST_SWEEP_MS     150U
#define TEMPLATE_LINE_LOST_STOP_MS     1500U
#define TEMPLATE_LINE_RECOVER_MIN_CORRECTION  60
#define TEMPLATE_LINE_RECOVER_MAX_CORRECTION 340

/* ---------- 直线蛇形抑制 ---------- */
/* 线误差一阶低通（EMA，百分比）+ 死区。8 路灰度在离散位置间抖动时，D 项会
 * 把噪声放大成来回修正；平滑 + 死区后直线保持直行。 */
#define TEMPLATE_LINE_ERROR_EMA_NUM      55
#define TEMPLATE_LINE_ERROR_DEADZONE     12

/* ---------- 堵转检测 ---------- */
#define TEMPLATE_LINE_STALL_DETECT_MS   180U

/* ---------- 运行时调参上限（line_task_set_gains 的入参校验） ---------- */
#define TEMPLATE_CONTROLLER_BASE_MAX    800
#define TEMPLATE_CONTROLLER_KP_MAX      2000
#define TEMPLATE_CONTROLLER_KD_MAX      4000

/* ---------- 上电起步 ---------- */
/* 上电后先等这么久再进入 RUNNING（期间电机零输出），给灰度首帧、电源、
 * 另一颗芯片留出稳定时间。上电车若不在线上，line_task 会原地等待看到线
 * 才开始走（见 app/line_task.c 的 s_had_valid_line）。 */
#define TEMPLATE_STARTUP_HOLD_MS       1000U

/* ---------- 硬件方向修正（换电机/换接线就改这里） ---------- */
#define TEMPLATE_MOTOR_OUTPUTS_SWAPPED   1U
#define TEMPLATE_LEFT_MOTOR_INVERTED    0U
#define TEMPLATE_RIGHT_MOTOR_INVERTED   0U
/* 左右电机出力配平（千分比）：直线跑偏时先调这两个再动 Kp。 */
#define TEMPLATE_DEFAULT_LEFT_MOTOR_TRIM_PERMILLE  1070U
#define TEMPLATE_DEFAULT_RIGHT_MOTOR_TRIM_PERMILLE 930U

/* ---------- 车轮编码器（x4 计数） ---------- */
/* 左 PB8/PB9，右 PB6/PB7；正转报负数就翻对应翻转位。 */
#define TEMPLATE_LEFT_ENCODER_INVERTED  0U
#define TEMPLATE_RIGHT_ENCODER_INVERTED 1U
#define TEMPLATE_ENCODER_SAMPLE_MS      10U
/* 实测值：每转计数、轮径、每转行驶距离（0.1 mm） */
#define TEMPLATE_LEFT_ENCODER_COUNTS_PER_REV  1489U
#define TEMPLATE_RIGHT_ENCODER_COUNTS_PER_REV 1474U
#define TEMPLATE_WHEEL_DIAMETER_MM      48U
#define TEMPLATE_LEFT_WHEEL_TRAVEL_X10_MM  1508U
#define TEMPLATE_RIGHT_WHEEL_TRAVEL_X10_MM 1508U

/* ---------- 速度环（内环，增量式 PI） ---------- */
#define TEMPLATE_LINE_MIN_SPEED_MM_S     50
#define TEMPLATE_LINE_MAX_SPEED_MM_S   700
#define TEMPLATE_SPEED_PI_KP_MILLI      550
#define TEMPLATE_SPEED_PI_KI_MILLI      380
/* 起转最小占空比（千分比），克服静摩擦用。 */
#define TEMPLATE_SPEED_PI_MIN_OUTPUT     225
/* 软起步：目标速度每 5 ms 拍最多增加这么多 mm/s（停车是立即的）。
 * 1 => 到 260 mm/s 约 1.3 s，另一颗芯片上的小球不会被甩出去。 */
#define TEMPLATE_LINE_RAMP_STEP_MM_S_PER_TICK   1
/* 转弯自适应减速：每 1 mm/s 打角量降低基础速度百分之几（0 = 关闭）。 */
#define TEMPLATE_LINE_TURN_DECEL_FRACTION       10
#define TEMPLATE_LINE_TURN_MIN_RUN_MM_S         80

#endif
