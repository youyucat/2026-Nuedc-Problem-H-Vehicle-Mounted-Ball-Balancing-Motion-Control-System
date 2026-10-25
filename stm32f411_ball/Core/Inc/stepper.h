/**
  ******************************************************************************
  * @file    stepper.h
  * @brief   步进电机脉冲控制（EN / DIR / STEP + 梯形加减速）
  *
  * 本文件属于用户代码，CubeMX 重新生成工程时不会被覆盖。
  ******************************************************************************
  */

#ifndef __STEPPER_H__
#define __STEPPER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ==========================================================================
 *                        参数配置区（按你的硬件改这里）
 * ========================================================================== */

/* --- 机械 / 细分换算 ------------------------------------------------------
 * 1.8° 电机 = 200 步/圈，驱动器细分 8 → 1600 脉冲/圈
 * 细分一定要和驱动器拨码开关上的档位一致，否则转的角度会有倍数误差。
 *   细分 1/4/8/16/32 → 200 / 800 / 1600 / 3200 / 6400
 * 0.9° 电机（400 步/圈）就把上面的数再乘 2。
 * ------------------------------------------------------------------------*/
#define STEPPER_STEPS_PER_REV     1600      /* 一圈需要多少脉冲 */

/* --- 电机轴 → 摆杆 的机械减速比 -----------------------------------------
 * 2026-10-25 水平仪标定结论：命令角 85 度时摆杆实际只倾 5.67 度，
 * 即电机转 15 度 = 摆杆转 1 度。5 个标定点（25/40/55/70/85 度命令）
 * 拟合出斜率恰为 1/15，残差为 0。
 *
 * 这个系数之前一直是 1.0（当成电机直连摆杆），导致所有上层角度都虚高
 * 15 倍：以为给了 5 度、实际只倾 0.33 度。整套控制参数因此全部失效。
 *
 * 换算关系：摆杆真实 1 度 = 1600/360*15 ≈ 66.67 个脉冲。
 * 摆杆分辨率提高到约 0.015 度/步（原来按 1:1 算是 0.225 度/步）。 */
#define STEPPER_GEAR_RATIO        15.0f

#define STEPPER_DEG_PER_MOVE      10        /* 每按一次按键转多少度 */

/* 每按一次按键实际下发的脉冲数（想直接写脉冲数就改这里） */
#define STEPPER_MOVE_STEPS        (((int32_t)STEPPER_STEPS_PER_REV * STEPPER_DEG_PER_MOVE) / 360)

/* --- 硬件极性 ----------------------------------------------------------- */
#define STEPPER_EN_ACTIVE_LOW     0         /* D36A 板实测：EN 高电平=使能（低/悬空=脱机，轴是松的） */
#define STEPPER_DIR_INVERT        0         /* 方向反了就改成 1 */

/* --- 定时器基准 ---------------------------------------------------------
 * TIM1 的定时器时钟 = 100MHz（APB2=100MHz，预分频=1），PSC=99 → 计数频率 1MHz。
 * 代码会在运行时按实际的 PCLK/PSC 反算，所以换了主频也不用改这里。
 * ----------------------------------------------------------------------*/
#define STEPPER_TIM_TICK_HZ       1000000UL /* 期望的定时器计数频率 */

/* STEP 高电平宽度（定时器 tick 数，10 = 10us）
 * 光耦输入的驱动器一般要求 >= 2.5us，10us 很稳。不要改得比 ARR 还大。
 * 这里故意用"固定脉宽"而不是 50% 占空比：CCR 全程不变，就不会因为 CCR
 * 预装载而在停止时多冒一个脉冲出去（那个会让位置一点点漂）。 */
#define STEPPER_PULSE_TICKS       10U

#define STEPPER_SPS_MIN           60U       /* 运行速度下限(脉冲/秒) */
#define STEPPER_SPS_START         300U      /* 起步瞬时速度(脉冲/秒)：太小则微动定位太慢，丢步就调小 */
#define STEPPER_SPS_MAX           40000U    /* 限速(脉冲/秒) */
#define STEPPER_ACCEL             20000U    /* 加速度(脉冲/秒²)，丢步就调小 */
#define STEPPER_RUN_SPEED         1600U     /* 默认运行速度(脉冲/秒) = 1 圈/秒 */

/* --- 伺服微动模式（H题要求4：行驶中摆杆稳球跟踪用） --- */
/* 2026-10-25 标定修正：电机到摆杆有 15:1 减速，原来的 800pps 实际只对应
 * 摆杆 800/66.67 ≈ 12 度/秒，想要 5 度倾角得等 0.42 秒 —— 执行器滞后比
 * 视觉延迟还大，是系统振荡的重要来源。提到 2400pps ≈ 36 度/秒，
 * 5 度倾角 0.14 秒到位。若出现丢步（角度不准/异响），按比例降。 */
#define STEPPER_SERVO_SPEED       2400U     /* 稳球微动巡航速度(脉冲/秒) ≈36°/s */
#define STEPPER_SERVO_DEADBAND    2         /* 伺服死区(步)：|Δ|≤此值不动，防抖 */

/* --- 按键确认"点头"（短按启动前摆杆抬一下再回平，肉眼可见 = 链路全通） --- */
#define STEPPER_ACK_DEG           3         /* 点头角度(度)：杆端行程约±6mm，看得清 */
#define STEPPER_ACK_STEPS         (((int32_t)STEPPER_STEPS_PER_REV * STEPPER_ACK_DEG) / 360)
#define STEPPER_ACK_SPS           300U      /* 点头速度(脉冲/秒)，柔和 */
#define STEPPER_ACK_DEG_RUN       2         /* 运行中/退出示意点头角度(度)：可见但尽量少扰动球 */
#define STEPPER_ACK_RUN_STEPS     (((int32_t)STEPPER_STEPS_PER_REV * STEPPER_ACK_DEG_RUN) / 360)

#define STEPPER_DISABLE_WHEN_IDLE 0         /* 1 = 空闲时松开使能（省电、发热低） */

/* ==========================================================================
 *                                 接口
 * ========================================================================== */

typedef enum {
    STEP_DIR_CW  = 0,   /* 正转 */
    STEP_DIR_CCW = 1    /* 反转 */
} stepper_dir_t;

/* --- 初始化 / 使能 --- */
void    stepper_init(void);
void    stepper_enable(uint8_t en);
uint8_t stepper_is_enabled(void);

/* --- 连续运行（本工程默认不用，留着备用） --- */
void    stepper_run(stepper_dir_t dir);
void    stepper_stop(void);          /* 平滑减速停止 */
void    stepper_estop(void);         /* 急停（会丢步） */

/* --- 定长 / 定位 --- */
void    stepper_move(int32_t steps); /* 走指定脉冲数：正=CW，负=CCW */
void    stepper_goto(int32_t pos);   /* 走到绝对位置（相对开机点） */

/* --- 伺服微动跟踪（稳球控制每 10ms 调一次） ---
 * 空闲且 |目标-当前| 超过死区才真正下发移动；正在动就只记住目标，
 * 走完这小段后下一次调用再补差。非阻塞，适合闭环高频调用。 */
void    stepper_servo_update(int32_t target);

/* --- 状态查询 --- */
int32_t       stepper_position(void);        /* 已输出脉冲累计位置 */
uint32_t      stepper_current_speed(void);   /* 当前速度(脉冲/秒) */
uint8_t       stepper_is_busy(void);         /* 1 = 还在动 */
uint8_t       stepper_is_move_active(void);  /* 1 = 定长移动指令进行中 */
int32_t       stepper_steps_left(void);      /* 定长移动剩余脉冲数（调试用） */
stepper_dir_t stepper_direction(void);       /* 当前方向 */

/* --- 速度设置 --- */
void     stepper_set_run_speed(uint32_t sps);
uint32_t stepper_get_run_speed(void);

/* --- 供 main / 中断调用 --- */
void    stepper_tick_1ms(void);                       /* 每 1ms 调用一次 */
void    stepper_pulse_isr(void);                      /* 在 TIM1 更新中断里调用 */
void    stepper_set_pwm_ref(uint16_t duty_permille);  /* PWM 脚占空比 0~1000 */

#ifdef __cplusplus
}
#endif

#endif /* __STEPPER_H__ */
