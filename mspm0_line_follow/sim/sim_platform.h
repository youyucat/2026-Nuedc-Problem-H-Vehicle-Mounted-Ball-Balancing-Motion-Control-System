#ifndef SIM_PLATFORM_H
#define SIM_PLATFORM_H

#include <stdbool.h>
#include <stdint.h>

/*
 * PC 仿真的「物理世界」+ 全部硬件 mock。
 *
 * 产品代码（app/control/services/drivers 里除了 ADC/编码器/电机这几个
 * 硬件驱动之外的全部 .c）原样编译，本文件提供它们的硬件接口实现：
 *   hal_time_millis / hal_board_init / hal_gray_positions
 *   motor_*            （PWM 命令 -> 轮速一阶模型）
 *   hal_encoder_*      （计数跟随仿真轮速）
 *   line_sensor_*      （车体位姿 -> 8 路灰度原始值）
 */

typedef struct {
    double x, y, th;          /* 车体位姿（mm, mm, rad；th=0 朝 +x） */
    double v_left, v_right;   /* 实际轮速 mm/s */
    double target_left, target_right;
} sim_car_t;

/* ---- 物理参数（命令行可覆盖，见 sim_main） ---- */
typedef struct {
    double wheel_base_mm;     /* 轮距 */
    double sensor_fwd_mm;     /* 阵列在车体原点前方多少 mm */
    double pitch_mm;          /* 相邻探头间距（8 路，中心对称） */
    double line_half_mm;      /* 黑线半宽 */
    double v_max_mm_s;        /* 满占空比轮速 */
    double motor_tau_ms;      /* 电机一阶滞后 */
    double noise;             /* 灰度读数噪声（ADC 量化近似） */
    unsigned seed;
} sim_params_t;

extern sim_params_t g_sim;
extern sim_car_t    g_car;

void sim_reset(double x, double y, double th);
void sim_mark_track_ready(void);

/* 物理推进 1ms：轮速一阶响应 -> 差速运动学积分 */
void sim_physics_step(void);

/* ---- 被 line_task.c 调用的硬件接口（mock 实现） ---- */
uint32_t sim_hal_time_millis(void);
void     sim_hal_board_init(void);
const int16_t *sim_hal_gray_positions(void);
void     sim_motor_brake(void);
void     sim_motor_disable(void);
bool     sim_motor_enable(void);
void     sim_motor_init(void);
bool     sim_motor_set(int16_t left, int16_t right);
int16_t  sim_motor_left_output(void);
int16_t  sim_motor_right_output(void);

#endif
