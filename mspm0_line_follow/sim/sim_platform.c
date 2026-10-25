/* PC 仿真：物理世界 + 硬件 mock 实现（见 sim_platform.h）。
 *
 * 符号约定与产品代码对齐：
 *   motor_set(left, right)：左右轮期望占空比 -1000..+1000
 *   hal_encoder_snapshot：delta 为「已修正引脚方向后」的正转正计数
 *   line_error 右偏为正：探头世界坐标到线的带符号距离取反后作为
 *   「线相对探头的横向偏移」（线在探头右侧为正）
 */

#include "sim_platform.h"
#include "sim_track.h"

#include <math.h>
#include <string.h>

#include "../hal/hal_time.h"
#include "../hal/hal_encoder.h"
#include "../drivers/motor.h"
#include "../drivers/line_sensor.h"
#include "../config/template_config.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ===================== 全局状态 ===================== */

sim_params_t g_sim = {
    140.0,    /* wheel_base_mm */
    60.0,     /* sensor_fwd_mm */
    10.0,     /* pitch_mm（8 路 ±35mm 覆盖，与 ±3500 位置图一致） */
    12.0,     /* line_half_mm */
    800.0,    /* v_max_mm_s */
    100.0,    /* motor_tau_ms */
    15.0,     /* noise */
    42U
};

sim_car_t g_car;

static uint32_t s_now_ms;
static int32_t  s_enc_delta_l, s_enc_delta_r;
static int32_t  s_enc_total_l,  s_enc_total_r;
static bool     s_motor_enabled;
static int16_t  s_pwm_l, s_pwm_r;
static int16_t  s_motor_out_l, s_motor_out_r;
static uint16_t s_raw[TEMPLATE_SENSOR_MAX_CHANNELS];
static int      s_track_ready;

/* ---- 简易 LCG 噪声 ---- */
static double sim_noise(void)
{
    g_sim.seed = g_sim.seed * 1664525U + 1013904223U;
    return ((double)((g_sim.seed >> 8) & 0xFFFF) / 65535.0 - 0.5) *
           2.0 * g_sim.noise;
}

void sim_reset(double x, double y, double th)
{
    g_car.x = x; g_car.y = y; g_car.th = th;
    g_car.v_left = g_car.v_right = 0.0;
    g_car.target_left = g_car.target_right = 0.0;
    s_now_ms = 0U;
    s_enc_delta_l = s_enc_delta_r = 0;
    s_enc_total_l = s_enc_total_r = 0;
    s_motor_enabled = false;
    s_pwm_l = s_pwm_r = 0;
    s_motor_out_l = s_motor_out_r = 0;
}

/* ===================== 物理推进 ===================== */

static double pwm_to_speed(int16_t pwm)
{
    /* 死区 + 线性：起转约 15% 占空比（对应 SPEED_PI_MIN_OUTPUT 的量级） */
    double frac = (double)pwm / 1000.0;
    const double dead = 0.15;
    if (fabs(frac) < dead) return 0.0;
    frac = (frac > 0.0) ? (frac - dead) / (1.0 - dead)
                        : (frac + dead) / (1.0 - dead);
    return frac * g_sim.v_max_mm_s;
}

void sim_physics_step(void)
{
    const double dt = 0.001;                       /* 1 ms */
    double tau = (g_sim.motor_tau_ms > 1.0) ? g_sim.motor_tau_ms : 1.0;
    double alpha = dt * 1000.0 / tau;
    double v, omega, travel_l, travel_r;

    if (alpha > 1.0) alpha = 1.0;
    if (s_motor_enabled) {
        g_car.target_left  = pwm_to_speed(s_pwm_l);
        g_car.target_right = pwm_to_speed(s_pwm_r);
    } else {
        g_car.target_left = g_car.target_right = 0.0;
    }
    g_car.v_left  += (g_car.target_left  - g_car.v_left)  * alpha;
    g_car.v_right += (g_car.target_right - g_car.v_right) * alpha;

    v = (g_car.v_left + g_car.v_right) / 2.0;
    omega = (g_car.v_right - g_car.v_left) / g_sim.wheel_base_mm;

    travel_l = g_car.v_left  * dt;
    travel_r = g_car.v_right * dt;

    /* 编码器计数：1489 counts/rev，150.8mm/rev（config 实测值）
     * counts = travel_mm * 1489 / 150.8 ≈ travel_mm * 9.874 */
    {
        double cl = travel_l * 1489.0 / 150.8;
        double cr = travel_r * 1489.0 / 150.8;
        s_enc_delta_l += (int32_t)cl;
        s_enc_delta_r += (int32_t)cr;
        s_enc_total_l += (int32_t)cl;
        s_enc_total_r += (int32_t)cr;
    }

    g_car.th += omega * dt;
    g_car.x  += v * cos(g_car.th) * dt;
    g_car.y  += v * sin(g_car.th) * dt;
    s_now_ms += 1U;
}

/* ===================== hal / drivers mock ===================== */

uint32_t hal_time_millis(void) { return s_now_ms; }

void hal_board_init(void) { }

static const int16_t s_position_map[8] =
    { -3500, -2500, -1500, -500, 500, 1500, 2500, 3500 };

const int16_t *hal_gray_positions(void) { return s_position_map; }

void hal_encoder_init(void) { }
void hal_encoder_reset(void)
{
    s_enc_delta_l = s_enc_delta_r = 0;
}
void hal_encoder_snapshot(hal_encoder_snapshot_t *snapshot, bool reset_delta)
{
    if (snapshot == NULL) return;
    snapshot->left_delta  = s_enc_delta_l;
    snapshot->right_delta = s_enc_delta_r;
    snapshot->left_total  = s_enc_total_l;
    snapshot->right_total = s_enc_total_r;
    snapshot->left_invalid_transitions = 0U;
    snapshot->right_invalid_transitions = 0U;
    snapshot->left_state = snapshot->right_state = 0U;
    if (reset_delta) {
        s_enc_delta_l = s_enc_delta_r = 0;
    }
}

/* ---- 电机 ---- */
void motor_init(void)                 { s_motor_enabled = false; }
bool motor_enable(void)               { s_motor_enabled = true; return true; }
void motor_disable(void)
{
    s_motor_enabled = false;
    s_pwm_l = s_pwm_r = 0;
    s_motor_out_l = s_motor_out_r = 0;
}
bool motor_set(int16_t left, int16_t right)
{
    if (!s_motor_enabled) return false;
    s_pwm_l = left; s_pwm_r = right;
    s_motor_out_l = left; s_motor_out_r = right;
    return true;
}
void motor_brake(void)
{
    s_pwm_l = s_pwm_r = 0;
    s_motor_out_l = s_motor_out_r = 0;
    g_car.v_left  *= 0.5;      /* 简单刹车减速 */
    g_car.v_right *= 0.5;
}
int16_t motor_left_output(void)  { return s_motor_out_l; }
int16_t motor_right_output(void) { return s_motor_out_r; }

/* ---- 灰度传感器模型 ---- */
static void sensor_generate(void)
{
    uint8_t ch;

    if (!s_track_ready) {
        for (ch = 0; ch < TEMPLATE_SENSOR_CHANNELS; ++ch) s_raw[ch] = 3400U;
        return;
    }
    for (ch = 0; ch < TEMPLATE_SENSOR_CHANNELS; ++ch) {
        /* 探头 ch 的车体横向位置：index0 = -3500 -> 最左（-35mm） */
        double p_mm = (3.5 - (double)ch) * g_sim.pitch_mm;
        double wx = g_car.x + g_sim.sensor_fwd_mm * cos(g_car.th)
                              - p_mm * sin(g_car.th);
        double wy = g_car.y + g_sim.sensor_fwd_mm * sin(g_car.th)
                              + p_mm * cos(g_car.th);
        double d = track_signed_dist(wx, wy);   /* 左负右正 */
        double off = -d;                        /* 线相对探头：右正 */
        double a = fabs(off);
        double t, v;

        if (a <= g_sim.line_half_mm) {
            t = 0.0;                            /* 全黑 */
        } else if (a >= g_sim.line_half_mm + 3.0) {
            t = 1.0;                            /* 全白 */
        } else {
            t = (a - g_sim.line_half_mm) / 3.0; /* 3mm 过渡带 */
        }
        v = 500.0 + t * (3400.0 - 500.0) + sim_noise();
        if (v < 0.0) v = 0.0;
        if (v > 4095.0) v = 4095.0;
        s_raw[ch] = (uint16_t)v;
    }
}

void sim_mark_track_ready(void) { s_track_ready = 1; }

void line_sensor_init(void) { }

bool line_sensor_read(line_sensor_raw_frame_t *frame)
{
    if (frame == NULL) return false;
    sensor_generate();
    frame->channel_count = TEMPLATE_SENSOR_CHANNELS;
    frame->sensor_error = (!s_track_ready);
    memcpy(frame->values, s_raw,
           sizeof(uint16_t) * TEMPLATE_SENSOR_CHANNELS);
    return true;
}

bool line_sensor_async_step(line_sensor_raw_frame_t *frame)
{
    /* 实机每 5ms 拍完成一帧（8 路 x 8 样本，ADC 中断驱动），同样每拍产新帧 */
    if (frame == NULL) return false;
    sensor_generate();
    frame->channel_count = TEMPLATE_SENSOR_CHANNELS;
    frame->sensor_error = (!s_track_ready);
    memcpy(frame->values, s_raw,
           sizeof(uint16_t) * TEMPLATE_SENSOR_CHANNELS);
    return true;
}
