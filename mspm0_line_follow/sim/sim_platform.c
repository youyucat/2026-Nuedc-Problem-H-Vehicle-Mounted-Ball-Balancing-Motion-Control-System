/* 本文件仅用于 PC 仿真（gcc 构建）。目标芯片构建（tiarmclang 经
 * device.opt 定义 __MSPM0G3507__）中整体排除——CCS 自动源码发现会把
 * sim 目录拉进固件构建，mock 符号会与真实驱动重定义（2026-11-07 链接
 * 错误 #10056 的修复）。makefile 的固件构建不含本目录，不受影响。 */
#ifndef __MSPM0G3507__
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
    /* ---- 以下 5 项 2026-11-07 由实车 open 阶跃实测（150..1000 六档）----
     * 拟合 v = k*(pwm - pwm0)：左 k=0.6213/pwm0=7、右 k=0.6147/pwm0=-15，
     * 满速 617/624mm/s，tau 80~95ms。
     * 旧值（800mm/s、死区 15%、tau 100）与实车差得多：满速高估 30%，
     * 死区高估 20 倍 —— 仿真的速度环整定因此失真。 */
    617.0,    /* v_max_mm_s：pwm=1000 实测 617mm/s（原 800） */
    0.007,    /* dead_frac：实测死区 0.7%（原 0.15） */
    90.0,     /* motor_tau_ms：实测 63% 上升 80~95ms（原 100） */
    60.0,     /* noise：实车白底峰峰 100~500，取 ±60 中位 */
    1.0,      /* gain_left */
    1.011,    /* gain_right：右轮 624/617 略快 1.1% */
    0.88,     /* turn_eff */
    42U
};

sim_car_t g_car;



static uint32_t s_now_ms;
static int32_t  s_enc_delta_l, s_enc_delta_r;
static double   s_enc_frac_l, s_enc_frac_r;   /* 计数小数余量 */
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
    s_enc_frac_l = s_enc_frac_r = 0.0;
    s_motor_enabled = false;
    s_pwm_l = s_pwm_r = 0;
    s_motor_out_l = s_motor_out_r = 0;
}

/* ===================== 物理推进 ===================== */

static double pwm_to_speed(int16_t pwm)
{
    /* 死区 + 线性。死区实测仅 0.7%（pwm=150 已有 81mm/s），
     * 旧值 15% 是错的：它让仿真里小 PWM 完全不转，速度环行为失真。 */
    double frac = (double)pwm / 1000.0;
    const double dead = g_sim.dead_frac;
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
        g_car.target_left  = pwm_to_speed(s_pwm_l)  * g_sim.gain_left;
        g_car.target_right = pwm_to_speed(s_pwm_r) * g_sim.gain_right;
    } else {
        g_car.target_left = g_car.target_right = 0.0;
    }
    g_car.v_left  += (g_car.target_left  - g_car.v_left)  * alpha;
    g_car.v_right += (g_car.target_right - g_car.v_right) * alpha;

    v = (g_car.v_left + g_car.v_right) / 2.0;
    /* 差速 -> 转角打折：实车轮胎侧滑、减速箱间隙让实际转向小于
     * 纯运动学预测（1.0 = 理想刚体） */
    omega = (g_car.v_right - g_car.v_left) / g_sim.wheel_base_mm *
            g_sim.turn_eff;

    travel_l = g_car.v_left  * dt;
    travel_r = g_car.v_right * dt;

    /* 编码器计数：1489 counts/rev，150.8mm/rev（config 实测值）
     * counts = travel_mm * 1489 / 150.8 ≈ travel_mm * 9.874 */
    {
        /* 小数余量必须保留：旧版每 1ms 独立截断，260mm/s 时每步丢
         * 0.57 个计数（≈22% 系统误差），且低速时直接不计数 ——
         * 仿真把编码器做得比实车精细，掩盖了速度环/阻尼对量化噪声
         * 的敏感性（2026-11-07 实车翻车的仿真根源之一）。
         * 保留余量后，10ms 采样窗口内 delta 是整数计数，速度分辨率
         * 自然变成 150.8/1489/0.01 ≈ 10.1 mm/s，与实车一致。 */
        double cl = travel_l * 1489.0 / 150.8 + s_enc_frac_l;
        double cr = travel_r * 1489.0 / 150.8 + s_enc_frac_r;
        int32_t il = (int32_t)cl;
        int32_t ir = (int32_t)cr;

        s_enc_frac_l = cl - (double)il;
        s_enc_frac_r = cr - (double)ir;
        s_enc_delta_l += il;
        s_enc_delta_r += ir;
        s_enc_total_l += il;
        s_enc_total_r += ir;
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

/* ---- 灰度传感器模型 ----
 * 2026-11-07 实车 raw 实测标定（车静置不同横向偏移，6 帧）：
 *   白电平各通道差极大：2441 ~ 3330（ch3 最暗、ch4 最亮），
 *   纯黑 ~480，白底噪声 ±60~260（ch5 最大）。
 * 旧的统一 white=3400 / black=500 / noise=15 让灰度"又亮又干净"，
 * 把实车最关键的低电平通道问题（ch3/ch6 未标定时余量仅 400）
 * 完全掩盖了 —— 这也是仿真看不出直线抖动的原因之一。 */
/* 2026-11-07 实车 cal apply 实测（黑/白电平，逐通道）：
 * 注意 ch4 黑电平 1265，是其他通道（234~423）的 5 倍 —— 探头异常
 * （脏污/高度/漏光）或标定时线未盖准，导致其阈值被抬到 2211。 */
static const double s_white_lv[TEMPLATE_SENSOR_CHANNELS] = {
    3173.0, 3102.0, 2893.0, 2624.0, 3157.0, 3007.0, 2749.0, 3059.0
};
static const double s_black_lv[TEMPLATE_SENSOR_CHANNELS] = {
    423.0, 281.0, 234.0, 247.0, 1265.0, 252.0, 256.0, 269.0
};

const double *sim_gray_white_levels(void) { return s_white_lv; }
const double *sim_gray_black_levels(void) { return s_black_lv; }

static void sensor_generate(void)
{
    uint8_t ch;

    if (!s_track_ready) {
        for (ch = 0; ch < TEMPLATE_SENSOR_CHANNELS; ++ch) {
            s_raw[ch] = (uint16_t)s_white_lv[ch];
        }
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
        v = s_black_lv[ch] + t * (s_white_lv[ch] - s_black_lv[ch])
            + sim_noise();
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

#endif /* __MSPM0G3507__ */
