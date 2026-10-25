/* PC 仿真主程序：跑场景 + 收指标。
 *
 * 场景：
 *   1 straight   直线起步（横向偏 8mm）—— 看蛇形/收敛
 *   2 arc500     R500 弯道 —— 看跟随误差
 *   3 arc250     R250 急弯 —— 看是否甩出/丢线
 *   4 lost       线中断 —— 看摇头恢复与 1500ms 停车时序
 *
 * 用法：sim_line [场景号] [kp_milli] [kd_milli] [base] [tau_ms] [vmax]
 *   不带参数 = 跑全部场景的默认参数回归
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../app/line_task.h"
#include "../config/template_config.h"
#include "../services/line_processing.h"
#include "../hal/hal_time.h"
#include "../hal/hal_encoder.h"
#include "../drivers/motor.h"
#include "../drivers/line_sensor.h"

#include "sim_platform.h"
#include "sim_track.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- 场景 ---- */
typedef struct {
    const char *name;
    track_seg_t seg;
    double x0, y0, th0;
    double run_ms;
} scene_t;

static scene_t s_scenes[] = {
    { "straight", { 0U, 0.0, 0.0, 1.0, 0.0, 2000.0, 0.0, 0.0, 0.0 },
      0.0, 8.0, 0.0, 3000.0 },
    { "arc500",   { 1U, 0.0, -500.0, 0.0, 0.0, 0.0, 500.0, -M_PI / 2, M_PI / 2 },
      0.0, -1008.0, 0.0, 4000.0 },
    { "arc250",   { 1U, 0.0, -250.0, 0.0, 0.0, 0.0, 250.0, -M_PI / 2, M_PI / 2 },
      0.0, -506.0, 0.0, 4000.0 },
    { "lost",     { 0U, 0.0, 0.0, 1.0, 0.0, 700.0, 0.0, 0.0, 0.0 },
      0.0, 0.0, 0.0, 3500.0 },
};

#define SCENE_N (sizeof(s_scenes) / sizeof(s_scenes[0]))

/* ---- 指标 ---- */
typedef struct {
    double max_abs_off;        /* 阵列中心横向偏差峰值 */
    double max_err;            /* |line_error| 峰值 */
    int    lost_events;        /* 丢线次数 */
    uint32_t lost_total_ms;    /* 累计丢线时长 */
    uint32_t recover_ms;       /* 最后一次丢线恢复用时（丢线事件长度） */
    int    finished;           /* 走完全程 */
    int    stopped_in_fault;   /* 进 FAULT */
    double sweep_seen;         /* 观察到的摇头修正峰值 */
    double end_progress;       /* 结束时里程 */
} sim_result_t;

static int s_line_valid_prev;
static uint32_t s_lost_start_ms;
static int g_verbose;   /* 命令行传 v 打开逐 200ms 轨迹打印 */

static void run_scene(const scene_t *sc, int16_t kp, int16_t kd,
                      int16_t base, double tau, double vmax,
                      sim_result_t *out)
{
    uint32_t now;
    line_task_snapshot_t snap;

    g_sim.motor_tau_ms = tau;
    g_sim.v_max_mm_s = vmax;
    memset(out, 0, sizeof(*out));

    track_set(&sc->seg, 1);
    sim_mark_track_ready();
    sim_reset(sc->x0, sc->y0, sc->th0);

    line_task_init();
    /* 模拟串口 stop：不许自动起步，等我们 start */
    line_task_set_auto_start(false);
    line_task_set_gains(base, kp, kd);
    line_task_get_snapshot(&snap);
    /* 标定（模拟 cal white + cal black + apply 的结果） */
    {
        uint16_t black[TEMPLATE_SENSOR_MAX_CHANNELS];
        uint16_t white[TEMPLATE_SENSOR_MAX_CHANNELS];
        uint8_t i;
        for (i = 0; i < TEMPLATE_SENSOR_CHANNELS; ++i) {
            black[i] = 500U;
            white[i] = 3400U;
        }
        (void)line_processing_apply_points(black, white);
    }
    (void)line_task_start();

    for (now = 0U; now < (uint32_t)sc->run_ms; ++now) {
        line_task_step(now);
        sim_physics_step();

        /* ---- 指标 ---- */
        {
            double cx = g_car.x + g_sim.sensor_fwd_mm * cos(g_car.th);
            double cy = g_car.y + g_sim.sensor_fwd_mm * sin(g_car.th);
            double off = -track_signed_dist(cx, cy);   /* 右正 */
            double a = fabs(off);
            if (a > out->max_abs_off) out->max_abs_off = a;

            line_task_get_snapshot(&snap);
            {
                double e = fabs((double)snap.line_error);
                if (e > out->max_err) out->max_err = e;
            }

            if (g_verbose && ((now % 200U) == 0U)) {
                printf("  t=%4u x=%6.1f y=%6.1f th=%+5.2f err=%+4d v=%d off=%+6.1f pwmL=%4d pwmR=%4d v=%d\r\n",
                       now, g_car.x, g_car.y, g_car.th,
                       snap.line_error, (snap.line_valid?1:0), off,
                       snap.left_output, snap.right_output,
                       (int)g_car.v_left);
                printf("      tgtL=%4d measL=%5d tgtR=%4d measR=%5d seq=%u encL=%d\r\n",
                       snap.left_target, snap.left_speed_mm_s,
                       snap.right_target, snap.right_speed_mm_s,
                       snap.sequence, (int)0);
            }
            if (!snap.line_valid && s_line_valid_prev) {
                out->lost_events++;
                s_lost_start_ms = now;
            }
            if (!snap.line_valid) {
                out->lost_total_ms++;
                out->recover_ms = now - s_lost_start_ms + 1U;
            }
            s_line_valid_prev = snap.line_valid ? 1 : 0;

            if (snap.state == LINE_TASK_STATE_FAULT) {
                out->stopped_in_fault = 1;
                break;
            }
        }
    }
    line_task_get_snapshot(&snap);
    out->end_progress = track_progress(
        g_car.x + g_sim.sensor_fwd_mm * cos(g_car.th),
        g_car.y + g_sim.sensor_fwd_mm * sin(g_car.th));
    out->finished = (snap.state == LINE_TASK_STATE_RUNNING) &&
                    (out->lost_events == 0);
    (void)base;
}

static void print_result(const char *name, const sim_result_t *r)
{
    printf("[%s] maxOff=%5.1fmm maxErr=%4.0f lost=%d/%ums "
           "fault=%d prog=%5.0fmm\r\n",
           name, r->max_abs_off, r->max_err, r->lost_events,
           r->lost_total_ms, r->stopped_in_fault, r->end_progress);
}

/* 摇头幅度观察：丢线场景专用（recover_lost_line 的目标差） */
static void observe_sweep(void) { }

int main(int argc, char **argv)
{
    int scene = -1;
    int16_t kp = TEMPLATE_DEFAULT_KP_MILLI;
    int16_t kd = TEMPLATE_DEFAULT_KD_MILLI;
    int16_t base = TEMPLATE_DEFAULT_BASE_OUTPUT;
    double tau = 100.0, vmax = 800.0;

    int i;
    if (argc > 1) scene = atoi(argv[1]);
    if (argc > 2) kp = (int16_t)atoi(argv[2]);
    if (argc > 3) kd = (int16_t)atoi(argv[3]);
    if (argc > 4) base = (int16_t)atoi(argv[4]);
    if (argc > 5) tau = atof(argv[5]);
    if (argc > 6) vmax = atof(argv[6]);
    if (argc > 7) g_sim.sensor_fwd_mm = atof(argv[7]);
    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "v") == 0) g_verbose = 1;
    }
    (void)observe_sweep;

    if (scene >= 1 && scene <= (int)SCENE_N) {
        sim_result_t r;
        run_scene(&s_scenes[scene - 1], kp, kd, base, tau, vmax, &r);
        print_result(s_scenes[scene - 1].name, &r);
    } else {
        unsigned i;
        printf("kp=%d kd=%d base=%d tau=%.0f vmax=%.0f\r\n",
               kp, kd, base, tau, vmax);
        for (i = 0; i < SCENE_N; ++i) {
            sim_result_t r;
            run_scene(&s_scenes[i], kp, kd, base, tau, vmax, &r);
            print_result(s_scenes[i].name, &r);
        }
    }
    return 0;
}
