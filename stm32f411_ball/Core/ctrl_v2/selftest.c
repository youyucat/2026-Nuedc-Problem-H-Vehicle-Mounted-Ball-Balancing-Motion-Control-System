/**
  ******************************************************************************
  * @file    selftest.c
  * @brief   开环参数辨识自检实现（非阻塞状态机）
  ******************************************************************************
  */

#include "selftest.h"

#if BALL_SELFTEST_ENABLE

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "main.h"
#include "stepper.h"

/* 视觉数据借用旧控制器的解析器（临时；ctrl_v2 接入后改成自己的 link 层） */
extern void     ball_ctrl_uart_poll(void);
extern int32_t  ball_ctrl_ball_01mm(void);
extern uint8_t  ball_ctrl_ball_ok(void);
extern uint32_t ball_ctrl_frame_count(void);

/* 调试日志走 PC6/PC7（USART6）。PA9/PA10 是与 MaixCam+主控的协议链路，
 * 自检输出绝不能发到那里，否则会污染 12 字节二进制帧。 */
extern UART_HandleTypeDef huart6;

#define ROD_STEPS_PER_DEG   ((float)STEPPER_STEPS_PER_REV / 360.0f)
#define T1_ANG_START        500      /* 0.5° */
#define T1_ANG_STEP         500
#define T1_ANG_END          6000     /* 6.0° */
#define T1_SETTLE_MS        300      /* 电机到位时间 */
#define T1_MEASURE_MS       2500     /* 每档观察时间 */
#define T1_BACK_MS          600      /* 回零等待 */
#define T1_MOVED_01MM       20       /* 位移 ≥2.0mm 视为"动了" */
#define T2_HOLD_MS          5000     /* 加速度辨识观察时长 */
#define T3_STEP_MDEG        5000     /* 延迟辨识的角度阶跃 */
#define T3_HOLD_MS          2000
#define T3_MOVED_01MM       5        /* 0.5mm 视为开始动 */

typedef enum {
    S_SETTLE = 0,
    S_T1_SET,
    S_T1_MARK,
    S_T1_MEAS,
    S_T1_BACK,
    S_T2_WAIT,
    S_T2_RUN,
    S_T3_PRE,
    S_T3_RUN,
    S_DONE,
    S_MONITOR          /* 自检结束后持续打印位置，供人工抬杆验证视觉 */
} st_state_t;

static st_state_t s_state;
static uint32_t   s_ts;          /* 进入当前状态的时刻 */
static int32_t    s_ang;         /* 当前测试角度 mdeg */
static int32_t    s_x_mark;
static int32_t    s_thresh;
static uint32_t   s_print_ts;
static uint32_t   s_t3_ms;
static uint8_t    s_t3_got;
static int32_t    s_level;

/* ------------------------------ 输出 ------------------------------ */

static void st_printf(const char *fmt, ...)
{
    char buf[96];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (n > (int)sizeof(buf) - 1) n = (int)sizeof(buf) - 1;
    (void)HAL_UART_Transmit(&huart6, (uint8_t *)buf, (uint16_t)n, 50);
}

/* ------------------------------ 动作 ------------------------------ */

static void st_set_angle(int32_t ang_mdeg)
{
    int32_t target = s_level + (int32_t)((float)ang_mdeg * 0.001f * ROD_STEPS_PER_DEG);
    stepper_servo_update(target);
}

static void st_enter(st_state_t s)
{
    s_state = s;
    s_ts    = HAL_GetTick();
}

static int32_t st_x(void)
{
    return ball_ctrl_ball_01mm();
}

/* ------------------------------ 接口 ------------------------------ */

void selftest_init(void)
{
    s_state    = S_SETTLE;
    s_ts       = HAL_GetTick();
    s_ang      = T1_ANG_START;
    s_x_mark   = 0;
    s_thresh   = 0;
    s_print_ts = 0;
    s_t3_ms    = 0;
    s_t3_got   = 0;
    s_level    = stepper_position();

    st_printf("\r\n# SELFTEST START\r\n");
    st_printf("# 开环辨识，全程不跑控制律\r\n");
    st_printf("# T1 静摩擦扫描 / T2 加速度 / T3 延迟\r\n");
    st_printf("# 上电等自动点头结束(约2s)后再放球，放好后别碰\r\n");
    st_printf("# 摆杆在 level_pos 打印后开始扫描\r\n\r\n");
}

void selftest_poll(void)
{
    uint32_t now;
    int32_t  disp, x, dx;

    ball_ctrl_uart_poll();          /* 保持视觉帧解析 */
    now = HAL_GetTick();

    switch (s_state) {

    /* ---------- 阶段 0：静置，确认球已停稳 ---------- */
    case S_SETTLE:
        if ((now - s_ts) >= 5000U) {
            s_level = stepper_position();
            st_printf("# level_pos=%ld\r\n", (long)s_level);
            st_enter(S_T1_SET);
        }
        break;

    /* ---------- 阶段 1：静摩擦阈值扫描 ---------- */
    case S_T1_SET:
        st_set_angle(s_ang);
        st_enter(S_T1_MARK);
        break;

    case S_T1_MARK:
        if ((now - s_ts) >= T1_SETTLE_MS) {
            s_x_mark = st_x();
            st_enter(S_T1_MEAS);
        }
        break;

    case S_T1_MEAS:
        if ((now - s_ts) >= T1_MEASURE_MS) {
            disp = st_x() - s_x_mark;
            if (disp < 0) disp = -disp;
            st_printf("T1,%ld,%ld,%s,%u\r\n", (long)s_ang, (long)disp,
                      (disp >= T1_MOVED_01MM) ? "MOVE" : "STUCK",
                      (unsigned)ball_ctrl_ball_ok());

            if (disp >= T1_MOVED_01MM) {
                s_thresh = s_ang;
                st_printf("# THRESHOLD=%ld mdeg\r\n", (long)s_thresh);
                st_set_angle(0);
                st_enter(S_T2_WAIT);
            } else {
                st_set_angle(0);
                st_enter(S_T1_BACK);
            }
        }
        break;

    case S_T1_BACK:
        if ((now - s_ts) >= T1_BACK_MS) {
            s_ang += T1_ANG_STEP;
            if (s_ang > T1_ANG_END) {
                st_printf("# THRESHOLD=NOT_FOUND（扫到 %d 都没动，查机械）\r\n", T1_ANG_END);
                s_thresh = 0;
                st_enter(S_T2_WAIT);
            } else {
                st_enter(S_T1_SET);
            }
        }
        break;

    /* ---------- 阶段 2：加速度辨识 ---------- */
    case S_T2_WAIT:
        if ((now - s_ts) >= 1000U) {
            /* 用「阈值 +2.5°」作为测试角度；阈值没找到就用 6° */
            s_ang = (s_thresh > 0) ? (s_thresh + 2500) : 6000;
            if (s_ang > 10000) s_ang = 10000;
            st_printf("# T2 angle=%ld mdeg, hold %d ms\r\n", (long)s_ang, T2_HOLD_MS);

            s_x_mark   = st_x();
            s_print_ts = now;
            st_set_angle(s_ang);
            st_enter(S_T2_RUN);
        }
        break;

    case S_T2_RUN:
        if ((now - s_ts) >= T2_HOLD_MS) {
            st_set_angle(0);
            st_enter(S_T3_PRE);
        } else if ((now - s_print_ts) >= 20U) {
            s_print_ts = now;
            st_printf("T2,%lu,%ld,%u\r\n",
                      (unsigned long)(now - s_ts), (long)(st_x() - s_x_mark),
                      (unsigned)ball_ctrl_ball_ok());
        }
        break;

    /* ---------- 阶段 3：延迟辨识 ---------- */
    case S_T3_PRE:
        if ((now - s_ts) >= 1500U) {
            s_x_mark   = st_x();
            s_print_ts = now;
            s_t3_ms    = 0;
            s_t3_got   = 0;
            st_printf("# T3 step %d mdeg\r\n", T3_STEP_MDEG);
            st_set_angle(T3_STEP_MDEG);
            s_ts = now;                    /* t0 = 现在（命令发出的时刻） */
            st_enter(S_T3_RUN);
        }
        break;

    case S_T3_RUN:
        if ((now - s_ts) >= T3_HOLD_MS) {
            if (!s_t3_got) st_printf("T3_DELAY,NOT_FOUND\r\n");
            st_set_angle(0);
            st_printf("\r\n# DONE —— 把上面输出复制保存为 CSV\r\n");
            st_printf("# 进入手动验证：用手把摆杆一端抬高约 1cm，\r\n");
            st_printf("#   同时看 M 行的位置值是否跟着大幅变化（跟手=视觉可用）\r\n");
            st_printf("# M,<位置0.1mm>,<视觉有效>,<累计帧>\r\n");
            s_print_ts = now - 200U;
            st_enter(S_MONITOR);
        } else {
            x  = st_x();
            dx = x - s_x_mark;
            if (dx < 0) dx = -dx;

            if (!s_t3_got && (dx >= T3_MOVED_01MM)) {
                s_t3_got = 1;
                st_printf("T3_DELAY,%lu\r\n", (unsigned long)(now - s_ts));
            }
            if ((now - s_print_ts) >= 20U) {
                s_print_ts = now;
                st_printf("T3,%lu,%ld,%u\r\n", (unsigned long)(now - s_ts),
                          (long)(x - s_x_mark), (unsigned)ball_ctrl_ball_ok());
            }
        }
        break;

    case S_MONITOR:
        /* 自检结束后的手动验证：持续打印位置，供人工抬摆杆对照。
         * M,<位置x0.1mm>,<视觉是否有效检出>,<累计帧数> */
        if ((now - s_print_ts) >= 100U) {
            s_print_ts = now;
            st_printf("M,%ld,%u,%lu\r\n", (long)st_x(),
                      (unsigned)ball_ctrl_ball_ok(),
                      (unsigned long)ball_ctrl_frame_count());
        }
        break;

    case S_DONE:
    default:
        break;
    }
}

#endif /* BALL_SELFTEST_ENABLE */
