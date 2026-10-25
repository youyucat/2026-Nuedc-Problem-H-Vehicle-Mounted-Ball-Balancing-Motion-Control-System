/**
  ******************************************************************************
  * @file    selftest.c
  * @brief   命令角 → 真实倾角 标定大测试（临时诊断，不参与正常控制）
  *
  * 背景：T4 证明步进位置严格跟随命令（4.444 步/度），但水平仪实测
  *       命令 5°/15°/25° 只对应真实 0/1/2°。即「电机转了，摆杆没转那么多」，
  *       比例约 12.5:1。原有的摩擦扫描/加速度/延迟测试都建立在这个错误
  *       换算之上，结论全部作废，因此整段删掉。
  *
  * 本测试只做一件事：把命令角扫一遍，每档停下来让人读水平仪。
  ******************************************************************************
  */

#include "selftest.h"

#if BALL_SELFTEST_ENABLE

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "main.h"
#include "stepper.h"

/* 视觉数据借用旧控制器的解析器（临时；ctrl_v2 接入后改成自己的 link 层）。
 * 帧解析由 main.c 主循环统一轮询，这里只读取结果。 */
extern int32_t  ball_ctrl_ball_01mm(void);

/* 步进内部状态 */
extern int32_t  stepper_position(void);
extern uint8_t  stepper_is_busy(void);

/* 调试日志走 PC6/PC7（USART6）。PA9/PA10 是与 MaixCam+主控的协议链路，
 * 自检输出绝不能发到那里，否则会污染 12 字节二进制帧。 */
extern UART_HandleTypeDef huart6;

/* 注意：这里仍用工程原有的换算系数（1600 步/圈 = 4.444 步/度）。
 * 本测试的目的就是测出它与真实倾角的偏差，所以不能先改它。 */
#define ROD_STEPS_PER_DEG   ((float)STEPPER_STEPS_PER_REV / 360.0f)

/* ================== 测试参数：想改档位只改这一段 ================== */

/* 命令角序列（mdeg）。由小到大，避免一上来就大幅摆动。
 * 按 12.5:1 的估计，这五档约对应真实 2 / 3.2 / 4.4 / 5.6 / 6.8 度，
 * 覆盖足够宽，水平仪 1 度分辨率下拟合斜率误差约 ±5%。 */
static const int32_t TEST_ANGLES[] = {
    25000, 40000, 55000, 70000, 85000
};
#define TEST_N   (sizeof(TEST_ANGLES) / sizeof(TEST_ANGLES[0]))

#define T_SETTLE_MS   5000U   /* 上电静置，避开自动点头 */
#define T_MOVE_MS      800U   /* 下发角度后等步进走完（85°=378步@800pps≈0.5s） */
#define T_HOLD_MS     4000U   /* 保持不动，留足时间读水平仪 + 手写记录 */
#define T_BACK_MS     1500U   /* 回平后等摆杆归位、球回落 */

/* ================================================================= */

typedef enum {
    S_SETTLE = 0,   /* 上电静置 */
    S_MOVE,         /* 已下发角度，等电机到位 */
    S_HOLD,         /* 保持中，用户读数 */
    S_BACK,         /* 已回平，等稳定 */
    S_DONE
} st_state_t;

static st_state_t s_state;
static uint32_t   s_ts;        /* 进入当前状态的时刻 */
static uint8_t    s_idx;       /* 当前档位索引 */
static int32_t    s_level;     /* 上电位置 = 水平零位（步数） */
static uint8_t    s_done_once;

/* ------------------------------ 输出 ------------------------------ */

static void st_printf(const char *fmt, ...)
{
    char buf[128];
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

/* 开始第 s_idx 档：下发角度，打印提示 */
static void st_begin_step(uint32_t now)
{
    int32_t ang = TEST_ANGLES[s_idx];

    st_set_angle(ang);
    s_state = S_MOVE;
    s_ts    = now;

    st_printf("\r\n# --------------------------------------------------\r\n");
    st_printf("# [%u/%u] 命令 %ld.%03ld 度，正在摆到位（约 1 秒）\r\n",
              (unsigned)(s_idx + 1U), (unsigned)TEST_N,
              (long)(ang / 1000), (long)(ang % 1000));
}

/* ------------------------------ 接口 ------------------------------ */

void selftest_init(void)
{
    s_state     = S_SETTLE;
    s_ts        = HAL_GetTick();
    s_idx       = 0;
    s_done_once = 0;
    s_level     = stepper_position();

    st_printf("\r\n# ==================================================\r\n");
    st_printf("# 角度标定大测试（开环，不跑控制律）\r\n");
    st_printf("# 共 %u 档，每档：摆到位 1s -> 保持 4s -> 回平 1.5s\r\n",
              (unsigned)TEST_N);
    st_printf("# 全程约 %u 秒\r\n",
              (unsigned)((T_SETTLE_MS + TEST_N * (T_MOVE_MS + T_HOLD_MS + T_BACK_MS)) / 1000U));
    st_printf("#\r\n");
    st_printf("# 你需要做的：看到「读数」提示后，抬头读水平仪，\r\n");
    st_printf("#   在纸上按「档号:读数」记录，例如  1:2  2:3  3:4\r\n");
    st_printf("# 建议先把钢球拿走，避免它滚到端点反复撞击\r\n");
    st_printf("# ==================================================\r\n");

    st_printf("\r\n# 静置 %u 秒（等自动点头结束），零位 = 上电时的摆杆位置\r\n",
              (unsigned)(T_SETTLE_MS / 1000U));
    st_printf("# 若此时水平仪不是 0，说明上电前没调平，请重来\r\n");
}

void selftest_poll(void)
{
    uint32_t now = HAL_GetTick();

    switch (s_state) {

    /* ---------- 上电静置 ---------- */
    case S_SETTLE:
        if ((now - s_ts) >= T_SETTLE_MS) {
            st_begin_step(now);
        }
        break;

    /* ---------- 等电机到位 ---------- */
    case S_MOVE:
        if ((now - s_ts) >= T_MOVE_MS) {
            s_state = S_HOLD;
            s_ts    = now;
            st_printf("# >>> [%u] 保持中，现在读水平仪并记下\r\n",
                      (unsigned)(s_idx + 1U));
        }
        break;

    /* ---------- 保持，等用户读数 ---------- */
    case S_HOLD:
        if ((now - s_ts) >= T_HOLD_MS) {
            /* R,<档号>,<命令mdeg>,<步进相对位置>,<球位置>,<忙>
             * 步进位置用于与命令交叉核对，球位置仅供参考 */
            st_printf("# [%u] 记录完毕，回平\r\n", (unsigned)(s_idx + 1U));
            st_printf("R,%u,%ld,%ld,%ld,%u\r\n",
                      (unsigned)(s_idx + 1U),
                      (long)TEST_ANGLES[s_idx],
                      (long)(stepper_position() - s_level),
                      (long)ball_ctrl_ball_01mm(),
                      (unsigned)stepper_is_busy());

            st_set_angle(0);
            s_state = S_BACK;
            s_ts    = now;
        }
        break;

    /* ---------- 回平后等稳定，进下一档 ---------- */
    case S_BACK:
        if ((now - s_ts) >= T_BACK_MS) {
            s_idx++;
            if (s_idx >= (uint8_t)TEST_N) {
                s_state = S_DONE;
                st_set_angle(0);
            } else {
                st_begin_step(now);
            }
        }
        break;

    /* ---------- 结束 ---------- */
    case S_DONE:
        if (!s_done_once) {
            s_done_once = 1;
            st_printf("\r\n# ==================================================\r\n");
            st_printf("# DONE —— 摆杆已回平\r\n");
            st_printf("# 请把记录整理成一行发我，格式：\r\n");
            st_printf("#   1:2 2:3 3:4 4:6 5:7        (档号:水平仪读数)\r\n");
            st_printf("# 共 %u 档，对应命令角：", (unsigned)TEST_N);
            {
                uint8_t i;
                for (i = 0; i < (uint8_t)TEST_N; i++) {
                    st_printf(" %ld", (long)(TEST_ANGLES[i] / 1000));
                }
            }
            st_printf("\r\n# ==================================================\r\n");
        }
        break;

    default:
        break;
    }
}

#endif /* BALL_SELFTEST_ENABLE */
