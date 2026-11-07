/*
 * 串口调试模块：行命令解析 + 灰度校准。
 *
 * 为什么不用 SysConfig 实例：本工程约定外设手写集中在 bsp/（TB6612 的
 * TIMA1 就是这么做的），.syscfg 只保留 SYSCTL/ADC/SysTick，避免
 * 多人协作时 syscfg 文件冲突。引脚固定 PA10/PA11（UART0 的 PINCM21/22，
 * 已核对本工程 16 个已占用引脚无冲突，见 bsp/board_profile.h）。
 *
 * 波特率来源：syscfg 配置 MCLK=80MHz、ULPCLK=MCLK/2=40MHz，
 * UART 的 BUSCLK 即 40MHz。若日后改系统时钟，需同步改下面的分频宏。
 */

#include "bsp/uart_debug.h"

#include <stdarg.h>
#include <stddef.h>
#include <string.h>

#include "ti_msp_dl_config.h"

#include "app/line_task.h"
#include "config/template_config.h"
#include "hal/hal_time.h"
#include "hal/hal_uart.h"
#include "drivers/uart_async.h"
#include "services/line_processing.h"

/* ===================== 状态 ===================== */

#define RX_LINE_MAX  96U

static char     s_rx_line[RX_LINE_MAX];
static uint8_t  s_rx_len;

static uint32_t s_stream_period_ms;      /* 0 = 关闭周期 stat */
static uint32_t s_stream_last_ms;

static uint8_t  s_cal_sample_target;     /* 采样帧数 */
static uint8_t  s_cal_sample_count;
static uint32_t s_cal_sum[TEMPLATE_SENSOR_CHANNELS];
static uint32_t s_cal_last_seq;
static bool     s_cal_is_white;          /* 正在采白还是黑 */
static uint16_t s_cal_white[TEMPLATE_SENSOR_MAX_CHANNELS];
static uint16_t s_cal_black[TEMPLATE_SENSOR_MAX_CHANNELS];
static bool     s_have_white;
static bool     s_have_black;

/* ===================== 底层收发（uart_async：原版验证模块） =====================
 * 时钟、波特率、引脚复用全部由 SysConfig 生成（SYSCFG_DL_UART_0_init），
 * TX 走 uart_async 的 1024B 环缓 + TX 空中断，RX 环缓由 HAL ISR 填充。
 * 2026-11-07：手写 UART 初始化三版分频全错后，整体换回原版模块。 */

static void dbg_write(const char *text)
{
    if (text == NULL) return;
    (void)uart_async_write(text, (uint16_t)strlen(text));
}

/* 整数版 printf：只用 %d/%u/%s/%x（不带 l/h 修饰）。
 * 嵌入式 vsnprintf 体积较大且本工程无 stdio 依赖，自己写个够用的。 */
static void dbg_printf(const char *fmt, ...)
{
    char chunk[12];
    va_list ap;
    const char *p;

    va_start(ap, fmt);
    for (p = fmt; *p != '\0'; ++p) {
        if (*p != '%') {
            char one[2] = { *p, '\0' };
            dbg_write(one);
            continue;
        }
        ++p;
        if (*p == '\0') break;
        switch (*p) {
        case 'd': {
            int v = va_arg(ap, int);
            int32_t n = (int32_t)v;
            uint8_t i = sizeof(chunk);
            bool neg = (n < 0);
            uint32_t u = neg ? (uint32_t)(-n) : (uint32_t)n;
            chunk[--i] = '\0';
            do { chunk[--i] = (char)('0' + (u % 10U)); u /= 10U; } while (u > 0U);
            if (neg) chunk[--i] = '-';
            dbg_write(&chunk[i]);
            break;
        }
        case 'u': {
            uint32_t u = va_arg(ap, uint32_t);
            uint8_t i = sizeof(chunk);
            chunk[--i] = '\0';
            do { chunk[--i] = (char)('0' + (u % 10U)); u /= 10U; } while (u > 0U);
            dbg_write(&chunk[i]);
            break;
        }
        case 'x': {
            uint32_t u = va_arg(ap, uint32_t);
            static const char hex[] = "0123456789abcdef";
            uint8_t i = sizeof(chunk);
            chunk[--i] = '\0';
            if (u == 0U) chunk[--i] = '0';
            while (u > 0U) { chunk[--i] = hex[u & 0xFU]; u >>= 4; }
            dbg_write(&chunk[i]);
            break;
        }
        case 's':
            dbg_write(va_arg(ap, const char *));
            break;
        default:
            break;
        }
    }
    va_end(ap);
}

/* ===================== 状态文本 ===================== */

static const char *state_name(line_task_state_t state)
{
    switch (state) {
    case LINE_TASK_STATE_IDLE:    return "IDLE";
    case LINE_TASK_STATE_RUNNING: return "RUN";
    case LINE_TASK_STATE_FAULT:   return "FAULT";
    default:                      return "?";
    }
}

static void print_stat(void)
{
    line_task_snapshot_t snap;

    line_task_get_snapshot(&snap);
    dbg_printf("[%s] seq=%u err=%d valid=%d mask=%x\r\n",
               state_name(snap.state), snap.sequence,
               snap.line_error, (snap.line_valid ? 1 : 0), snap.active_mask);
    dbg_printf("  vL=%d vR=%d mm/s | pwmL=%d pwmR=%d | tgtL=%d tgtR=%d | err_flag=%d\r\n",
               snap.left_speed_mm_s, snap.right_speed_mm_s,
               snap.left_output, snap.right_output,
               snap.left_target, snap.right_target,
               (snap.sensor_error ? 1 : 0));
}

static void print_raw(void)
{
    line_frame_t f;
    uint8_t i;

    line_task_get_frame(&f);
    dbg_write("raw  :");
    for (i = 0U; i < f.channel_count; ++i) {
        dbg_printf(" %u", f.raw[i]);
    }
    dbg_write("\r\nthresh:");
    for (i = 0U; i < f.channel_count; ++i) {
        dbg_printf(" %u", f.thresholds[i]);
    }
    dbg_printf("\r\nmask=%x cal=%d err=%d\r\n",
               f.active_mask, (f.calibrated ? 1 : 0), (f.sensor_error ? 1 : 0));
}

static void print_help(void)
{
    dbg_write(
        "help | stat | raw\r\n"
        "stream <ms>|off   : period stat print\r\n"
        "start | stop | reset\r\n"
        "gain [base kp kd] : query or set (IDLE only)\r\n"
        "trim [l r]       : query or set (IDLE only)\r\n"
        "cal white         : sample WHITE, car on white floor\r\n"
        "cal black         : sample BLACK, car on line\r\n"
        "cal apply         : apply points -> thresholds\r\n"
        "cal show | cal reset\r\n");
}

/* ===================== 校准采样 ===================== */

static void cal_begin(bool is_white)
{
    line_task_snapshot_t snap;

    line_task_get_snapshot(&snap);
    if (snap.state == LINE_TASK_STATE_RUNNING) {
        dbg_write("cal: stop the car first (stop)\r\n");
        return;
    }
    s_cal_is_white       = is_white;
    s_cal_sample_target  = 20U;          /* 20 帧 x 5ms = 100ms 平均 */
    s_cal_sample_count   = 0U;
    s_cal_last_seq       = snap.sequence;
    memset(s_cal_sum, 0, sizeof(s_cal_sum));
    dbg_write(is_white ? "cal: sampling WHITE...\r\n"
                        : "cal: sampling BLACK...\r\n");
}

static void cal_sample_step(void)
{
    line_task_snapshot_t snap;
    line_frame_t f;
    uint8_t i;

    line_task_get_snapshot(&snap);
    if (s_cal_sample_count >= s_cal_sample_target) return;
    if (snap.sequence == s_cal_last_seq) return;      /* 等新帧 */
    s_cal_last_seq = snap.sequence;

    line_task_get_frame(&f);
    if (f.sensor_error) return;
    for (i = 0U; i < f.channel_count; ++i) {
        s_cal_sum[i] += f.raw[i];
    }
    if (++s_cal_sample_count >= s_cal_sample_target) {
        uint16_t *dst = s_cal_is_white ? s_cal_white : s_cal_black;
        uint8_t n = s_cal_sample_count;
        dbg_write(s_cal_is_white ? "white:" : "black:");
        for (i = 0U; i < f.channel_count; ++i) {
            dst[i] = (uint16_t)(s_cal_sum[i] / n);
            dbg_printf(" %u", dst[i]);
        }
        if (s_cal_is_white) s_have_white = true; else s_have_black = true;
        dbg_write("\r\n");
        if (!s_have_white) dbg_write("next: cal black\r\n");
        else if (!s_have_black) dbg_write("next: cal white\r\n");
        else dbg_write("now : cal apply\r\n");
    }
}

static void print_calibration(void)
{
    line_calibration_data_t cal;
    uint8_t i;

    line_processing_get_calibration(&cal);
    dbg_printf("calibrated=%d\r\n", (cal.calibrated ? 1 : 0));
    dbg_write("black:");
    for (i = 0U; i < cal.channel_count; ++i) dbg_printf(" %u", cal.black[i]);
    dbg_write("\r\nwhite:");
    for (i = 0U; i < cal.channel_count; ++i) dbg_printf(" %u", cal.white[i]);
    dbg_write("\r\nthr  :");
    for (i = 0U; i < cal.channel_count; ++i) dbg_printf(" %u", cal.thresholds[i]);
    dbg_write("\r\n");
}

static void handle_cal(const char *arg)
{
    if (arg == NULL) {
        print_calibration();
        return;
    }
    if (strcmp(arg, "white") == 0) {
        cal_begin(true);
    } else if (strcmp(arg, "black") == 0) {
        cal_begin(false);
    } else if (strcmp(arg, "apply") == 0) {
        if (!s_have_white || !s_have_black) {
            dbg_write("cal: need both white & black samples first\r\n");
            return;
        }
        if (line_processing_apply_points(s_cal_black, s_cal_white)) {
            dbg_write("cal: applied, thresholds updated\r\n");
            print_calibration();
        } else {
            dbg_write("cal: rejected (span too small on some channel,\r\n"
                      "     check sensor distance / lighting)\r\n");
        }
    } else if (strcmp(arg, "show") == 0) {
        print_calibration();
    } else if (strcmp(arg, "reset") == 0) {
        line_processing_init();
        dbg_write("cal: reset to defaults (uncalibrated)\r\n");
    } else {
        dbg_write("cal: unknown subcommand\r\n");
    }
}

/* ===================== 命令解析 ===================== */

/* 把行拆成最多 4 个空格分隔的词，返回个数；在原地写 '\0' 分割。 */
static uint8_t split_words(char *line, char *words[], uint8_t max_words)
{
    uint8_t n = 0U;
    char *p = line;

    while ((*p == ' ') || (*p == '\t')) ++p;
    while (*p != '\0') {
        if (n < max_words) words[n] = p;
        ++n;
        while (*p != '\0' && *p != ' ' && *p != '\t') ++p;
        if (*p != '\0') *p++ = '\0';
        while (*p == ' ' || *p == '\t') ++p;
    }
    return n;
}

static int parse_int(const char *s, bool *ok)
{
    int value = 0;
    bool neg = false;

    *ok = false;
    if (s == NULL || *s == '\0') return 0;
    if (*s == '-') { neg = true; ++s; }
    if (*s == '\0') return 0;
    while (*s >= '0' && *s <= '9') {
        value = value * 10 + (*s - '0');
        ++s;
    }
    if (*s != '\0') return 0;            /* 有残留字符：不是纯数字 */
    *ok = true;
    return neg ? -value : value;
}

static void handle_line(char *line)
{
    char *words[4];
    uint8_t n = split_words(line, words, 4U);

    if (n == 0U) return;
    dbg_printf("# %s\r\n", words[0]);

    if (strcmp(words[0], "help") == 0 || strcmp(words[0], "?") == 0) {
        print_help();
    } else if (strcmp(words[0], "stat") == 0) {
        print_stat();
    } else if (strcmp(words[0], "raw") == 0) {
        print_raw();
    } else if (strcmp(words[0], "start") == 0) {
        line_task_set_auto_start(true);
        dbg_write(line_task_start() ? "started\r\n"
                                     : "start failed (motor?)\r\n");
    } else if (strcmp(words[0], "stop") == 0) {
        line_task_set_auto_start(false);   /* 关掉自动起步，否则停不住 */
        line_task_stop();
        dbg_write("stopped (auto-start disabled)\r\n");
    } else if (strcmp(words[0], "reset") == 0) {
        line_task_reset_fault();
        dbg_write("fault cleared\r\n");
    } else if (strcmp(words[0], "stream") == 0) {
        if (n >= 2U && strcmp(words[1], "off") == 0) {
            s_stream_period_ms = 0U;
            dbg_write("stream off\r\n");
        } else if (n >= 2U) {
            bool ok;
            int ms = parse_int(words[1], &ok);
            if (ok && ms >= 20 && ms <= 10000) {
                s_stream_period_ms = (uint32_t)ms;
                s_stream_last_ms = hal_time_millis();
                dbg_printf("stream every %d ms\r\n", ms);
            } else {
                dbg_write("stream: 20..10000 ms or off\r\n");
            }
        } else {
            dbg_printf("stream period=%u ms\r\n", s_stream_period_ms);
        }
    } else if (strcmp(words[0], "gain") == 0) {
        int16_t base, kp, kd;
        if (n >= 4U) {
            bool ok1, ok2, ok3;
            int b = parse_int(words[1], &ok1);
            int k = parse_int(words[2], &ok2);
            int d = parse_int(words[3], &ok3);
            if (ok1 && ok2 && ok3 &&
                line_task_set_gains((int16_t)b, (int16_t)k, (int16_t)d)) {
                dbg_write("gain set\r\n");
            } else {
                dbg_write("gain: rejected (must be IDLE, or out of range)\r\n");
            }
        } else {
            line_task_get_gains(&base, &kp, &kd);
            dbg_printf("base=%d kp(m)= %d kd(m)= %d\r\n", base, kp, kd);
        }
    } else if (strcmp(words[0], "trim") == 0) {
        if (n >= 3U) {
            bool ok1, ok2;
            int l = parse_int(words[1], &ok1);
            int r = parse_int(words[2], &ok2);
            if (ok1 && ok2 &&
                line_task_set_motor_trim((uint16_t)l, (uint16_t)r)) {
                dbg_write("trim set\r\n");
            } else {
                dbg_write("trim: rejected (IDLE only, 850..1150)\r\n");
            }
        } else {
            dbg_write("trim: usage trim <left> <right> (permille 850..1150)\r\n");
        }
    } else if (strcmp(words[0], "cal") == 0) {
        handle_cal((n >= 2U) ? words[1] : NULL);
    } else {
        dbg_write("unknown command, try help\r\n");
    }
}

/* ===================== 对外接口 ===================== */

void uart_debug_init(void)
{
    /* UART0 的时钟/波特率/引脚复用全部由 SysConfig 生成
     * （SYSCFG_DL_UART_0_init，在 hal_system_bringup 里调用）。
     * 这里只挂收发的环形缓冲与中断。 */
    uart_async_init();

    dbg_write("\r\n=== line_follow mspm0 debug ===\r\n"
              "type help. live data: stream 200\r\n");
}

void uart_debug_poll(void)
{
    uint32_t now;

    /* ---- 1. 校准采样推进（在命令处理之后、有新帧时累加） ---- */
    if (s_cal_sample_count < s_cal_sample_target) {
        cal_sample_step();
    }

    /* ---- 2. 接收：从 uart_async 环读取（RX 由 HAL 中断填充） ---- */
    {
        char c;
        while (uart_async_read(&c)) {
            if (c == '\r' || c == '\n') {
                if (s_rx_len > 0U) {
                    s_rx_line[s_rx_len] = '\0';
                    handle_line(s_rx_line);
                    s_rx_len = 0U;
                }
            } else if (s_rx_len < (RX_LINE_MAX - 1U)) {
                /* 简单行编辑：退格删一个字符 */
                if (c == '\b') {
                    if (s_rx_len > 0U) --s_rx_len;
                } else {
                    s_rx_line[s_rx_len++] = c;
                }
            } else {
                s_rx_len = 0U;                        /* 超长：整行丢弃 */
            }
        }
    }

    /* ---- 3. 周期打印 ---- */
    now = hal_time_millis();
    if ((s_stream_period_ms > 0U) &&
        ((uint32_t)(now - s_stream_last_ms) >= s_stream_period_ms)) {
        s_stream_last_ms = now;
        print_stat();
    }
}
