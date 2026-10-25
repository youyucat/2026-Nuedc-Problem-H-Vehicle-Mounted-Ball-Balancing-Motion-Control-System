/*
 * 串口调试模块：UART0 手写 DriverLib 初始化 + 行命令解析。
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
#include "services/line_processing.h"

/* ===================== 波特率分频（BUSCLK = 40 MHz） ===================== */

#define DBG_CLK_HZ     40000000UL   /* ULPCLK = MCLK(80MHz)/2 */
#define DBG_BAUD       115200UL
/* divisor = clk/(16*baud) = 21.701 -> IBRD=21, FBRD=round(0.701*64)=45，
 * 实际波特率 115191，误差 0.008%。 */
#define DBG_IBRD       (DBG_CLK_HZ / (16UL * DBG_BAUD))
#define DBG_FBRD       ((uint32_t)((((DBG_CLK_HZ % (16UL * DBG_BAUD)) * 64UL) \
                        + (8UL * DBG_BAUD)) / (16UL * DBG_BAUD)))

/* ===================== 发送：环形缓冲 + TX 中断 ===================== */

#define TX_BUF_SIZE  256U   /* 2 的幂，配合 & (SIZE-1) 取模 */
_Static_assert((TX_BUF_SIZE & (TX_BUF_SIZE - 1U)) == 0U,
               "TX buffer size must be a power of two");

static volatile uint8_t  s_tx_buf[TX_BUF_SIZE];
static volatile uint16_t s_tx_head;      /* 写入位置 */
static volatile uint16_t s_tx_tail;      /* 发送位置 */
static volatile bool     s_tx_active;   /* TX 中断驱动中 */

/* ===================== 接收：主循环轮询 + 行缓冲 ===================== */

#define RX_LINE_MAX  48U

static char    s_rx_line[RX_LINE_MAX];
static uint8_t s_rx_len;

/* ===================== 周期打印 / 校准采样状态 ===================== */

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

/* ===================== 底层收发 ===================== */

static void tx_start(void)
{
    if (s_tx_active) return;
    if (s_tx_head == s_tx_tail) return;
    s_tx_active = true;
    /* 先送一个字节再开中断：TXIFG 因 FIFO 空置位，使能后立即进中断续发 */
    DL_UART_transmitData(UART0, s_tx_buf[s_tx_tail]);
    s_tx_tail = (uint16_t)((s_tx_tail + 1U) & (TX_BUF_SIZE - 1U));
    DL_UART_enableInterrupt(UART0, DL_UART_INTERRUPT_TX);
}

void UART0_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(UART0) == DL_UART_IIDX_TX) {
        if (s_tx_head != s_tx_tail) {
            DL_UART_transmitData(UART0, s_tx_buf[s_tx_tail]);
            s_tx_tail = (uint16_t)((s_tx_tail + 1U) & (TX_BUF_SIZE - 1U));
        } else {
            DL_UART_disableInterrupt(UART0, DL_UART_INTERRUPT_TX);
            s_tx_active = false;
        }
    }
}

static void dbg_write(const char *text)
{
    if (text == NULL) return;
    while (*text != '\0') {
        uint16_t next = (uint16_t)((s_tx_head + 1U) & (TX_BUF_SIZE - 1U));
        if (next == s_tx_tail) break;        /* 缓冲满：丢弃（不阻塞控制拍） */
        s_tx_buf[s_tx_head] = (uint8_t)*text++;
        s_tx_head = next;
    }
    tx_start();
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
    DL_UART_ClockConfig clock_config = {
        .clockSel    = DL_UART_CLOCK_BUSCLK,
        .divideRatio = DL_UART_CLOCK_DIVIDE_RATIO_1,
    };
    DL_UART_Config uart_config = {
        .mode        = DL_UART_MODE_NORMAL,
        .direction   = DL_UART_DIRECTION_TX_RX,
        .flowControl = DL_UART_FLOW_CONTROL_NONE,
        .parity      = DL_UART_PARITY_NONE,
        .wordLength  = DL_UART_WORD_LENGTH_8_BITS,
        .stopBits    = DL_UART_STOP_BITS_ONE,
    };

    DL_UART_Main_reset(UART0);
    DL_UART_Main_enablePower(UART0);
    hal_time_delay_us(16U);                       /* 上电位稳定（惯例） */

    /* PA10 = UART0_TX (PINCM21, PF2)，PA11 = UART0_RX (PINCM22, PF2)。
     * 已核对 bsp/board_profile.h：这两脚未被本工程占用。 */
    DL_UART_Main_setClockConfig(UART0, &clock_config);
    DL_UART_Main_init(UART0, &uart_config);
    DL_UART_Main_setOversampling(UART0, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART0, DBG_IBRD, DBG_FBRD);
    DL_UART_Main_setTXFIFOThreshold(UART0,
        DL_UART_TX_FIFO_LEVEL_3_4_EMPTY);
    DL_UART_Main_setRXFIFOThreshold(UART0,
        DL_UART_RX_FIFO_LEVEL_1_2_FULL);
    DL_UART_Main_enableFIFOs(UART0);
    DL_UART_Main_enable(UART0);
    DL_UART_Main_clearInterruptStatus(UART0,
        DL_UART_INTERRUPT_TX | DL_UART_INTERRUPT_RX);
    NVIC_EnableIRQ(UART0_INT_IRQn);

    dbg_write("\r\n=== line_follow mspm0 debug ===\r\n"
              "type help\r\n");
}

void uart_debug_poll(void)
{
    uint32_t now;

    /* ---- 1. 校准采样推进（在命令处理之后、有新帧时累加） ---- */
    if (s_cal_sample_count < s_cal_sample_target) {
        cal_sample_step();
    }

    /* ---- 2. 接收：轮询 RX FIFO ---- */
    while (!DL_UART_isRXFIFOEmpty(UART0)) {
        char c = (char)DL_UART_receiveData(UART0);
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

    /* ---- 3. 周期打印 ---- */
    now = hal_time_millis();
    if ((s_stream_period_ms > 0U) &&
        ((uint32_t)(now - s_stream_last_ms) >= s_stream_period_ms)) {
        s_stream_last_ms = now;
        print_stat();
    }
}
