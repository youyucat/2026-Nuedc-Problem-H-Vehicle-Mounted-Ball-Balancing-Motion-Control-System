/**
  ******************************************************************************
  * @file    ball_ctrl.c
  * @brief   H 题要求4：摆杆稳球控制
  *          控制律二选一（BALL_USE_LQR，见 ball_ctrl.h）：
  *            0 = 滑模 PD + 回零偏学习 + 静摩擦脱困脉冲（默认，
  *                移植自成熟参考工程 empty_LP_MSPM0G3507 ball_balance 控制模式1）
  *            1 = LQR 状态反馈 + 积分增广 + 静摩擦脱困脉冲（Core/lqr/，
  *                原理与整定见 Core/lqr/README_LQR.md）
  *          并适配本工程机械约定（+角度 = 摆杆 +x 端/画面左端抬高）。
  *
  * 与参考工程的差异（其余逐行对应移植）：
  *   1. 符号翻转：参考工程机械 +角使 +x 端下降（其稳定律为 u=-KP·x-KD·v）；
  *      本工程台架实测 +角抬高 +x 端（M,2000 测试，旧串级版同号验证可稳球），
  *      故 P/D/偏置/脱困脉冲/方向约束/前馈全部取反号。
  *   2. 目标位置：参考工程目标恒为管中心；本工程支持 R 帧设定目标（要求6），
  *      滑模变量统一改为 e = 球位置 - 目标位置。
  *   3. 车加速度来源：参考工程用 IMU/轮速差分；本工程由车主控 A 帧直接给。
  *   4. 丢球不退出任务（保持 600ms 最后指令 → 12°/s 回平），W1/W2 仅告警。
  *
  * 控制（100Hz tick；滤波/测速/控制律由 50Hz 视觉帧驱动）：
  *   测速  ：近 2 帧位置差分 / 实测帧周期 + EMA（相机速度字段偏小，不用）
  *   预测  ：e_pred = e_ema + v·120ms（提前刹车，减少过冲）
  *   PD    ：u = +KP·e_pred + KD·v（无积分；KP=0.22°/mm，KD=0.05°/(mm/s)）
  *   限幅  ：|e|>40mm 松弛到 ±14° 加速回正，近端 ±10° 防过冲
  *   偏置  ：回零偏学习（机械零不水平的常驻小倾角，替代积分，±3°）
  *   脱困  ：球准静止 200ms 后发渐宽静摩擦脉冲（近端至 5°、远端至 9°）
  *   约束  ：输出只朝目标方向推球（球快速滚向目标时允许同侧刹车）
  *   前馈  ：车主控 A 帧加速度线性映射 mdeg（绕过方向约束直接叠加）
  *   输出  ：斜率限幅 60°/s（减速/换向 120°/s，回平 12°/s）+ 启动稳定窗 400ms
  *           + 微调死区 0.12°（稳定球不追针）
  *
  * 视觉输入 = MaixCam 12 字节二进制帧（0xAA55 头 + CRC16，20ms 恒发，
  * 含位置/速度/置信度/标志位），ASCII 帧（A/M/Z/R/B）共线兼容。
  * 检出有效 = 链路新鲜(≤150ms) + 摆杆已标定(帧标志bit1) + 检出(bit0)
  *            + 置信度≥35。位置一帧跳变 >4cm 连续 3 帧确认才接受。
  *
  * 丢球行为：保持最后指令 600ms → 限速回平（任务保持运行，球回视野自动
  * 继续）；W1 丢球减速 / W0 重捕获 / W2 回平停车告警。
  *
  * 本文件属于用户代码，CubeMX 重新生成工程时不会被覆盖。
  ******************************************************************************
  */

#include "ball_ctrl.h"
#include "ball_lqr.h"
#include "stepper.h"
#include "main.h"
#include <stdlib.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

/* ===================== 机械换算 ===================== */

/* 摆杆每度对应多少步进脉冲。默认步进直连摆杆（1:1）；
 * 若有减速齿轮/曲柄连杆，把实测传动比乘上来，例如 1:3 → * 3.0f */
#define ROD_STEPS_PER_DEG   ((float)STEPPER_STEPS_PER_REV / 360.0f * ROD_DIR_SIGN)

#define CTRL_DT_MS          10U         /* 控制周期 ms（tick_10ms 调用） */

/* "视觉没给速度"的哨兵值（ASCII B 帧兼容用；速度实际由差分测速给出） */
#define BALL_V_NA           INT32_MIN

/* 差分测速环形窗容量（测速窗口 = BALL_V_DIFF_FRAMES，需 window+1 个槽） */
#define BALL_DIFF_WINDOW_MAX 4U

/* ===================== 内部状态 ===================== */

static volatile uint8_t  s_running    = 0;   /* 1 = 要求4稳球进行中 */
static uint8_t           s_fault      = 0;   /* 恒 0：参考设计丢球不退出任务 */
static int32_t           s_level_pos  = 0;   /* 水平零位对应的步进绝对位置 */
static int32_t           s_angle_mdeg = 0;   /* 当前输出角（限速后，遥测用） */

/* --- 视觉输入 --- */
static volatile int32_t  s_ball_01mm  = 0;   /* 最近一次有效检出的球偏移（0.1mm） */
static volatile uint8_t  s_conf       = 255U;/* 最新置信度，255 = 视觉未提供 */
static volatile int32_t  s_accel      = 0;   /* 最新车纵向加速度（mm/s^2） */
static volatile uint32_t s_good_tick  = 0;   /* 最近一次有效检出时刻 */
static volatile uint32_t s_link_tick  = 0;   /* 最近一次合法帧（含未检出）时刻 */
static uint8_t           s_ball_ok    = 0;   /* tick 判定：检出有效且新鲜 */
static uint8_t           s_ever_det   = 0;   /* 启动以来是否曾锁定过球 */

/* --- 球目标 --- */
static volatile int32_t  s_xref_01mm  = 0;   /* 球目标位置（0.1mm），R 帧设定 */

/* --- 位置滤波 + 差分测速 --- */
static int32_t  s_x_ema_01mm = 0;            /* 位置 EMA */
static int32_t  s_x_hist[BALL_DIFF_WINDOW_MAX];
static uint8_t  s_xh_write = 0;
static uint8_t  s_xh_count = 0;
static uint32_t s_last_diff_frame = 0;       /* 上次参与差分的帧号 */
static uint32_t s_frame_period_ms = 0;       /* 实测帧周期 EMA */
static int32_t  s_v_est   = 0;               /* 测速输出 mm/s */
static uint8_t  s_filter_ready = 0;          /* 0=待种入（首帧/重捕获后） */

/* --- 跳变拒绝（一帧 >4cm 连续 3 帧确认才算重捕获） --- */
static int32_t  s_jump_pos    = 0;
static uint32_t s_jump_frames = 0;
static uint32_t s_jump_last_ms = 0;

/* --- 回零偏学习 --- */
static int32_t  s_bias_mdeg = 0;
static uint32_t s_bias_ms   = 0;             /* 准静止持续时间 */
static int32_t  s_bias_sign = 0;             /* 上次偏差符号（换边检测） */

/* --- 静摩擦脱困脉冲 --- */
static uint32_t s_stick_ms    = 0;           /* 准静止持续 */
static int32_t  s_stick_boost = 0;           /* 当前脉冲输出 mdeg */
static int32_t  s_stick_memory = 0;          /* 上次脱困幅度记忆 mdeg */

/* --- 输出与丢球管理 --- */
static uint32_t s_last_proc_frame = 0;       /* 上次已参与控制的帧号 */
static uint32_t s_last_ctrl_ms = 0;          /* 上次控制律运行时刻 */
static int32_t  s_req_mdeg     = 0;          /* 最近一次完整请求角（保持期沿用） */
static int32_t  s_alpha_cmd_mdeg = 0;        /* 限速后指令角 */
static int32_t  s_fb_mdeg      = 0;          /* 遥测：反馈项（PD+偏置+脉冲） */
static int32_t  s_ff_mdeg      = 0;          /* 遥测：惯性前馈 */
static uint8_t  s_sat          = 0;          /* 遥测：反馈项被角度限幅 */
static int32_t  s_drive_dir    = 0;          /* 当前推球方向（+1=正角往回推） */
static uint32_t s_start_ms     = 0;          /* 启动时刻（启动稳定窗） */
static uint8_t  s_warn_sent    = 0;          /* W1 已发 */
static uint8_t  s_warn2_sent   = 0;          /* W2 已发 */
static uint8_t  s_lost_phase   = 0;          /* 0=可用 1=保持期 2=回平 */

/* --- 台架手动模式（M 帧）：绕过控制律直接指定摆角，用于标定方向 --- */
static uint8_t  s_manual     = 0;
static int32_t  s_manual_mdeg = 0;

/* ===================== UART 接收（环形缓冲） ===================== */

static uint8_t  s_rx_byte;                          /* HAL 单字节接收缓冲 */
#define RX_RING_SIZE        64U                     /* 必须是 2 的幂 */
#define RX_RING_MASK        (RX_RING_SIZE - 1U)
static volatile uint8_t s_ring[RX_RING_SIZE];
static volatile uint8_t s_head = 0, s_tail = 0;

#define LINE_BUF_LEN        24U
static char     s_line[LINE_BUF_LEN];
static uint8_t  s_line_len = 0;
static uint8_t  s_line_dirty = 0;       /* 行内混入二进制/噪声字节 -> 整行丢弃 */

/* ===================== MaixCam 二进制帧解析 ===================== */

/* 帧格式（视觉程序 VisionUartLink.send，每 20ms 恒发）：
 *   [0]=0xAA [1]=0x55 [2]=类型(1) [3]=seq [4]=flags
 *   [5..6]=位置 int16 LE 0.1mm   [7..8]=速度 int16 LE mm/s
 *   [9]=置信度0~100  [10..11]=CRC16-CCITT(前10字节) LE
 *   flags: bit0=检出球 bit1=摆杆标定完成 bit2=跟踪有效 */
#define VIS_FRAME_LEN       12U
#define VIS_HDR0            0xAAU
#define VIS_HDR1            0x55U
#define VIS_TYPE_BALL       0x01U
#define VIS_FLAG_BALL       0x01U
#define VIS_FLAG_CAL        0x02U
#define VIS_FLAG_TRACK      0x04U

static uint8_t  s_bin_state = 0;                /* 0=找帧头 1=收帧中 */
static uint8_t  s_bin_idx  = 0;
static uint8_t  s_bin_buf[VIS_FRAME_LEN];

static volatile uint8_t  s_vision_cal = 0;      /* 摆杆标定完成标志 */
static volatile uint32_t s_frame_cnt  = 0;      /* 合法帧计数 */
static volatile uint32_t s_crc_err    = 0;      /* 坏帧计数 */

/** CRC16-CCITT：多项式 0x1021，初值 0xFFFF，MSB 先行（与视觉程序一致） */
static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFU;
    uint32_t i;
    uint8_t  j;

    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (j = 0; j < 8U; j++) {
            if (crc & 0x8000U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/**
 * 二进制帧字节状态机。返回 1 = 该字节已被帧消费（不进 ASCII 解析器），
 * 返回 0 = 该字节与帧无关（转给 ASCII 解析器）。
 */
static uint8_t ball_ctrl_bin_parse(uint8_t b)
{
    uint16_t crc_calc, crc_recv;

    if (s_bin_state == 0U) {                    /* 找帧头 */
        if (b != VIS_HDR0) return 0U;
        s_bin_buf[0] = b;
        s_bin_idx  = 1U;
        s_bin_state = 1U;
        return 1U;
    }

    if (s_bin_idx == 1U) {                      /* 第二字节必须是 0x55 */
        if (b != VIS_HDR1) {
            if (b == VIS_HDR0) {
                s_bin_idx = 1U;                 /* 连续 0xAA：仍是头 */
            } else {
                s_bin_state = 0U;
            }
            return 1U;
        }
        s_bin_buf[1] = b;
        s_bin_idx = 2U;
        return 1U;
    }

    /* 帧内：继续收 */
    s_bin_buf[s_bin_idx++] = b;
    if (s_bin_idx < VIS_FRAME_LEN) return 1U;

    /* 收满一帧：校验并分发 */
    s_bin_state = 0U;
    s_bin_idx   = 0U;

    crc_calc = crc16_ccitt(s_bin_buf, 10U);
    crc_recv = (uint16_t)(s_bin_buf[10] | ((uint16_t)s_bin_buf[11] << 8));

    if (crc_calc != crc_recv || s_bin_buf[2] != VIS_TYPE_BALL) {
        s_crc_err++;                            /* 坏帧：丢弃，重新同步 */
        return 1U;
    }

    s_frame_cnt++;
    s_vision_cal = (s_bin_buf[4] & VIS_FLAG_CAL) ? 1U : 0U;
    ball_ctrl_feed_vision(
        (int32_t)(int16_t)(s_bin_buf[5] | ((uint16_t)s_bin_buf[6] << 8)),
        (int32_t)(int16_t)(s_bin_buf[7] | ((uint16_t)s_bin_buf[8] << 8)),
        s_bin_buf[9],
        s_bin_buf[4]);
    return 1U;
}

/* ===================== 内部小工具 ===================== */

static float limitf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

/** 给车主控的警告帧（TX 线，车主控可选监听） */
static void w_send(const char *s)
{
    (void)HAL_UART_Transmit(&huart1, (const uint8_t *)s, (uint16_t)strlen(s), 20U);
}

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int32_t abs_i32(int32_t v)
{
    return (v >= 0) ? v : -v;
}

/* 整数 EMA：out += (in - out) / div；商为 0 但有残差时保底 ±1，避免小信号卡死 */
static int32_t filter_step(int32_t cur, int32_t in, int32_t div)
{
    int32_t delta = in - cur;
    int32_t step  = delta / div;

    if ((step == 0) && (delta != 0)) step = (delta > 0) ? 1 : -1;
    return cur + step;
}

/* 斜率限幅：rate mdeg/s，dt ms；最小步进 1 mdeg 保证高斜率不被整除吞掉 */
static int32_t slew_toward(int32_t cur, int32_t tgt, uint32_t rate_mdeg_s,
                           uint32_t dt_ms)
{
    int32_t delta = tgt - cur;
    int32_t max_step = (int32_t)(((uint64_t)rate_mdeg_s * dt_ms) / 1000U);

    if (max_step < 1) max_step = 1;
    if (delta >  max_step) return cur + max_step;
    if (delta < -max_step) return cur - max_step;
    return tgt;
}

/* 差分测速：近 window 帧位置差分 / 实测帧周期（20ms 只是上限，相机实际可能
 * 更慢），再 EMA 平滑。帧号不连续（丢帧）时清空历史，避免把间隔算成巨大
 * 速度。相机速度字段偏小（经验值），不直接使用。 */
static int32_t estimate_ball_velocity(int32_t pos01, uint32_t frame_cnt,
                                      uint32_t dt_ms)
{
    uint32_t window = (uint32_t)BALL_V_DIFF_FRAMES;
    uint32_t old_index, period_ms;

    if (window < 2U) window = 2U;
    if (window > (uint32_t)(BALL_DIFF_WINDOW_MAX - 1U)) {
        window = BALL_DIFF_WINDOW_MAX - 1U;
    }

    if ((s_xh_count == 0U) || (frame_cnt != s_last_diff_frame + 1U)) {
        s_xh_count = 0U;                        /* 帧不连续：历史作废重新积累 */
        s_xh_write = 0U;
    } else {
        /* 连续帧：刷新实测帧周期 EMA */
        s_frame_period_ms = (s_frame_period_ms == 0U) ? dt_ms :
                            (s_frame_period_ms + dt_ms) / 2U;
    }
    s_last_diff_frame = frame_cnt;
    s_x_hist[s_xh_write] = pos01;
    s_xh_write = (uint8_t)((s_xh_write + 1U) % BALL_DIFF_WINDOW_MAX);
    if (s_xh_count < BALL_DIFF_WINDOW_MAX) s_xh_count++;

    if (s_xh_count > window) {
        int32_t v_raw;

        old_index = (s_xh_write + BALL_DIFF_WINDOW_MAX - window - 1U) %
                    BALL_DIFF_WINDOW_MAX;
        period_ms = (s_frame_period_ms != 0U) ? s_frame_period_ms
                                              : BALL_VISION_FRAME_MS;
        if (period_ms < 5U) period_ms = 5U;
        /* 0.1mm 差分 × 100 / (window × 周期ms) → mm/s */
        v_raw = (int32_t)(((int64_t)(pos01 - s_x_hist[old_index]) * 100LL) /
                          ((int64_t)window * (int64_t)period_ms));
        s_v_est = filter_step(s_v_est, v_raw, BALL_V_DIFF_FILTER);
    }
    return s_v_est;
}

/* 远端松弛限幅：偏离目标 >40mm 时放宽到 ±14° 加速回正，近端 ±10° 防过冲
 * （"先大幅回正再微调"）。PD 限幅与合成指令限幅共用，保证松弛真的生效。 */
static int32_t active_angle_limit(int32_t err01)
{
    if (abs_i32(err01) > BALL_RETURN_DIST_01MM) return BALL_RETURN_MAX_MDEG;
    return BALL_CONTROL_MAX_MDEG;
}

/* 回零偏学习：机械零不水平需要常驻小倾角顶着（替代积分项）。
 * 球准静止且偏离目标 1.5~60mm、|v|≤15mm/s 持续 150ms 后慢速积累；
 * ±4° 限幅、±6°/s 限速；球换边（|e|>2.5mm 处符号翻转）保留 70%，
 * 缓慢洗掉学错的方向。
 * （仅滑模 PD 模式编译：LQR 模式由积分项替代，见 ball_lqr.c） */
#if !BALL_USE_LQR
static void settle_bias_update(int32_t err01, int32_t v, uint32_t dt_ms)
{
    uint8_t eligible;
    int32_t rate, step;

    eligible = ((abs_i32(err01) >= BALL_BIAS_MIN_POS_01MM) &&
                (abs_i32(err01) <= BALL_BIAS_MAX_POS_01MM) &&
                (abs_i32(v) <= BALL_BIAS_MAX_VEL_MM_S)) ? 1U : 0U;

    if (abs_i32(err01) >= BALL_BIAS_FLIP_POS_01MM) {
        int32_t sign = (err01 > 0) ? 1 : -1;

        if ((s_bias_sign != 0) && (sign != s_bias_sign)) {
            s_bias_mdeg = (int32_t)(((int64_t)s_bias_mdeg *
                                     BALL_BIAS_FLIP_RETAIN) / 100);
        }
        s_bias_sign = sign;
    }

    if (eligible) {
        if (s_bias_ms < BALL_BIAS_CONFIRM_MS) {
            s_bias_ms += dt_ms;
        } else {
            /* 本工程 +角抬高 +x 端：球停在目标 +x 侧需要正偏置往回顶 */
            rate = (int32_t)(((int64_t)err01 * BALL_BIAS_KI) / 10);
            rate = clamp_i32(rate, -BALL_BIAS_MAX_RATE, BALL_BIAS_MAX_RATE);
            step = (int32_t)(((int64_t)rate * dt_ms) / 1000);
            if ((step == 0) && (rate != 0)) step = (rate > 0) ? 1 : -1;
            s_bias_mdeg = clamp_i32(s_bias_mdeg + step,
                                    -BALL_BIAS_LIMIT_MDEG,
                                     BALL_BIAS_LIMIT_MDEG);
        }
    } else {
        uint32_t decay = dt_ms * 3U;

        if (decay >= s_bias_ms) s_bias_ms = 0U; else s_bias_ms -= decay;
    }
}
#endif /* !BALL_USE_LQR */

/* 静摩擦脱困：持续饱和倾角可能把粘滞球一直顶死；改成"斜坡-保持-卸力"
 * 脉冲，让管在两次尝试间卸力，逐脉冲加宽直到突破真实启动摩擦阈值，
 * 球一动立即收力。近端（|e|<10mm）首脉冲 5°、每脉冲 +0.5°、上限 7.5°；
 * 远端首脉冲 4°、每脉冲 +1.5°、上限 12°，并记忆上次脱困幅度的 90%。 */
static int32_t stiction_recovery_output(int32_t err01, int32_t v, uint32_t dt_ms)
{
    int32_t  out = 0;
    int32_t  amp, esc, cap;
    uint32_t active_ms, cycle_ms, phase_ms, pulse_idx;
    uint8_t  transit;

    if ((abs_i32(err01) >= BALL_STICTION_POS_01MM) &&
        (abs_i32(v) <= BALL_STICTION_VEL_MM_S)) {
        s_stick_ms += dt_ms;
        s_stick_boost = 0;

        if (s_stick_ms > BALL_STICTION_HOLD_MS) {
            active_ms = s_stick_ms - BALL_STICTION_HOLD_MS;
            cycle_ms  = BALL_STICK_RAMP_MS + BALL_STICK_HOLD_MS +
                        BALL_STICK_RELEASE_MS;
            pulse_idx = active_ms / cycle_ms;
            if (pulse_idx > 8U) pulse_idx = 8U;
            phase_ms  = active_ms % cycle_ms;
            transit   = (abs_i32(err01) >= BALL_STICK_TRANSIT_POS_01MM) ? 1U : 0U;

            amp = transit ? BALL_STICK_TRA_BOOST : BALL_STICK_NEAR_BOOST;
            esc = transit ? BALL_STICK_TRA_STEP  : BALL_STICK_NEAR_STEP;
            cap = transit ? BALL_STICK_TRA_MAX   : BALL_STICK_NEAR_MAX;
            if (transit && (s_stick_memory > amp)) amp = s_stick_memory;
            amp += (int32_t)pulse_idx * esc;
            if (amp > cap) amp = cap;

            if (phase_ms < BALL_STICK_RAMP_MS) {
                s_stick_boost = (int32_t)(((int64_t)amp * phase_ms) /
                                          BALL_STICK_RAMP_MS);
            } else if (phase_ms < (BALL_STICK_RAMP_MS + BALL_STICK_HOLD_MS)) {
                s_stick_boost = amp;
            }
            /* 卸力段（保持期结束后）：boost 保持 0 */
        }

        if (err01 > 0)      out = s_stick_boost;   /* +x 侧 → 正脉冲往回推 */
        else if (err01 < 0) out = -s_stick_boost;
    } else if (abs_i32(v) > BALL_STICK_TRANSIT_VEL) {
        /* 球动起来了：远端脱困成功的话记住这次的幅度 */
        if ((abs_i32(err01) >= BALL_STICK_TRANSIT_POS_01MM) &&
            (s_stick_boost > 0)) {
            s_stick_memory = (int32_t)(((int64_t)s_stick_boost *
                                        BALL_STICK_MEMORY_PCT) / 100);
        }
        s_stick_ms    = 0U;
        s_stick_boost = 0;
    } else {
        /* 慢速但不算静止：静止计时按 3 倍速衰减，轻微抖动不清零 */
        uint32_t decay = dt_ms * 3U;

        if (decay >= s_stick_ms) s_stick_ms = 0U; else s_stick_ms -= decay;
        s_stick_boost = 0;
    }
    return out;
}

/* 方向约束：输出默认只允许"朝目标方向推球"。球快速滚向目标时必须允许
 * 同侧刹车输出（刹车方向与推球方向相反），否则球会无阻尼穿过目标。 */
static int32_t constrain_to_ball_side(int32_t target_mdeg, int32_t err01,
                                      int32_t v)
{
    if (err01 >= BALL_DIR_SWITCH_01MM) {
        s_drive_dir = 1;                    /* 球在目标 +x 侧：正角往回推 */
    } else if (err01 <= -BALL_DIR_SWITCH_01MM) {
        s_drive_dir = -1;
    }

    if (((s_drive_dir > 0) && (target_mdeg < 0)) ||
        ((s_drive_dir < 0) && (target_mdeg > 0))) {
        if (((int64_t)err01 * v < 0) &&
            (abs_i32(v) > BALL_BRAKE_MIN_VEL_MM_S)) {
            return target_mdeg;             /* 球正滚向目标：允许刹车 */
        }
        return 0;
    }
    return target_mdeg;
}

/* 新视觉帧：滤波/测速 + 控制律 + 脱困脉冲，返回反馈项输出 mdeg。
 * dt_ms = 距上次控制律的实测间隔（≈帧周期）。
 * 控制律二选一（BALL_USE_LQR，见 ball_ctrl.h 配置区）：
 *   滑模 PD：预测误差 e_pred = e + v·120ms → PD + 回零偏学习
 *   LQR    ：原始误差（真实状态反馈，前推会与 v 项双重计阻尼）→
 *            K1·e + K2·v + KI·∫e（积分在 ball_lqr 内部，替代偏置学习） */
static int32_t control_update(uint32_t dt_ms)
{
    int32_t err, pd, combined, out, lim, stick;

    if (!s_filter_ready) {
        /* 首帧/重捕获：位置直接种入，测速清零（防差分踢跳） */
        s_x_ema_01mm   = s_ball_01mm;
        s_v_est        = 0;
        s_xh_count     = 0U;
        s_xh_write     = 0U;
        s_filter_ready = 1U;
    } else {
        s_x_ema_01mm = filter_step(s_x_ema_01mm, s_ball_01mm,
                                   BALL_POS_FILTER_DIV);
        s_v_est = estimate_ball_velocity(s_ball_01mm, s_frame_cnt, dt_ms);
    }

#if BALL_USE_LQR
    /* --- LQR 状态反馈 + 积分增广（Core/lqr/，推导见 README_LQR.md） --- */
    err = s_x_ema_01mm - s_xref_01mm;
    if (abs_i32(err) <= BALL_CENTER_DEADBAND_01MM) err = 0;

    lim = active_angle_limit(err);
    pd    = clamp_i32(ball_lqr_update(err, s_v_est, dt_ms, lim), -lim, lim);
    stick = stiction_recovery_output(err, s_v_est, dt_ms);

    combined = pd + stick;      /* 积分项已含稳态补偿，不叠加偏置学习 */
#else
    /* --- 滑模 PD + 回零偏学习（默认路径，与参考设计一致） --- */
    {
        int32_t pred, p, d;

        /* 预测：位置前推 120ms（提前刹车）；滑模变量 = 相对目标的偏差 */
        pred = s_x_ema_01mm + (int32_t)(((int64_t)s_v_est * BALL_LOOKAHEAD_MS) / 100);
        err  = pred - s_xref_01mm;
        if (abs_i32(err) <= BALL_CENTER_DEADBAND_01MM) err = 0;

        /* 本工程 +角抬高 +x 端：正偏差需正角往回推（参考工程机械方向相反） */
        p = (int32_t)(((int64_t)BALL_PD_KP_MDEG_PER_MM * err) / 10);
        d = (int32_t)((int64_t)BALL_PD_KD_MDEG_PER_MM_S * s_v_est);

        lim = active_angle_limit(err);
        pd  = clamp_i32(p + d, -lim, lim);

        settle_bias_update(err, s_v_est, dt_ms);
        stick = stiction_recovery_output(err, s_v_est, dt_ms);

        combined  = pd + s_bias_mdeg + stick;
    }
#endif

    out       = clamp_i32(combined, -lim, lim);
    s_sat     = (abs_i32(combined) > lim) ? 1U : 0U;
    s_fb_mdeg = out;
    return out;
}

/* 清控制状态（滤波/偏置/脉冲/跳变）。指令角 s_alpha_cmd_mdeg 故意不清零：
 * 重捕获/回平都从当前角经斜率限幅平滑过渡。 */
static void reset_control_state(void)
{
    s_x_ema_01mm = 0;
    s_xh_write = 0U;
    s_xh_count = 0U;
    s_last_diff_frame = 0U;
    s_frame_period_ms = 0U;
    s_v_est = 0;
    s_filter_ready = 0U;
    s_jump_pos = 0;
    s_jump_frames = 0U;
    s_jump_last_ms = 0U;
    s_bias_mdeg = 0;
    s_bias_ms = 0U;
    s_bias_sign = 0;
    s_stick_ms = 0U;
    s_stick_boost = 0;
    s_stick_memory = 0;
#if BALL_USE_LQR
    ball_lqr_reset();        /* LQR 模式：积分状态一并清零 */
#endif
    s_req_mdeg = 0;
    s_fb_mdeg = 0;
    s_ff_mdeg = 0;
    s_sat = 0;
    s_drive_dir = 0;
    s_last_ctrl_ms = 0U;
}

static void apply_angle_output(void);
static void ball_ctrl_parse_byte(uint8_t b);    /* 定义在文件末尾，先声明 */

/* ===================== 对外接口 ===================== */

void ball_ctrl_init(void)
{
    s_running    = 0;
    s_fault      = 0;
    s_level_pos  = 0;
    s_angle_mdeg = 0;
    s_ball_01mm  = 0;
    s_conf       = 255U;
    s_accel      = 0;
    s_good_tick  = 0;
    s_link_tick  = 0;
    s_ball_ok    = 0;
    s_ever_det   = 0;
    s_xref_01mm  = 0;
    s_alpha_cmd_mdeg = 0;
    s_manual     = 0;
    s_manual_mdeg = 0;
    s_warn_sent  = 0;
    s_warn2_sent = 0;
    s_lost_phase = 0;
    s_start_ms   = 0;
    s_head = 0; s_tail = 0;
    s_line_len = 0;
    s_line_dirty = 0;
    s_bin_state = 0U;
    s_bin_idx   = 0U;
    s_vision_cal = 0U;
    s_frame_cnt = 0U;
    s_crc_err   = 0U;

    reset_control_state();
    ball_ctrl_uart_start_rx();
}

void ball_ctrl_start(void)
{
    /* 当前电机位置定义为摆杆水平零位（上电前应先目视调平摆杆） */
    s_level_pos = stepper_position();
    s_conf      = 255U;
    s_accel     = 0;
    s_xref_01mm = 0;
    s_ball_ok   = 0;
    s_manual    = 0;
    s_fault     = 0;
    s_alpha_cmd_mdeg = 0;
    s_warn_sent  = 0;
    s_warn2_sent = 0;
    s_lost_phase = 0;
    s_start_ms   = HAL_GetTick();    /* 启动稳定窗从现在起算 */
    s_good_tick  = s_start_ms;       /* 丢球计时从启动时刻起算 */
    s_link_tick  = s_start_ms;
    reset_control_state();
    s_running = 1;
}

void ball_ctrl_stop_level(void)
{
    s_running = 0;
    s_manual  = 0;
    reset_control_state();
    s_alpha_cmd_mdeg = 0;
    stepper_goto(s_level_pos);       /* 平滑回到水平零位 */
}

uint8_t ball_ctrl_is_running(void) { return s_running; }

void ball_ctrl_relevel(void)
{
    s_level_pos = stepper_position();
}

/**
 * 视觉检出帧：球位置入口（二进制帧 / ASCII B 帧共用）。
 * @param x01mm  球偏移 0.1mm（画面左 = 正）
 * @param v_mms  相机速度字段（偏小，不使用；速度由差分测速给出）
 * @param conf   置信度 0~100；255 = 帧没带置信度
 *
 * 低置信度帧只更新置信度显示，不更新位置、不刷新计时（按丢球处理）。
 * 一帧跳变 >4cm 视为误检/重捕获候选：连续 3 帧在同一位置附近才接受，
 * 接受时复位滤波在新位置重新种入。
 */
void ball_ctrl_feed_ball(int32_t x01mm, int32_t v_mms, uint8_t conf)
{
    uint32_t now = HAL_GetTick();

    (void)v_mms;

    if (conf != 255U) {
        s_conf = conf;
        if (conf < (uint8_t)BALL_CONF_MIN) return;   /* 低置信度：按丢球处理 */
    }

    if (s_filter_ready &&
        (abs_i32(x01mm - s_ball_01mm) > BALL_JUMP_01MM)) {
        if ((s_jump_frames != 0U) &&
            (abs_i32(x01mm - s_jump_pos) <= BALL_JUMP_01MM)) {
            s_jump_frames++;
            s_jump_pos = (s_jump_pos + x01mm) / 2;   /* 候选位置滑动平均 */
        } else {
            s_jump_pos    = x01mm;
            s_jump_frames = 1U;
        }
        s_jump_last_ms = now;
        if (s_jump_frames < BALL_JUMP_CONFIRM) return;   /* 未确认：丢弃 */
        /* 连续确认 = 真实重捕获：复位滤波/脱困，在新位置重新种入 */
        s_filter_ready = 0U;
        s_stick_ms     = 0U;
        s_stick_boost  = 0;
        s_jump_frames  = 0U;
        s_jump_last_ms = 0U;
    } else if (s_jump_frames != 0U) {
        s_jump_frames  = 0U;                     /* 回到正常位置：候选作废 */
        s_jump_last_ms = 0U;
    }

    s_ball_01mm = x01mm;
    s_good_tick = now;
    s_ever_det  = 1U;
}

/**
 * MaixCam 二进制帧入口（由帧解析器调用）
 * @param x01mm  球偏移 0.1mm（画面左 = 正）；未检出帧无意义
 * @param v_mms  球速度 mm/s；未检出帧无意义
 * @param conf   置信度 0~100
 * @param flags  bit0=检出球 bit1=摆杆已标定 bit2=跟踪有效
 *
 * 未检出帧（bit0=0）：只刷新链路新鲜度，不刷新检出计时 —— 丢球逻辑
 * 照常运行，位置/速度保持上次值供遥测观察。
 */
void ball_ctrl_feed_vision(int32_t x01mm, int32_t v_mms, uint8_t conf, uint8_t flags)
{
    s_link_tick = HAL_GetTick();        /* 任何合法帧都刷新链路新鲜度 */

    if (flags & VIS_FLAG_BALL) {
        ball_ctrl_feed_ball(x01mm, v_mms, conf);
    }
}

void ball_ctrl_feed_accel(int32_t a_mm_s2)
{
    s_accel = clamp_i32(a_mm_s2, -BALL_ACCEL_LIMIT_MM_S2, BALL_ACCEL_LIMIT_MM_S2);
}

void ball_ctrl_feed_xref(int32_t x01mm)
{
    float lim = ROD_HALF_LEN_MM * 10.0f;

    if (x01mm >  (int32_t)lim) x01mm =  (int32_t)lim;
    if (x01mm < -(int32_t)lim) x01mm = -(int32_t)lim;
    s_xref_01mm = x01mm;
}

void ball_ctrl_feed_manual(int32_t angle_mdeg)
{
    if (angle_mdeg == 0) {
        s_manual     = 0U;
        s_manual_mdeg = 0;
    } else {
        s_manual     = 1U;
        s_manual_mdeg = angle_mdeg;
    }
}

/* ===================== 指令角输出 ===================== */

/** 指令角 -> 步进绝对位置，servo 微动跟踪 */
static void apply_angle_output(void)
{
    int32_t target = s_level_pos +
        (int32_t)((float)s_alpha_cmd_mdeg * 0.001f * ROD_STEPS_PER_DEG);

    s_angle_mdeg = s_alpha_cmd_mdeg;
    stepper_servo_update(target);
}

/* ===================== 100Hz 控制节拍 ===================== */

void ball_ctrl_tick_10ms(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t age, dt, slew = BALL_SLEW_RETURN;
    uint8_t  link_fresh, det_valid, hold;
    int32_t  requested = 0, lim;

    if (!s_running) return;

    /* ---------- 台架手动：直接输出指定角度（不需要视觉） ---------- */
    if (s_manual) {
        s_fb_mdeg = 0;
        s_ff_mdeg = 0;
        s_sat     = 0;
        s_alpha_cmd_mdeg = clamp_i32(s_manual_mdeg,
                                     -BALL_RETURN_MAX_MDEG,
                                      BALL_RETURN_MAX_MDEG);
        s_angle_mdeg = s_alpha_cmd_mdeg;
        apply_angle_output();
        return;
    }

    /* ---------- 链路 / 检出有效性 ---------- */
    age = now - s_good_tick;
    link_fresh = ((s_frame_cnt != 0U) &&
                  ((now - s_link_tick) <= BALL_VISION_TIMEOUT_MS)) ? 1U : 0U;
    det_valid = ((link_fresh != 0U) && (s_vision_cal != 0U) &&
                 (age <= BALL_VISION_TIMEOUT_MS)) ? 1U : 0U;
    hold = ((link_fresh != 0U) &&
            (age <= BALL_DETECTION_HOLD_MS)) ? 1U : 0U;
    s_ball_ok = det_valid;

    /* 跳变候选超时清理（可疑位置不再重复出现就作废） */
    if ((s_jump_frames != 0U) && (s_jump_last_ms != 0U) &&
        ((now - s_jump_last_ms) > BALL_DETECTION_HOLD_MS)) {
        s_jump_frames  = 0U;
        s_jump_last_ms = 0U;
    }

    if (det_valid != 0U) {
        /* ---------- 正常控制 ---------- */
        if (s_warn_sent || s_warn2_sent) {
            w_send("W0\n");                      /* 球回来了 */
        }
        s_warn_sent  = 0U;
        s_warn2_sent = 0U;
        s_lost_phase = 0U;

        if (s_frame_cnt != s_last_proc_frame) {
            /* 新视觉帧：滤波/测速/控制律（帧周期驱动） */
            s_last_proc_frame = s_frame_cnt;
            dt = (s_last_ctrl_ms == 0U) ? BALL_VISION_FRAME_MS
                                        : (now - s_last_ctrl_ms);
            if (dt < 5U) dt = 5U;
            if (dt > 100U) dt = 100U;
            (void)control_update(dt);
            s_last_ctrl_ms = now;
        }

        /* 惯性前馈：A 帧加速度线性映射（每拍刷新，绕过方向约束直接叠加） */
        s_ff_mdeg = (int32_t)(((int64_t)s_accel * 1000 * BALL_INERTIA_DIRECTION) /
                              BALL_INERTIA_MM_S2_PER_DEG);
        s_ff_mdeg = clamp_i32(s_ff_mdeg, -BALL_INERTIA_MAX_MDEG,
                               BALL_INERTIA_MAX_MDEG);

        lim = active_angle_limit(s_x_ema_01mm - s_xref_01mm);
        requested = constrain_to_ball_side(s_fb_mdeg,
                                           s_x_ema_01mm - s_xref_01mm,
                                           s_v_est) + s_ff_mdeg;
        requested = clamp_i32(requested, -lim, lim);

        /* 启动稳定窗：滤波建立前不出力，避免启动猛摆 */
        if ((now - s_start_ms) < BALL_ENABLE_SETTLE_MS) {
            requested = 0;
        } else if (abs_i32(requested) < BALL_TARGET_DEADBAND) {
            requested = 0;                       /* 微调死区：稳定球不追针 */
        }
        s_req_mdeg = requested;

        /* 斜率选择：减速/换向用刹车斜率 120°/s，加速推球用 60°/s */
        if ((abs_i32(requested) < abs_i32(s_alpha_cmd_mdeg)) ||
            (((int64_t)requested * (int64_t)s_alpha_cmd_mdeg) < 0)) {
            slew = BALL_SLEW_BRAKE;
        } else {
            slew = BALL_SLEW_CONTROL;
        }
    } else if (hold != 0U) {
        /* ---------- 保持期：相机短暂丢检出，沿用最后指令避免来回摆 ---------- */
        s_last_ctrl_ms = now;
        requested = s_req_mdeg;
        slew = BALL_SLEW_CONTROL;
        if (s_ever_det && !s_warn_sent && (s_lost_phase == 0U)) {
            w_send("W1\n");                      /* 建议车主控减速 */
            s_warn_sent = 1U;
        }
        if (s_lost_phase == 0U) s_lost_phase = 1U;
    } else {
        /* ---------- 丢球超时/链路断：限速回平（任务保持运行） ---------- */
        if (s_lost_phase < 2U) {
            if (s_ever_det) {
                w_send("W2\n");                  /* 建议车主控停车 */
                s_warn2_sent = 1U;
            }
            reset_control_state();               /* 重捕获后在新位置重新种入 */
        }
        s_lost_phase = 2U;
        requested = 0;
        slew = BALL_SLEW_RETURN;                 /* 12°/s 缓慢回平 */
    }

    s_alpha_cmd_mdeg = slew_toward(s_alpha_cmd_mdeg, requested, slew, CTRL_DT_MS);
    apply_angle_output();
}

/* ===================== 遥测 ===================== */

int32_t ball_ctrl_ball_01mm(void)  { return s_ball_01mm; }
int32_t ball_ctrl_vel_mms(void)    { return s_v_est; }
uint8_t ball_ctrl_conf(void)       { return s_conf; }
int32_t ball_ctrl_xref_01mm(void)  { return s_xref_01mm; }
int32_t ball_ctrl_angle_mdeg(void) { return s_angle_mdeg; }
int32_t ball_ctrl_fb_mdeg(void)    { return s_fb_mdeg; }
int32_t ball_ctrl_ff_mdeg(void)    { return s_ff_mdeg; }
int32_t ball_ctrl_accel(void)      { return s_accel; }
uint8_t ball_ctrl_ball_ok(void)    { return s_ball_ok; }
uint8_t ball_ctrl_sat(void)        { return s_sat; }
uint8_t ball_ctrl_fault(void)      { return s_fault; }
uint8_t ball_ctrl_vision_cal(void) { return s_vision_cal; }
uint32_t ball_ctrl_frame_count(void) { return s_frame_cnt; }
uint32_t ball_ctrl_crc_err(void)     { return s_crc_err; }

/* ===================== UART 接收 ===================== */

void ball_ctrl_uart_start_rx(void)
{
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);
}

/* 由 USART1 RX 完成回调调用 */
void ball_ctrl_uart_rx_isr(uint8_t b)
{
    uint8_t next = (uint8_t)((s_head + 1U) & RX_RING_MASK);

    if (next != s_tail) {                /* 满了就丢最新字节，保旧数据 */
        s_ring[s_head] = b;
        s_head = next;
    }
}

uint8_t ball_ctrl_uart_rx_byte(void) { return s_rx_byte; }

/* 主循环里调用：把环形缓冲的字节喂给行解析器 */
void ball_ctrl_uart_poll(void)
{
    while (s_tail != s_head) {
        uint8_t b = s_ring[s_tail];
        s_tail = (uint8_t)((s_tail + 1U) & RX_RING_MASK);
        ball_ctrl_parse_byte(b);
    }
}

/* ===================== 行解析（v2：多字段 + 可选校验和） ===================== */

/**
 * 把 tag 后面的内容拆成最多 max_n 个 long："123" 或 ",123,-45,90"
 * 返回字段个数
 */
static uint8_t ball_ctrl_split_fields(char *s, long *f, uint8_t max_n)
{
    uint8_t n = 0;

    if (*s == ',') s++;
    if (*s == '\0') return 0U;

    while (n < max_n) {
        char *end;
        long  v = strtol(s, &end, 10);

        if (end == s) break;            /* 没有数字了 */
        f[n++] = v;
        if (*end != ',') break;
        s = end + 1;
    }
    return n;
}

/** 一行收完了：校验和 -> 按首字母分发 */
static void ball_ctrl_line_dispatch(void)
{
    char   *star, *p, tag;
    long    f[3];
    uint8_t n;
    uint8_t xorsum = 0U;

    if (s_line_len == 0U) return;

    /* --- 可选 XOR 校验和：*HH = 星号前所有字节的异或（十六进制） --- */
    star = strchr(s_line, '*');
    if (star != NULL) {
        long sum;
        char *end;

        for (p = s_line; p < star; p++) xorsum ^= (uint8_t)*p;
        sum = strtol(star + 1, &end, 16);
        if (end == star + 1) return;                 /* * 后没有数：丢帧 */
        if ((long)xorsum != (sum & 0xFFL)) return;   /* 校验失败：丢帧 */
        *star = '\0';
    }

    tag = s_line[0];
    n = ball_ctrl_split_fields(&s_line[1], f, 3U);

    switch (tag) {
    case 'B': case 'b':                              /* 球状态 */
        if (n >= 1U) {
            ball_ctrl_feed_ball((int32_t)f[0],
                                (n >= 2U) ? (int32_t)f[1] : BALL_V_NA,
                                (n >= 3U) ? (uint8_t)limitf((float)f[2], 0.0f, 100.0f)
                                          : 255U);
        }
        break;

    case 'A': case 'a':                              /* 车纵向加速度 */
        if (n >= 1U) ball_ctrl_feed_accel((int32_t)f[0]);
        break;

    case 'R': case 'r':                              /* 球目标位置（要求6用） */
        if (n >= 1U) ball_ctrl_feed_xref((int32_t)f[0]);
        break;

    case 'M': case 'm':                              /* 台架手动摆角 */
        if (n >= 1U) ball_ctrl_feed_manual((int32_t)f[0]);
        break;

    case 'Z': case 'z':                              /* 水平基准清零 */
        ball_ctrl_relevel();
        break;

    default:
        break;                                       /* 未知帧忽略 */
    }
}

/** 单字节 ASCII 行解析：'\n' 为帧尾，'\r' 忽略，混入噪声/超长整行丢弃 */
static void ball_ctrl_ascii_parse_byte(uint8_t b)
{
    if (b == '\r') return;

    if (b == '\n') {
        if (!s_line_dirty) {
            s_line[s_line_len] = '\0';
            ball_ctrl_line_dispatch();
        }
        s_line_len  = 0U;
        s_line_dirty = 0U;
        return;
    }

    /* 不可打印字节：当前行作废（防二进制噪声伪造 ASCII 帧） */
    if (b < 0x20U || b > 0x7EU) {
        s_line_dirty = 1U;
        return;
    }

    if (s_line_len < (LINE_BUF_LEN - 1U)) {
        s_line[s_line_len++] = (char)b;
    } else {
        s_line_len   = 0U;
        s_line_dirty = 1U;                       /* 超长帧丢弃 */
    }
}

/** 字节总入口：先给二进制帧状态机，不是帧内容的才进 ASCII 解析 */
static void ball_ctrl_parse_byte(uint8_t b)
{
    if (ball_ctrl_bin_parse(b)) return;
    ball_ctrl_ascii_parse_byte(b);
}
