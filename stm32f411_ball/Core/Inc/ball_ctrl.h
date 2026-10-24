/**
  ******************************************************************************
  * @file    ball_ctrl.h
  * @brief   H 题要求4：行驶过程中摆杆稳球控制
  *          （参考设计移植：滑模 PD + 回零偏学习 + 静摩擦脱困脉冲）
  *
  * 控制结构（移植自 MSPM0 参考工程 ball_balance，100Hz）：
 *   测速：近 2 帧位置差分 / 实测帧周期 + EMA（相机速度字段偏小，不直接用）
 *   预测：e_pred = e_ema + v·120ms（提前刹车，减少过冲）
 *   PD  ：u = +KP·e_pred + KD·v（无积分；KP=0.22°/mm，KD=0.05°/(mm/s)）
  *   限幅：|x|>40mm 松弛到 ±14° 加速回正，近端 ±10° 防过冲
  *   偏置：回零偏学习（机械零不水平的常驻小倾角，替代积分，±3°）
  *   脱困：球准静止 200ms 后发渐宽静摩擦脉冲（近端至 5°、远端至 9°）
  *   约束：输出只朝中心推球（快速滚向中心时允许同侧刹车）
  *   前馈：车主控 A 帧加速度线性映射 mdeg（绕过方向约束直接叠加）
  *   输出：斜率限幅 60°/s（刹车 120°/s，回平 12°/s）+ 启动稳定窗 + 微调死区
  *
  * 视觉输入（MaixCam 小球识别程序，UART1 115200，每 20ms 恒定发 12 字节帧）：
  *   字节0-1  帧头 0xAA 0x55；字节2 类型=1；字节3 序号；字节4 标志位
  *   字节5-6  球位置 int16 小端 0.1mm（画面左 = 正）
  *   字节7-8  球速度 int16 小端 mm/s（仅遥测参考）
  *   字节9    置信度 0~100；字节10-11 CRC16-CCITT(0x1021,0xFFFF) 低字节在前
  *   位置一帧跳变 >4cm 视为误检/重捕获，连续 3 帧确认才接受
  *
  * ASCII 帧（与二进制共线）：A,<a> 车加速度；R,<x> 球目标；M,<mdeg> 手动；
  * Z 水平清零。丢球行为：保持最后指令 600ms → 限速回平（任务保持运行，
  * 球回视野自动继续）；W1 丢球减速 / W0 重捕获 / W2 回平停车告警。
  *
  * 本文件属于用户代码，CubeMX 重新生成工程时不会被覆盖。
  ******************************************************************************
  */

#ifndef __BALL_CTRL_H__
#define __BALL_CTRL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ==========================================================================
 *                    配置区（上车前按实际机械改这里）
 * ========================================================================== */

/* --- 角度正方向约定：+角度 = 摆杆 +x 端（画面左端）抬高 ------------------- */

/* 步进正方向是否使摆杆 +x 端抬高。台架验证：启动后发 M,2000（抬 2 度），
 * 画面左端应抬高；抬的是右端就把它改成 -1.0f。 */
#define ROD_DIR_SIGN        (+1.0f)

/* 摆杆半长（mm）：限制球目标位置范围，防止指令把球赶到杆外。 */
#define ROD_HALF_LEN_MM     (120.0f)

/* --- 参考设计控制参数（移植自 MSPM0 ball_balance，滑模 PD） ---------------- */

/* PD 增益：u = +KP·e_pred + KD·v（mdeg；无积分，稳态偏差由回零偏学习消除）。
 * e = 球相对目标偏差。注意：参考工程机械上 +角压低 +x 端（其律为 -KP·x）；
 * 本工程台架实测 +角抬高 +x 端（M,2000 测试），移植时 P/D 已翻转符号。
 * 整定：回中慢加 KP；过冲/振荡加 KD 或减 KP。 */
#define BALL_PD_KP_MDEG_PER_MM   (220)    /* 0.22°/mm */
#define BALL_PD_KD_MDEG_PER_MM_S (50)     /* 0.05°/(mm/s) */

/* 角度限幅：远端松弛加速回正，近端收紧防过冲（"先大幅回正再微调"） */
#define BALL_CONTROL_MAX_MDEG    (10000)  /* 常规限幅 ±10° */
#define BALL_RETURN_MAX_MDEG     (14000)  /* 远端限幅 ±14° */
#define BALL_RETURN_DIST_01MM    (400)    /* 远端判定：|x| > 40mm */

/* 预测与死区 */
#define BALL_LOOKAHEAD_MS        (120)    /* 位置前推时间（提前刹车） */
#define BALL_CENTER_DEADBAND_01MM (5)     /* 中心死区 0.5mm：稳定球不打针 */

/* 滤波与测速 */
#define BALL_POS_FILTER_DIV      (2)      /* 位置 EMA 除数 */
#define BALL_V_DIFF_FRAMES       (2U)     /* 差分测速帧窗 */
#define BALL_V_DIFF_FILTER       (2U)     /* 测速 EMA 除数 */
#define BALL_VISION_FRAME_MS     (20U)    /* 视觉标称帧周期 */

/* 静摩擦脱困脉冲：球准静止(≥1.5mm 且 |v|≤8mm/s)持续 200ms 后，
 * 发"斜坡-保持-卸力"脉冲（300/250/100ms），幅度逐脉冲递增，球一动即收 */
#define BALL_STICTION_POS_01MM   (15)
#define BALL_STICTION_VEL_MM_S   (8)
#define BALL_STICTION_HOLD_MS    (200U)
#define BALL_STICK_TRANSIT_POS_01MM (100) /* ≥10mm 算远端行程 */
#define BALL_STICK_TRANSIT_VEL   (18)
#define BALL_STICK_NEAR_BOOST    (4500)   /* 近端首脉冲 4.5° */
#define BALL_STICK_NEAR_STEP     (250)    /* 每脉冲 +0.25° */
#define BALL_STICK_NEAR_MAX      (5000)   /* 近端上限 5° */
#define BALL_STICK_TRA_BOOST     (3000)   /* 远端首脉冲 3° */
#define BALL_STICK_TRA_STEP      (1000)   /* 每脉冲 +1° */
#define BALL_STICK_TRA_MAX       (9000)   /* 远端上限 9° */
#define BALL_STICK_RAMP_MS       (300U)
#define BALL_STICK_HOLD_MS       (250U)
#define BALL_STICK_RELEASE_MS    (100U)
#define BALL_STICK_MEMORY_PCT    (90)     /* 记住上次脱困幅度 */

/* 回零偏学习：机械零不水平的常驻小倾角（替代积分项）。
 * 球准静止且偏离中心 1.5~60mm 持续 150ms 后慢速积累；±3° 限幅；
 * 球换边保留 70%。 */
#define BALL_BIAS_MIN_POS_01MM   (15)
#define BALL_BIAS_MAX_POS_01MM   (600)
#define BALL_BIAS_MAX_VEL_MM_S   (15)
#define BALL_BIAS_CONFIRM_MS     (150U)
#define BALL_BIAS_KI             (120)    /* mdeg/s 每 0.1mm 偏差 */
#define BALL_BIAS_MAX_RATE       (400)    /* mdeg/s */
#define BALL_BIAS_LIMIT_MDEG     (3000)
#define BALL_BIAS_FLIP_POS_01MM  (25)
#define BALL_BIAS_FLIP_RETAIN    (70)     /* %（球换边时偏置保留比例） */

/* 输出整形 */
#define BALL_ENABLE_SETTLE_MS    (400U)   /* 启动稳定窗：滤波建立前不出力 */
#define BALL_TARGET_DEADBAND     (120)    /* 微调死区 0.12°：稳定球不追针 */
#define BALL_DIR_SWITCH_01MM     (30)     /* 方向约束换边阈值 3mm */
#define BALL_BRAKE_MIN_VEL_MM_S  (8)      /* 刹车例外最低球速 */
#define BALL_SLEW_CONTROL        (60000U) /* 常规斜率 60°/s */
#define BALL_SLEW_BRAKE          (120000U)/* 刹车斜率 120°/s */
#define BALL_SLEW_RETURN         (12000U) /* 丢球回平斜率 12°/s */

/* 视觉链路与丢球 */
#define BALL_CONF_MIN            (35)     /* 置信度下限（参考设计 35） */
#define BALL_VISION_TIMEOUT_MS   (150U)   /* 帧龄超过此值 = 链路不新鲜 */
#define BALL_DETECTION_HOLD_MS   (600U)   /* 丢球保持期：沿用最后指令 */
#define BALL_JUMP_01MM           (400)    /* 一帧位置跳变 >4cm 可疑 */
#define BALL_JUMP_CONFIRM        (3U)     /* 连续 3 帧确认才接受 */

/* 惯性前馈：车主控 A 帧加速度（mm/s²）→ mdeg 线性映射（180mm/s² 每 1°）。
 * 物理上车加速(+a 与杆 +x 同向)时球受向 -x 的惯性力，杆应放低 +x 端补偿
 * （本工程 +角抬高 +x 端，故默认 -1；参考工程机械方向相反所以是 +1）。 */
#define BALL_INERTIA_DIRECTION   (-1)
#define BALL_INERTIA_MM_S2_PER_DEG (180)
#define BALL_INERTIA_MAX_MDEG    (5000)   /* 前馈上限 ±5° */
#define BALL_ACCEL_LIMIT_MM_S2   (1500)   /* A 帧加速度输入限幅 */

/* ==========================================================================
 *                                 接口
 * ========================================================================== */

void    ball_ctrl_init(void);        /* 初始化（含挂起 USART1 接收） */
void    ball_ctrl_start(void);       /* 启动要求4稳球（记录当前电机位置=水平零位） */
void    ball_ctrl_stop_level(void);  /* 停止稳球，摆杆回到水平零位 */
uint8_t ball_ctrl_is_running(void);
void    ball_ctrl_relevel(void);     /* 重新定义水平零位 = 当前电机位置 */

void    ball_ctrl_tick_10ms(void);   /* 每 10ms 调用一次（100Hz 控制律） */

/* --- UART 帧（内部已带环形缓冲，中断往里塞字节，主循环 poll 解析） --- */
void    ball_ctrl_uart_start_rx(void);       /* 挂起 1 字节中断接收 */
void    ball_ctrl_uart_rx_isr(uint8_t b);    /* RX 回调里调用：字节入环形缓冲 */
uint8_t ball_ctrl_uart_rx_byte(void);        /* RX 回调里取刚收到的字节 */
void    ball_ctrl_uart_poll(void);           /* 主循环调用：解析收到的帧 */

/* --- MaixCam 二进制帧入口（解析器内部自动调用，外部一般不用） ---
 * flags: bit0=检出球 bit1=摆杆已标定 bit2=跟踪有效
 * 未检出帧（bit0=0）不刷新丢球计时，位置/速度保持上次值供遥测 */
void    ball_ctrl_feed_vision(int32_t x01mm, int32_t v_mms, uint8_t conf,
                              uint8_t flags);

/* --- 遥测（串口打印 / 调试用） --- */
int32_t ball_ctrl_ball_01mm(void);   /* 最新球偏移，0.1mm */
int32_t ball_ctrl_vel_mms(void);     /* 差分测速结果，mm/s */
uint8_t ball_ctrl_conf(void);        /* 最新置信度 0~100，255=视觉未提供 */
int32_t ball_ctrl_xref_01mm(void);   /* 球目标位置，0.1mm */
int32_t ball_ctrl_angle_mdeg(void);  /* 当前摆杆指令角（限速后），mdeg */
int32_t ball_ctrl_fb_mdeg(void);     /* 反馈项输出（PD+偏置+脱困），mdeg */
int32_t ball_ctrl_ff_mdeg(void);     /* 惯性前馈输出，mdeg */
int32_t ball_ctrl_accel(void);       /* 最新车加速度，mm/s² */
uint8_t ball_ctrl_ball_ok(void);     /* 1 = 视觉数据有效且置信度达标 */
uint8_t ball_ctrl_sat(void);         /* 1 = PD 输出被角度限幅 */
uint8_t ball_ctrl_fault(void);       /* 保留接口：参考设计不自动退出，恒 0 */
uint8_t ball_ctrl_vision_cal(void);  /* 1 = MaixCam 摆杆标定完成（帧标志bit1） */
uint32_t ball_ctrl_frame_count(void);/* 已收到的合法视觉帧数（通信健康度） */
uint32_t ball_ctrl_crc_err(void);    /* 校验失败帧数（长期增长=线路受干扰） */

#ifdef __cplusplus
}
#endif

#endif /* __BALL_CTRL_H__ */
