#ifndef APP_LINE_TASK_H
#define APP_LINE_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "services/line_processing.h"   /* line_frame_t（get_frame 用） */

/*
 * 循迹任务（双芯架构中的「循迹芯片」主任务）。
 *
 * 一条 5 ms 控制拍里做完整条链：
 *   灰度 8 路异步帧 -> 归一化/判黑 -> 加权重心误差 -> EMA + 死区
 *   -> PD（线控制器）-> 差速限幅 + 弯道减速
 *   -> 左右目标轮速 -> 软起步斜坡 -> 轮速增量 PI -> TB6612 占空比
 * 丢线时：保持 80 ms -> 左右摇头扫描找线 -> 1500 ms 仍找不到就停车。
 *
 * 与源工程 app/template_app.c 的差异：去掉了上位机协议直控、Flash 参数
 * 存储、灰度标定界面、IMU/OLED/步进/球平衡；起步改成"上电自检后自动进入
 * RUNNING"（TEMPLATE_STARTUP_HOLD_MS）。
 */

typedef enum {
    LINE_TASK_STATE_IDLE = 0,  /* 上电等待 / 已停机（电机零输出） */
    LINE_TASK_STATE_RUNNING,   /* 循迹中 */
    LINE_TASK_STATE_FAULT      /* 堵转或电机异常，需 line_task_reset_fault() */
} line_task_state_t;

typedef struct {
    line_task_state_t state;
    uint32_t sequence;         /* 控制拍计数（每 5 ms 一拍） */
    bool line_valid;           /* 本拍是否算出了有效线位置 */
    int16_t line_error;        /* -1000..+1000，右偏为正，满量程=最外侧探头 */
    uint32_t active_mask;      /* 本帧判为黑线的通道位图（bit0 = CH1） */
    bool sensor_error;         /* 传感板 ERR 脚有效 */
    int16_t left_target;       /* 目标轮速 mm/s */
    int16_t right_target;
    int16_t left_output;       /* 实际输出占空比 -1000..+1000 */
    int16_t right_output;
    int32_t left_speed_mm_s;   /* 实测轮速 mm/s */
    int32_t right_speed_mm_s;
} line_task_snapshot_t;

/* 板级初始化 + 灰度预热一帧 + 电机待机，然后处于 IDLE。 */
void line_task_init(void);

/* 主循环里每圈调用一次（可高频调用，内部按 5 ms 限拍）。 */
void line_task_step(uint32_t now_ms);

/* 复制一份状态（调试器观察 / 上层显示用）。 */
void line_task_get_snapshot(line_task_snapshot_t *snapshot);

/* 手动起步 / 停车 / 清故障。上电 IDLE 满 TEMPLATE_STARTUP_HOLD_MS 后
 * step() 会自动调用一次 line_task_start()。 */
bool line_task_start(void);
void line_task_stop(void);
void line_task_reset_fault(void);

/* 自动起步开关（默认开，保持"上电自检后自动循迹"的行为）。
 * 串口调试的 stop 命令会关掉它 —— 否则刚停下 TEMPLATE_STARTUP_HOLD_MS
 * 就又被自动起步拉起来，手动停不下来。start 命令会重新打开。 */
void line_task_set_auto_start(bool enabled);

/* 取最近一帧灰度处理结果（原始值/归一化/阈值/判黑位图），
 * 供串口调试的 raw 视图与校准采样使用。 */
void line_task_get_frame(line_frame_t *frame);

/* 运行时调参（不做 Flash 持久化）：base 为基础速度 mm/s，
 * kp/kd 以千分之一为单位。越界返回 false 且不生效。 */
bool line_task_set_gains(int16_t base_output, int16_t kp_milli,
                         int16_t kd_milli);
void line_task_get_gains(int16_t *base_output, int16_t *kp_milli,
                         int16_t *kd_milli);

/* 左右电机出力配平（千分比，850..1150）。 */
bool line_task_set_motor_trim(uint16_t left_permille, uint16_t right_permille);

#endif
