#include "control/line_controller.h"

#include <stdbool.h>
#include <stddef.h>

static int16_t clamp_output(int32_t value)
{
    if (value > 1000) return 1000;
    if (value < -1000) return -1000;
    return (int16_t)value;
}

void line_controller_init(line_controller_t *controller,
                          int16_t base_output,
                          int16_t kp_milli,
                          int16_t kd_milli)
{
    if (controller == NULL) return;
    controller->base_output = base_output;
    controller->kp_milli = kp_milli;
    controller->kd_milli = kd_milli;
    controller->kdw_milli = LINE_CONTROLLER_KDW_MILLI_DEFAULT;
    controller->ki_milli = LINE_CONTROLLER_KI_MILLI_DEFAULT;
    controller->p_boost_permille =
        LINE_CONTROLLER_P_BOOST_PERMILLE_DEFAULT;
    controller->previous_error = 0;
    controller->has_previous = false;
}

void line_controller_reset(line_controller_t *controller)
{
    if (controller == NULL) return;
    controller->previous_error = 0;
    controller->has_previous = false;
    controller->i_acc = 0;
}

void line_controller_step(line_controller_t *controller,
                          int16_t error,
                          int16_t wheel_diff_mms,
                          int16_t *left,
                          int16_t *right)
{
    int32_t derivative = 0;
    int32_t correction;
    int32_t abs_error;
    int32_t gain;
    int32_t kdw_eff;

    if ((controller == NULL) || (left == NULL) || (right == NULL)) return;
    if (controller->has_previous) {
        derivative = (int32_t)error - controller->previous_error;
    }
    /* 阻尼项来自编码器轮速差而不是误差差分（2026-11-07 仿真对照）：
     * error 是 8 个离散探头的加权质心，EMA+量化后拍差分 D 信号稀烂——
     * kd 从 130 扫到 400 四个场景曲线纹丝不动。轮速差 (v_r - v_l) 由
     * 编码器测得，干净且直接正比于车体角速度，是真正的航向阻尼。
     * 符号：车向右转时 v_r>v_l，diff>0，+kd_w·diff 产生反向修正，
     * 抑制转弯过冲——这正是 D 项该干没干成的事。 */
    abs_error = (error >= 0) ? error : -error;
    gain = (int32_t)controller->kp_milli +
           ((int32_t)controller->kp_milli *
            (int32_t)controller->p_boost_permille / 1000) *
               abs_error / 1000;
    /* 轮速差阻尼随 |e| 退让（2026-11-07 实车）：弯道稳态需要持续差速，
     * 固定 kdw 会把转弯量当成「转角过快」持续反推 —— 实测表现为
     * 「转弯一半角度不够脱线 / 直接冲出弯道」。所以直线（|e|≈0）保留
     * 全阻尼压蛇形，弯道（|e| 大）阻尼线性衰减到 35%。 */
    {
        int32_t rolloff = (abs_error * 1000) / 600;   /* e=600(≈21mm) 时全让路 */
        if (rolloff > 1000) rolloff = 1000;
        kdw_eff = (int32_t)controller->kdw_milli * (1000 - rolloff) / 1000;
    }
    correction = (gain * error +
                  (int32_t)controller->kd_milli * derivative) / 1000 +
                 ((kdw_eff * wheel_diff_mms) / 1000) +
                 ((int32_t)controller->ki_milli * controller->i_acc) /
                     1000000;
    *left = clamp_output((int32_t)controller->base_output + correction);
    *right = clamp_output((int32_t)controller->base_output - correction);
    controller->previous_error = error;
    controller->has_previous = true;
}

void line_controller_integrate(line_controller_t *controller, int16_t error)
{
    int32_t limit;

    if ((controller == NULL) || (controller->ki_milli <= 0)) return;
    limit = LINE_CONTROLLER_I_ACC_LIMIT;
    controller->i_acc += error;
    if (controller->i_acc >  limit) controller->i_acc =  limit;
    if (controller->i_acc < -limit) controller->i_acc = -limit;
}
