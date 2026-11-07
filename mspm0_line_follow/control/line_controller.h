#ifndef CONTROL_LINE_CONTROLLER_H
#define CONTROL_LINE_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

/* 非线性 P 的默认增益爬升系数（千分比）：|e|=1000 时 P 增益 ×(1+该值/1000)。
 * 2026-11-07 仿真对照：单独用它救弯道无效且直线恶化，默认 0 保留机制。
 * 弯道的真正解药是下面的轮速差阻尼。 */
#ifndef LINE_CONTROLLER_P_BOOST_PERMILLE_DEFAULT
#define LINE_CONTROLLER_P_BOOST_PERMILLE_DEFAULT 0
#endif

/* 轮速差阻尼增益（千分比）：correction += kdw/1000 × (v_r − v_l)。
 * 500 = 轮速差 60mm/s 时产生 30mm/s 的反向修正。
 * 误差差分 D 在离散探头下是死的（kd 130→400 仿真无变化），
 * 编码器轮速差才是干净的航向阻尼信号。 */
#ifndef LINE_CONTROLLER_KDW_MILLI_DEFAULT
#define LINE_CONTROLLER_KDW_MILLI_DEFAULT 500
#endif

/* 转向积分：仿真否决（ki 300/600/1000 弯道全线恶化——100ms 执行滞后下
 * I 建立过程中车已冲出，出弯残留反致过冲），默认 0，机制保留。
 * 长窗差分 D 同样被仿真否决（50ms 窗 arc500 82.7→137mm，滞后主导的
 * 系统里任何"超前量"都会吃掉稳定裕度）。入弯脱线的正解是弯道降速。 */
#ifndef LINE_CONTROLLER_KI_MILLI_DEFAULT
#define LINE_CONTROLLER_KI_MILLI_DEFAULT 0
#endif
#ifndef LINE_CONTROLLER_I_ACC_LIMIT
#define LINE_CONTROLLER_I_ACC_LIMIT 300000
#endif

typedef struct {
    int16_t base_output;
    int16_t kp_milli;
    int16_t kd_milli;
    int16_t p_boost_permille;   /* 非线性 P 增益爬升系数（千分比） */
    int16_t kdw_milli;          /* 轮速差阻尼增益（千分比） */
    int16_t ki_milli;           /* 转向积分增益（千分比） */
    int32_t i_acc;              /* 误差积分累加（拍单位） */
    int16_t previous_error;
    bool has_previous;
} line_controller_t;

void line_controller_init(line_controller_t *controller,
                          int16_t base_output,
                          int16_t kp_milli,
                          int16_t kd_milli);
void line_controller_reset(line_controller_t *controller);
void line_controller_step(line_controller_t *controller,
                          int16_t error,
                          int16_t wheel_diff_mms,
                          int16_t *left,
                          int16_t *right);
/* 转向积分累加（每控制拍调用一次，仅在线有效时累加） */
void line_controller_integrate(line_controller_t *controller, int16_t error);

#endif
