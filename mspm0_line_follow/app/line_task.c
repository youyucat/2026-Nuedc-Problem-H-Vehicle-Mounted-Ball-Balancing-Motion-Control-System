#include "app/line_task.h"

#include <stddef.h>

#include "hal/hal.h"
#include "config/template_config.h"
#include "control/line_controller.h"
#include "control/line_estimator.h"
#include "drivers/encoder_speed.h"
#include "drivers/line_sensor.h"
#include "drivers/motor.h"
#include "services/line_processing.h"

/*
 * 循迹任务的实现，逻辑逐条来自源工程 empty_LP_MSPM0G3507_nortos_ticlang 的
 * app/template_app.c（running_step / recover_lost_line / speed_pi_step 等），
 * 只做了"剪掉另一颗芯片的职责"这一件事：
 *   - 删掉 calibration_*（灰度标定界面）、parameter_store_*（Flash 参数区）、
 *     imu_service_* / stepper_* / oled_* / pc_protocol_* 的所有调用；
 *   - 删掉 MOTOR_TEST 状态与 template_app_* 的上位机命令入口；
 *   - 起步条件从"上位机 run_start（要求已标定 + 有有效线）"改成
 *     "上电 TEMPLATE_STARTUP_HOLD_MS 后自动 start()"。
 * 控制律、限幅、丢线恢复、软起步、堵转门限一个都没改，参数仍在
 * config/template_config.h。
 */

typedef struct {
    int32_t last_error;
    int32_t output;
} speed_pi_state_t;

static line_task_snapshot_t s_snapshot;
static line_frame_t s_line_frame;
static line_controller_t s_controller;
static uint32_t s_boot_ms;
static uint32_t s_last_control_ms;
static uint32_t s_line_lost_since_ms;
static uint32_t s_line_stall_since_ms;
static bool s_line_lost_timing;
static bool s_line_stall_timing;
static bool s_had_valid_line;
static int16_t s_line_error_filtered;
static bool s_line_error_init;
static int16_t s_last_valid_left_target;
static int16_t s_last_valid_right_target;
static int16_t s_ramp_left;
static int16_t s_ramp_right;
static int8_t s_last_turn_sign;
static speed_pi_state_t s_left_speed_pi;
static speed_pi_state_t s_right_speed_pi;
static int16_t s_left_speed_output;
static int16_t s_right_speed_output;
static uint16_t s_left_motor_trim_permille =
    TEMPLATE_DEFAULT_LEFT_MOTOR_TRIM_PERMILLE;
static uint16_t s_right_motor_trim_permille =
    TEMPLATE_DEFAULT_RIGHT_MOTOR_TRIM_PERMILLE;
static uint32_t s_last_speed_sequence;
static bool s_auto_start = true;

static void clear_motion(void)
{
    s_snapshot.left_target = 0;
    s_snapshot.right_target = 0;
    s_snapshot.left_output = 0;
    s_snapshot.right_output = 0;
    s_snapshot.left_speed_mm_s = 0;
    s_snapshot.right_speed_mm_s = 0;
    line_controller_reset(&s_controller);
    s_left_speed_pi = (speed_pi_state_t){0};
    s_right_speed_pi = (speed_pi_state_t){0};
    s_left_speed_output = 0;
    s_right_speed_output = 0;
    s_last_speed_sequence = 0U;
    s_line_lost_since_ms = 0U;
    s_line_lost_timing = false;
    s_line_stall_since_ms = 0U;
    s_line_stall_timing = false;
    s_had_valid_line = false;
    s_line_error_init = false;
    s_line_error_filtered = 0;
    s_last_valid_left_target = 0;
    s_last_valid_right_target = 0;
    s_ramp_left = 0;
    s_ramp_right = 0;
    s_last_turn_sign = 0;
}

static int16_t clamp_line_target(int16_t target)
{
    if (target > TEMPLATE_LINE_MAX_SPEED_MM_S) {
        return TEMPLATE_LINE_MAX_SPEED_MM_S;
    }
    if (target < TEMPLATE_LINE_MIN_SPEED_MM_S) {
        return TEMPLATE_LINE_MIN_SPEED_MM_S;
    }
    return target;
}

static int16_t apply_motor_trim(int16_t output, uint16_t trim_permille)
{
    int32_t trimmed = ((int32_t)output * trim_permille) / 1000;

    if (trimmed > 1000) return 1000;
    if (trimmed < -1000) return -1000;
    return (int16_t)trimmed;
}

static int16_t ramp_toward(int16_t current, int16_t target, int16_t step)
{
    if (current < target) {
        current = (int16_t)(current + step);
        if (current > target) current = target;
    } else if (current > target) {
        current = (int16_t)(current - step);
        if (current < target) current = target;
    }
    return current;
}

static void limit_line_correction(int16_t *left_target,
                                  int16_t *right_target)
{
    int32_t correction;
    int32_t correction_abs;
    int32_t base_effective;

    if ((left_target == NULL) || (right_target == NULL)) return;
    correction = ((int32_t)*left_target - *right_target) / 2;
    if (correction > TEMPLATE_LINE_MAX_CORRECTION_MM_S)
        correction = TEMPLATE_LINE_MAX_CORRECTION_MM_S;
    if (correction < -TEMPLATE_LINE_MAX_CORRECTION_MM_S)
        correction = -TEMPLATE_LINE_MAX_CORRECTION_MM_S;
    correction_abs = (correction >= 0) ? correction : -correction;
    /* 转弯自适应减速：打角越大基础速度越低，横向加速度小，另一颗芯片
     * 上的小球不会被甩出去（WHEELTEC 转弯自动减速）。 */
    base_effective = (int32_t)s_controller.base_output -
        (correction_abs * TEMPLATE_LINE_TURN_DECEL_FRACTION) / 100;
    if (base_effective < TEMPLATE_LINE_TURN_MIN_RUN_MM_S)
        base_effective = TEMPLATE_LINE_TURN_MIN_RUN_MM_S;
    *left_target = clamp_line_target(
        (int16_t)(base_effective + correction));
    *right_target = clamp_line_target(
        (int16_t)(base_effective - correction));
}

static int16_t speed_pi_step(speed_pi_state_t *state,
                             int16_t target_mm_s,
                             int16_t measured_mm_s)
{
    int32_t error;
    int32_t delta;
    int32_t minimum_output;

    if (state == NULL) return 0;
    if (target_mm_s == 0) {
        *state = (speed_pi_state_t){0};
        return 0;
    }

    error = (int32_t)target_mm_s - measured_mm_s;
    delta = ((int32_t)TEMPLATE_SPEED_PI_KP_MILLI *
             (error - state->last_error) +
             (int32_t)TEMPLATE_SPEED_PI_KI_MILLI * error) / 1000;

    /* 保持源工程的手感：PI 更新前先给到最小起转占空比，但允许本次更新把
     * 占空比压到该值以下（实测轮速已经过高时）。 */
    minimum_output = (target_mm_s < 0) ?
                     -(int32_t)TEMPLATE_SPEED_PI_MIN_OUTPUT :
                     (int32_t)TEMPLATE_SPEED_PI_MIN_OUTPUT;
    if ((target_mm_s > 0) && (state->output < minimum_output)) {
        state->output = minimum_output;
    } else if ((target_mm_s < 0) && (state->output > minimum_output)) {
        state->output = minimum_output;
    }
    state->output += delta;
    state->last_error = error;

    if (state->output > 1000) state->output = 1000;
    if (state->output < -1000) state->output = -1000;
    return (int16_t)state->output;
}

static void recover_lost_line(uint32_t now_ms,
                              int16_t *left_target,
                              int16_t *right_target)
{
    uint32_t lost_ms;

    if ((left_target == NULL) || (right_target == NULL)) return;
    *left_target = 0;
    *right_target = 0;
    if (!s_had_valid_line) return;

    if (!s_line_lost_timing) {
        s_line_lost_timing = true;
        s_line_lost_since_ms = now_ms;
    }
    lost_ms = now_ms - s_line_lost_since_ms;
    if (lost_ms <= TEMPLATE_LINE_LOST_HOLD_MS) {
        *left_target = s_last_valid_left_target;
        *right_target = s_last_valid_right_target;
        return;
    }
    if (lost_ms >= TEMPLATE_LINE_LOST_STOP_MS) return;

    /* 摇头扫描寻线：沿左右方向交替转头（每 SWEEP_MS 换向），扫描幅度从
     * 「上次修正 ≥ 下限」随失线时长线性增至上限，直到重新捕线或超时停车。
     * 首段沿用最后有效转弯方向，直线丢线时也有个默认方向。 */
    {
        uint32_t search_ms = lost_ms - TEMPLATE_LINE_LOST_HOLD_MS;
        uint32_t sweep_phase = search_ms / TEMPLATE_LINE_LOST_SWEEP_MS;
        int32_t last_correction =
            ((int32_t)s_last_valid_left_target -
             s_last_valid_right_target) / 2;
        int32_t correction;
        int32_t base_recover;
        uint8_t sweep_direction;

        if (last_correction < 0) last_correction = -last_correction;
        if (last_correction < TEMPLATE_LINE_RECOVER_MIN_CORRECTION) {
            last_correction = TEMPLATE_LINE_RECOVER_MIN_CORRECTION;
        }
        correction = last_correction +
            ((TEMPLATE_LINE_RECOVER_MAX_CORRECTION - last_correction) *
             (int32_t)search_ms) /
            ((int32_t)TEMPLATE_LINE_LOST_STOP_MS -
             (int32_t)TEMPLATE_LINE_LOST_HOLD_MS);
        if (correction > TEMPLATE_LINE_RECOVER_MAX_CORRECTION) {
            correction = TEMPLATE_LINE_RECOVER_MAX_CORRECTION;
        }
        sweep_direction = ((sweep_phase & 1U) == 0U) ? 1U : 0U;
        if ((sweep_phase == 0U) && (s_last_turn_sign < 0)) {
            sweep_direction = 0U;
        }
        base_recover = ((int32_t)s_controller.base_output * 70) / 100;
        if (sweep_direction != 0U) {
            *left_target = clamp_line_target(
                (int16_t)(base_recover + correction));
            *right_target = clamp_line_target(
                (int16_t)(base_recover - correction));
        } else {
            *left_target = clamp_line_target(
                (int16_t)(base_recover - correction));
            *right_target = clamp_line_target(
                (int16_t)(base_recover + correction));
        }
    }
}

static bool line_wheel_stalled(uint32_t now_ms,
                               int16_t left_target,
                               int16_t right_target,
                               int16_t left_output,
                               int16_t right_output,
                               const encoder_speed_snapshot_t *speed)
{
    bool left_stall;
    bool right_stall;

    if (speed == NULL) return false;
    left_stall = (left_target >= 250) && (left_output >= 900) &&
                 (speed->left_speed_mm_s < 20) &&
                 (speed->right_speed_mm_s > 150);
    right_stall = (right_target >= 250) && (right_output >= 900) &&
                  (speed->right_speed_mm_s < 20) &&
                  (speed->left_speed_mm_s > 150);
    if (!left_stall && !right_stall) {
        s_line_stall_timing = false;
        return false;
    }
    if (!s_line_stall_timing) {
        s_line_stall_timing = true;
        s_line_stall_since_ms = now_ms;
        return false;
    }
    return (uint32_t)(now_ms - s_line_stall_since_ms) >=
           TEMPLATE_LINE_STALL_DETECT_MS;
}

static void running_step(uint32_t now_ms)
{
    encoder_speed_snapshot_t speed;
    int16_t left_target;
    int16_t right_target;
    int16_t left_output;
    int16_t right_output;

    encoder_speed_get(&speed);
    left_target = 0;
    right_target = 0;
    if (s_snapshot.line_valid) {
        int16_t line_error_in;

        if (s_line_lost_timing) line_controller_reset(&s_controller);
        s_line_lost_timing = false;
        /* 直线蛇形抑制：线误差一阶低通（EMA）+ 死区。8 路灰度在离散位置
         * 间跳动时 D 项会放大噪声，平滑后直线误差趋近 0，弯道仍快速响应。 */
        if (!s_line_error_init) {
            s_line_error_init = true;
            s_line_error_filtered = s_snapshot.line_error;
        } else {
            s_line_error_filtered = (int16_t)(
                (s_line_error_filtered *
                     (100 - TEMPLATE_LINE_ERROR_EMA_NUM) +
                 s_snapshot.line_error * TEMPLATE_LINE_ERROR_EMA_NUM) / 100);
        }
        line_error_in = s_line_error_filtered;
        if ((line_error_in > -TEMPLATE_LINE_ERROR_DEADZONE) &&
            (line_error_in < TEMPLATE_LINE_ERROR_DEADZONE)) {
            line_error_in = 0;
        }
        line_controller_step(&s_controller, line_error_in,
                             &left_target, &right_target);
        limit_line_correction(&left_target, &right_target);
        s_last_valid_left_target = left_target;
        s_last_valid_right_target = right_target;
        if (s_snapshot.line_error > 20) s_last_turn_sign = 1;
        else if (s_snapshot.line_error < -20) s_last_turn_sign = -1;
        s_had_valid_line = true;
    } else {
        recover_lost_line(now_ms, &left_target, &right_target);
    }
    /* 软起步：目标速度从 0 缓慢爬到目标（每 5 ms 拍 1 mm/s），停车是立即的
     * 并复位斜坡，下次起步重新缓起 —— 另一颗芯片上的小球不挨冲击。 */
    if ((left_target == 0) && (right_target == 0)) {
        s_ramp_left = 0;
        s_ramp_right = 0;
    } else {
        s_ramp_left = ramp_toward(
            s_ramp_left, left_target,
            TEMPLATE_LINE_RAMP_STEP_MM_S_PER_TICK);
        s_ramp_right = ramp_toward(
            s_ramp_right, right_target,
            TEMPLATE_LINE_RAMP_STEP_MM_S_PER_TICK);
    }
    left_target = s_ramp_left;
    right_target = s_ramp_right;
    s_snapshot.left_target = left_target;
    s_snapshot.right_target = right_target;
    left_output = s_left_speed_output;
    right_output = s_right_speed_output;
    if (speed.sequence != s_last_speed_sequence) {
        left_output = apply_motor_trim(
            speed_pi_step(&s_left_speed_pi, left_target,
                          (int16_t)speed.left_speed_mm_s),
            s_left_motor_trim_permille);
        right_output = apply_motor_trim(
            speed_pi_step(&s_right_speed_pi, right_target,
                          (int16_t)speed.right_speed_mm_s),
            s_right_motor_trim_permille);
        s_left_speed_output = left_output;
        s_right_speed_output = right_output;
        s_last_speed_sequence = speed.sequence;
    }
    if ((left_target == 0) && (right_target == 0)) {
        motor_brake();
        s_snapshot.left_output = 0;
        s_snapshot.right_output = 0;
        return;
    }
    if (!motor_set(left_output, right_output)) {
        motor_disable();
        clear_motion();
        s_snapshot.state = LINE_TASK_STATE_FAULT;
        return;
    }
    if (line_wheel_stalled(now_ms, left_target, right_target,
                           left_output, right_output, &speed)) {
        motor_disable();
        clear_motion();
        s_snapshot.state = LINE_TASK_STATE_FAULT;
        return;
    }
    s_snapshot.left_output = motor_left_output();
    s_snapshot.right_output = motor_right_output();
}

void line_task_init(void)
{
    line_sensor_raw_frame_t warm = {0};

    hal_board_init();
    encoder_speed_init();
    line_sensor_init();
    /* 预热一帧（同步兜底路径），保证首个控制拍就有数据可用。 */
    (void)line_sensor_read(&warm);
    line_processing_init();
    motor_init();
    line_controller_init(&s_controller,
        TEMPLATE_DEFAULT_BASE_OUTPUT,
        TEMPLATE_DEFAULT_KP_MILLI,
        TEMPLATE_DEFAULT_KD_MILLI);

    s_snapshot = (line_task_snapshot_t){0};
    s_line_frame = (line_frame_t){0};
    s_snapshot.state = LINE_TASK_STATE_IDLE;
    clear_motion();

    s_boot_ms = hal_time_millis();
    s_last_control_ms = s_boot_ms;
}

void line_task_step(uint32_t now_ms)
{
    line_sensor_raw_frame_t raw = {0};
    encoder_speed_snapshot_t speed;

    encoder_speed_update(now_ms);
    encoder_speed_get(&speed);
    s_snapshot.left_speed_mm_s = speed.left_speed_mm_s;
    s_snapshot.right_speed_mm_s = speed.right_speed_mm_s;

    if ((uint32_t)(now_ms - s_last_control_ms) <
        TEMPLATE_CONTROL_PERIOD_MS) {
        return;
    }
    s_last_control_ms = now_ms;

    /* 异步帧：发起后台采样并取走上一帧；新帧未就绪时按"无效帧"处理
     * （line_valid 为 false，自然走丢线恢复），控制拍零阻塞。 */
    if (!line_sensor_async_step(&raw)) {
        raw.channel_count = TEMPLATE_SENSOR_CHANNELS;
        raw.sensor_error = true;
    }
    line_processing_update(&raw, &s_line_frame);
    ++s_snapshot.sequence;
    s_snapshot.active_mask = s_line_frame.active_mask;
    s_snapshot.sensor_error = s_line_frame.sensor_error;
    s_snapshot.line_valid = line_estimator_compute(
        &s_line_frame, hal_gray_positions(), &s_snapshot.line_error);

    if (s_snapshot.state == LINE_TASK_STATE_IDLE) {
        /* 上电等待窗口结束后自动起步；不在线上时 running_step 会保持
         * 零目标（电机不转），直到看到线才真正走。
         * s_auto_start 由串口 stop 命令关闭 —— 手动停不该被自动起步覆盖。 */
        if (s_auto_start &&
            ((uint32_t)(now_ms - s_boot_ms) >= TEMPLATE_STARTUP_HOLD_MS)) {
            (void)line_task_start();
        }
        return;
    }
    if (s_snapshot.state == LINE_TASK_STATE_RUNNING) {
        running_step(now_ms);
    }
}

void line_task_get_snapshot(line_task_snapshot_t *snapshot)
{
    if (snapshot != NULL) *snapshot = s_snapshot;
}

void line_task_set_auto_start(bool enabled)
{
    s_auto_start = enabled;
}

void line_task_get_frame(line_frame_t *frame)
{
    if (frame != NULL) *frame = s_line_frame;
}

bool line_task_start(void)
{
    if (s_snapshot.state == LINE_TASK_STATE_RUNNING) return true;
    if (!motor_enable()) return false;
    clear_motion();
    s_snapshot.state = LINE_TASK_STATE_RUNNING;
    return true;
}

void line_task_stop(void)
{
    motor_disable();
    clear_motion();
    s_snapshot.state = LINE_TASK_STATE_IDLE;
    s_boot_ms = hal_time_millis(); /* 重新计时，避免立刻又被自动起步 */
}

void line_task_reset_fault(void)
{
    if (s_snapshot.state == LINE_TASK_STATE_FAULT) {
        motor_disable();
        clear_motion();
        s_snapshot.state = LINE_TASK_STATE_IDLE;
        s_boot_ms = hal_time_millis();
    }
}

bool line_task_set_gains(int16_t base_output, int16_t kp_milli,
                         int16_t kd_milli)
{
    if ((s_snapshot.state != LINE_TASK_STATE_IDLE) ||
        (base_output < 0) ||
        (base_output > TEMPLATE_CONTROLLER_BASE_MAX) ||
        (kp_milli < 0) || (kp_milli > TEMPLATE_CONTROLLER_KP_MAX) ||
        (kd_milli < 0) || (kd_milli > TEMPLATE_CONTROLLER_KD_MAX)) {
        return false;
    }
    line_controller_init(&s_controller, base_output, kp_milli, kd_milli);
    return true;
}

void line_task_get_gains(int16_t *base_output, int16_t *kp_milli,
                         int16_t *kd_milli)
{
    if (base_output != NULL) *base_output = s_controller.base_output;
    if (kp_milli != NULL) *kp_milli = s_controller.kp_milli;
    if (kd_milli != NULL) *kd_milli = s_controller.kd_milli;
}

bool line_task_set_motor_trim(uint16_t left_permille,
                              uint16_t right_permille)
{
    if ((s_snapshot.state != LINE_TASK_STATE_IDLE) ||
        ((uint16_t)(left_permille - 850U) > 300U) ||
        ((uint16_t)(right_permille - 850U) > 300U)) {
        return false;
    }
    s_left_motor_trim_permille = left_permille;
    s_right_motor_trim_permille = right_permille;
    return true;
}
