#include "drivers/motor.h"

#include "hal/hal.h"
#include "config/template_config.h"

static motor_state_t s_state;
static int16_t s_left;
static int16_t s_right;

static int16_t clamp_output(int16_t value)
{
    if (value > 1000) return 1000;
    if (value < -1000) return -1000;
    return value;
}

static uint16_t magnitude(int16_t value)
{
    return (uint16_t)((value < 0) ? -value : value);
}

static void set_one(bool logical_left, int16_t output, bool inverted)
{
    bool forward;
    bool physical_left = logical_left;

    forward = output >= 0;
    if (inverted) forward = !forward;
#if TEMPLATE_MOTOR_OUTPUTS_SWAPPED
    physical_left = !physical_left;
#endif
    if (physical_left) {
        hal_motor_set_left_direction(forward);
    } else {
        hal_motor_set_right_direction(forward);
    }
}

void motor_init(void)
{
    s_left = 0;
    s_right = 0;
    s_state = MOTOR_STATE_DISABLED;
    hal_motor_set_pwm(0U, 0U);
    hal_motor_safe_inputs();
    hal_motor_standby(false);
}

bool motor_enable(void)
{
    if (s_state == MOTOR_STATE_FAULT) return false;
    hal_motor_set_pwm(0U, 0U);
    hal_motor_safe_inputs();
    hal_motor_standby(true);
    s_left = 0;
    s_right = 0;
    s_state = MOTOR_STATE_READY;
    return true;
}

void motor_disable(void)
{
    hal_motor_set_pwm(0U, 0U);
    hal_motor_safe_inputs();
    hal_motor_standby(false);
    s_left = 0;
    s_right = 0;
    if (s_state != MOTOR_STATE_FAULT) s_state = MOTOR_STATE_DISABLED;
}

bool motor_set(int16_t left, int16_t right)
{
    if (s_state != MOTOR_STATE_READY) return false;
    left = clamp_output(left);
    right = clamp_output(right);
    if (((s_left < 0) && (left > 0)) || ((s_left > 0) && (left < 0)) ||
        ((s_right < 0) && (right > 0)) ||
        ((s_right > 0) && (right < 0))) {
        hal_motor_set_pwm(0U, 0U);
    }
    set_one(true, left, TEMPLATE_LEFT_MOTOR_INVERTED != 0U);
    set_one(false, right, TEMPLATE_RIGHT_MOTOR_INVERTED != 0U);
#if TEMPLATE_MOTOR_OUTPUTS_SWAPPED
    hal_motor_set_pwm(magnitude(right), magnitude(left));
#else
    hal_motor_set_pwm(magnitude(left), magnitude(right));
#endif
    s_left = left;
    s_right = right;
    return true;
}

void motor_brake(void)
{
    hal_motor_set_pwm(0U, 0U);
    hal_motor_safe_inputs();
    s_left = 0;
    s_right = 0;
}

motor_state_t motor_state(void)
{
    return s_state;
}

int16_t motor_left_output(void)
{
    return s_left;
}

int16_t motor_right_output(void)
{
    return s_right;
}
