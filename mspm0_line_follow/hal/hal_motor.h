#ifndef HAL_HAL_MOTOR_H
#define HAL_HAL_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

/* TB6612 dual H-bridge interface. Output values are in permille (0..1000).
 * The STBY pin is hard-wired to +5 V on the reference board; the standby
 * call is kept for boards that drive it from the MCU. */

void hal_motor_standby(bool enabled);

/* Drive all direction inputs to the safe (motor-off) state. */
void hal_motor_safe_inputs(void);

void hal_motor_set_left_direction(bool forward);
void hal_motor_set_right_direction(bool forward);

/* Set both PWM outputs; each value is clamped to 0..1000 permille. */
void hal_motor_set_pwm(uint16_t left_permille, uint16_t right_permille);

#endif
