#ifndef DRIVERS_MOTOR_H
#define DRIVERS_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOTOR_STATE_DISABLED = 0,
    MOTOR_STATE_READY,
    MOTOR_STATE_FAULT
} motor_state_t;

void motor_init(void);
bool motor_enable(void);
void motor_disable(void);
bool motor_set(int16_t left, int16_t right);
void motor_brake(void);
motor_state_t motor_state(void);
int16_t motor_left_output(void);
int16_t motor_right_output(void);

#endif
