#ifndef CONTROL_LINE_CONTROLLER_H
#define CONTROL_LINE_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t base_output;
    int16_t kp_milli;
    int16_t kd_milli;
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
                          int16_t *left,
                          int16_t *right);

#endif
