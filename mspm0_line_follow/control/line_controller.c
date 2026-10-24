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
    controller->previous_error = 0;
    controller->has_previous = false;
}

void line_controller_reset(line_controller_t *controller)
{
    if (controller == NULL) return;
    controller->previous_error = 0;
    controller->has_previous = false;
}

void line_controller_step(line_controller_t *controller,
                          int16_t error,
                          int16_t *left,
                          int16_t *right)
{
    int32_t derivative = 0;
    int32_t correction;

    if ((controller == NULL) || (left == NULL) || (right == NULL)) return;
    if (controller->has_previous) {
        derivative = (int32_t)error - controller->previous_error;
    }
    correction = ((int32_t)controller->kp_milli * error +
                  (int32_t)controller->kd_milli * derivative) / 1000;
    *left = clamp_output((int32_t)controller->base_output + correction);
    *right = clamp_output((int32_t)controller->base_output - correction);
    controller->previous_error = error;
    controller->has_previous = true;
}
