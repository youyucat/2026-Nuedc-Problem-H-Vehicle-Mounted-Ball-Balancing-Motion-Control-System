#ifndef CONTROL_LINE_ESTIMATOR_H
#define CONTROL_LINE_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>

#include "services/line_processing.h"

bool line_estimator_compute(const line_frame_t *frame,
                            const int16_t *positions,
                            int16_t *error);

#endif
