#include "control/line_estimator.h"

#include <stddef.h>

bool line_estimator_compute(const line_frame_t *frame,
                            const int16_t *positions,
                            int16_t *error)
{
    int64_t weighted = 0;
    uint32_t strength = 0U;
    int32_t maximum_position = 0;

    if ((frame == NULL) || (positions == NULL) || (error == NULL) ||
        frame->sensor_error || (frame->active_mask == 0U)) {
        return false;
    }
    for (uint8_t i = 0U; i < frame->channel_count; ++i) {
        int32_t absolute_position = positions[i];
        uint16_t value;

        if (absolute_position < 0) absolute_position = -absolute_position;
        if (absolute_position > maximum_position) {
            maximum_position = absolute_position;
        }
        if ((frame->active_mask & ((uint32_t)1U << i)) == 0U) continue;
        value = frame->normalized[i];
        if (value == 0U) value = 1U;
        weighted += (int64_t)positions[i] * value;
        strength += value;
    }
    if ((strength == 0U) || (maximum_position == 0)) return false;
    weighted /= strength;
    weighted = weighted * 1000 / maximum_position;
    if (weighted > 1000) weighted = 1000;
    if (weighted < -1000) weighted = -1000;
    *error = (int16_t)weighted;
    return true;
}
