#include "services/line_processing.h"

#include <stddef.h>

static line_calibration_data_t s_calibration;

static uint16_t absolute_difference(uint16_t first, uint16_t second)
{
    return (first >= second) ?
        (uint16_t)(first - second) : (uint16_t)(second - first);
}

static bool points_valid(const line_calibration_data_t *data)
{
    if ((data == NULL) ||
        (data->channel_count != TEMPLATE_SENSOR_CHANNELS)) {
        return false;
    }
    for (uint8_t i = 0U; i < data->channel_count; ++i) {
        if ((data->black[i] > TEMPLATE_ADC_MAXIMUM) ||
            (data->white[i] > TEMPLATE_ADC_MAXIMUM) ||
            (data->thresholds[i] > TEMPLATE_ADC_MAXIMUM) ||
            (absolute_difference(data->black[i], data->white[i]) <
             TEMPLATE_CAL_MIN_SPAN)) {
            return false;
        }
    }
    return true;
}

void line_processing_init(void)
{
    s_calibration.channel_count = TEMPLATE_SENSOR_CHANNELS;
    s_calibration.calibrated = false;
    for (uint8_t i = 0U; i < TEMPLATE_SENSOR_MAX_CHANNELS; ++i) {
        s_calibration.black[i] = 0U;
        s_calibration.white[i] = TEMPLATE_ADC_MAXIMUM;
        s_calibration.thresholds[i] = TEMPLATE_ADC_MAXIMUM / 2U;
    }
}

static uint16_t normalize(uint8_t channel, uint16_t raw)
{
    uint16_t black = s_calibration.black[channel];
    uint16_t white = s_calibration.white[channel];
    uint32_t span;

    if (!s_calibration.calibrated) {
        return (uint16_t)(TEMPLATE_ADC_MAXIMUM - raw);
    }
    span = absolute_difference(black, white);
    if (black < white) {
        if (raw <= black) return TEMPLATE_ADC_MAXIMUM;
        if (raw >= white) return 0U;
        return (uint16_t)(((uint32_t)(white - raw) *
                           TEMPLATE_ADC_MAXIMUM) / span);
    }
    if (raw >= black) return TEMPLATE_ADC_MAXIMUM;
    if (raw <= white) return 0U;
    return (uint16_t)(((uint32_t)(raw - white) *
                       TEMPLATE_ADC_MAXIMUM) / span);
}

static bool is_black(uint8_t channel, uint16_t raw)
{
    if (s_calibration.black[channel] < s_calibration.white[channel]) {
        return raw < s_calibration.thresholds[channel];
    }
    return raw > s_calibration.thresholds[channel];
}

void line_processing_update(const line_sensor_raw_frame_t *raw,
                            line_frame_t *frame)
{
    if ((raw == NULL) || (frame == NULL)) return;
    frame->channel_count = raw->channel_count;
    frame->active_mask = 0U;
    frame->calibrated = s_calibration.calibrated;
    frame->sensor_error = raw->sensor_error;
    for (uint8_t i = 0U; i < raw->channel_count; ++i) {
        frame->raw[i] = raw->values[i];
        frame->normalized[i] = normalize(i, raw->values[i]);
        frame->thresholds[i] = s_calibration.thresholds[i];
        if (!raw->sensor_error && is_black(i, raw->values[i])) {
            frame->active_mask |= (uint32_t)1U << i;
        }
    }
}

void line_processing_get_calibration(line_calibration_data_t *data)
{
    if (data != NULL) *data = s_calibration;
}

bool line_processing_apply_calibration(const line_calibration_data_t *data)
{
    if ((data == NULL) || !points_valid(data)) {
        return false;
    }
    s_calibration = *data;
    return true;
}

bool line_processing_apply_points(
    const uint16_t black[TEMPLATE_SENSOR_MAX_CHANNELS],
    const uint16_t white[TEMPLATE_SENSOR_MAX_CHANNELS])
{
    line_calibration_data_t next = {0};

    if ((black == NULL) || (white == NULL)) return false;
    next.channel_count = TEMPLATE_SENSOR_CHANNELS;
    next.calibrated = true;
    for (uint8_t i = 0U; i < next.channel_count; ++i) {
        next.black[i] = black[i];
        next.white[i] = white[i];
        next.thresholds[i] =
            (uint16_t)(((uint32_t)black[i] + white[i]) / 2U);
    }
    if (!points_valid(&next)) return false;
    s_calibration = next;
    return true;
}

bool line_processing_set_threshold(uint8_t channel, uint16_t value)
{
    if ((channel >= TEMPLATE_SENSOR_CHANNELS) ||
        (value > TEMPLATE_ADC_MAXIMUM)) {
        return false;
    }
    s_calibration.thresholds[channel] = value;
    return true;
}

bool line_processing_set_thresholds(const uint16_t *values, uint8_t count)
{
    if ((values == NULL) || (count != TEMPLATE_SENSOR_CHANNELS)) return false;
    for (uint8_t i = 0U; i < count; ++i) {
        if (values[i] > TEMPLATE_ADC_MAXIMUM) return false;
    }
    for (uint8_t i = 0U; i < count; ++i) {
        s_calibration.thresholds[i] = values[i];
    }
    return true;
}
