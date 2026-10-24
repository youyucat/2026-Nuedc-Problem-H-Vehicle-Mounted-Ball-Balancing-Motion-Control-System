#ifndef SERVICES_LINE_PROCESSING_H
#define SERVICES_LINE_PROCESSING_H

#include <stdbool.h>
#include <stdint.h>

#include "config/template_config.h"
#include "drivers/line_sensor.h"

typedef struct {
    uint8_t channel_count;
    bool calibrated;
    uint16_t black[TEMPLATE_SENSOR_MAX_CHANNELS];
    uint16_t white[TEMPLATE_SENSOR_MAX_CHANNELS];
    uint16_t thresholds[TEMPLATE_SENSOR_MAX_CHANNELS];
} line_calibration_data_t;

typedef struct {
    uint8_t channel_count;
    uint16_t raw[TEMPLATE_SENSOR_MAX_CHANNELS];
    uint16_t normalized[TEMPLATE_SENSOR_MAX_CHANNELS];
    uint16_t thresholds[TEMPLATE_SENSOR_MAX_CHANNELS];
    uint32_t active_mask;
    bool calibrated;
    bool sensor_error;
} line_frame_t;

void line_processing_init(void);
void line_processing_update(const line_sensor_raw_frame_t *raw,
                            line_frame_t *frame);
void line_processing_get_calibration(line_calibration_data_t *data);
bool line_processing_apply_calibration(const line_calibration_data_t *data);
bool line_processing_apply_points(
    const uint16_t black[TEMPLATE_SENSOR_MAX_CHANNELS],
    const uint16_t white[TEMPLATE_SENSOR_MAX_CHANNELS]);
bool line_processing_set_threshold(uint8_t channel, uint16_t value);
bool line_processing_set_thresholds(const uint16_t *values, uint8_t count);

#endif
