#ifndef DRIVERS_ENCODER_SPEED_H
#define DRIVERS_ENCODER_SPEED_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int32_t left_delta;
    int32_t right_delta;
    int32_t left_total;
    int32_t right_total;
    int32_t left_counts_per_second;
    int32_t right_counts_per_second;
    int32_t left_speed_mm_s;
    int32_t right_speed_mm_s;
    uint32_t left_invalid_transitions;
    uint32_t right_invalid_transitions;
    uint32_t sample_period_ms;
    uint32_t sequence;
    bool valid;
} encoder_speed_snapshot_t;

void encoder_speed_init(void);
void encoder_speed_update(uint32_t now_ms);
void encoder_speed_reset_controller_sample(void);
void encoder_speed_get(encoder_speed_snapshot_t *snapshot);
bool encoder_speed_set_calibration(uint16_t left_counts_per_revolution,
                                   uint16_t right_counts_per_revolution,
                                   uint16_t left_travel_x10_mm,
                                   uint16_t right_travel_x10_mm);
void encoder_speed_get_calibration(uint16_t *left_counts_per_revolution,
                                   uint16_t *right_counts_per_revolution,
                                   uint16_t *left_travel_x10_mm,
                                   uint16_t *right_travel_x10_mm);

#endif
