#include "drivers/encoder_speed.h"

#include <limits.h>
#include <stddef.h>

#include "hal/hal.h"
#include "config/template_config.h"

static encoder_speed_snapshot_t s_snapshot;
static uint32_t s_last_sample_ms;
static uint16_t s_left_counts_per_revolution;
static uint16_t s_right_counts_per_revolution;
static uint16_t s_left_travel_x10_mm;
static uint16_t s_right_travel_x10_mm;

static int32_t counts_per_second(int32_t delta, uint32_t elapsed_ms)
{
    int64_t value;

    if (elapsed_ms == 0U) return 0;
    value = ((int64_t)delta * 1000LL) / (int64_t)elapsed_ms;
    if (value > INT32_MAX) return INT32_MAX;
    if (value < INT32_MIN) return INT32_MIN;
    return (int32_t)value;
}

static int32_t counts_to_mm_s(int32_t delta, uint32_t elapsed_ms,
                              uint16_t counts_per_revolution,
                              uint16_t travel_x10_mm)
{
    int64_t value;
    int64_t denominator;

    if ((elapsed_ms == 0U) || (counts_per_revolution == 0U)) return 0;
    denominator = (int64_t)counts_per_revolution *
                  (int64_t)elapsed_ms;
    value = ((int64_t)delta * (int64_t)travel_x10_mm * 100LL) /
            denominator;
    if (value > INT32_MAX) return INT32_MAX;
    if (value < INT32_MIN) return INT32_MIN;
    return (int32_t)value;
}

void encoder_speed_init(void)
{
    hal_encoder_reset();
    s_snapshot = (encoder_speed_snapshot_t){0};
    s_left_counts_per_revolution = TEMPLATE_LEFT_ENCODER_COUNTS_PER_REV;
    s_right_counts_per_revolution = TEMPLATE_RIGHT_ENCODER_COUNTS_PER_REV;
    s_left_travel_x10_mm = TEMPLATE_LEFT_WHEEL_TRAVEL_X10_MM;
    s_right_travel_x10_mm = TEMPLATE_RIGHT_WHEEL_TRAVEL_X10_MM;
    s_last_sample_ms = hal_time_millis();
}

void encoder_speed_update(uint32_t now_ms)
{
    hal_encoder_snapshot_t raw;
    uint32_t elapsed_ms;

    elapsed_ms = now_ms - s_last_sample_ms;
    if (elapsed_ms < TEMPLATE_ENCODER_SAMPLE_MS) return;

    hal_encoder_snapshot(&raw, true);
    s_snapshot.left_delta = raw.left_delta;
    s_snapshot.right_delta = raw.right_delta;
    s_snapshot.left_total = raw.left_total;
    s_snapshot.right_total = raw.right_total;
    s_snapshot.left_counts_per_second =
        counts_per_second(raw.left_delta, elapsed_ms);
    s_snapshot.right_counts_per_second =
        counts_per_second(raw.right_delta, elapsed_ms);
    s_snapshot.left_speed_mm_s = counts_to_mm_s(
        raw.left_delta, elapsed_ms, s_left_counts_per_revolution,
        s_left_travel_x10_mm);
    s_snapshot.right_speed_mm_s = counts_to_mm_s(
        raw.right_delta, elapsed_ms, s_right_counts_per_revolution,
        s_right_travel_x10_mm);
    s_snapshot.left_invalid_transitions = raw.left_invalid_transitions;
    s_snapshot.right_invalid_transitions = raw.right_invalid_transitions;
    s_snapshot.sample_period_ms = elapsed_ms;
    ++s_snapshot.sequence;
    s_snapshot.valid = true;
    s_last_sample_ms = now_ms;
}

void encoder_speed_reset_controller_sample(void)
{
    hal_encoder_reset();
    s_snapshot.left_delta = 0;
    s_snapshot.right_delta = 0;
    s_snapshot.left_total = 0;
    s_snapshot.right_total = 0;
    s_snapshot.left_counts_per_second = 0;
    s_snapshot.right_counts_per_second = 0;
    s_snapshot.left_speed_mm_s = 0;
    s_snapshot.right_speed_mm_s = 0;
    s_snapshot.left_invalid_transitions = 0U;
    s_snapshot.right_invalid_transitions = 0U;
    s_snapshot.sample_period_ms = 0U;
    s_snapshot.sequence = 0U;
    s_snapshot.valid = false;
    s_last_sample_ms = hal_time_millis();
}

void encoder_speed_get(encoder_speed_snapshot_t *snapshot)
{
    if (snapshot != NULL) *snapshot = s_snapshot;
}

bool encoder_speed_set_calibration(uint16_t left_counts_per_revolution,
                                   uint16_t right_counts_per_revolution,
                                   uint16_t left_travel_x10_mm,
                                   uint16_t right_travel_x10_mm)
{
    if ((left_counts_per_revolution < 100U) ||
        (left_counts_per_revolution > 20000U) ||
        (right_counts_per_revolution < 100U) ||
        (right_counts_per_revolution > 20000U) ||
        (left_travel_x10_mm < 100U) || (left_travel_x10_mm > 5000U) ||
        (right_travel_x10_mm < 100U) || (right_travel_x10_mm > 5000U)) {
        return false;
    }
    s_left_counts_per_revolution = left_counts_per_revolution;
    s_right_counts_per_revolution = right_counts_per_revolution;
    s_left_travel_x10_mm = left_travel_x10_mm;
    s_right_travel_x10_mm = right_travel_x10_mm;
    return true;
}

void encoder_speed_get_calibration(uint16_t *left_counts_per_revolution,
                                   uint16_t *right_counts_per_revolution,
                                   uint16_t *left_travel_x10_mm,
                                   uint16_t *right_travel_x10_mm)
{
    if (left_counts_per_revolution != NULL)
        *left_counts_per_revolution = s_left_counts_per_revolution;
    if (right_counts_per_revolution != NULL)
        *right_counts_per_revolution = s_right_counts_per_revolution;
    if (left_travel_x10_mm != NULL)
        *left_travel_x10_mm = s_left_travel_x10_mm;
    if (right_travel_x10_mm != NULL)
        *right_travel_x10_mm = s_right_travel_x10_mm;
}
