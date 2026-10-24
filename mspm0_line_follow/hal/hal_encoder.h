#ifndef HAL_HAL_ENCODER_H
#define HAL_HAL_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

/* Wheel quadrature encoder interface. The decoder counts every valid A/B
 * edge (x4) in the GPIO interrupt handler. */

typedef struct {
    int32_t left_delta;
    int32_t right_delta;
    int32_t left_total;
    int32_t right_total;
    uint32_t left_invalid_transitions;
    uint32_t right_invalid_transitions;
    uint8_t left_state;
    uint8_t right_state;
} hal_encoder_snapshot_t;

/* Configure the encoder pins, decoder state and GPIO interrupt. */
void hal_encoder_init(void);

/* Zero the counters; safe to call at any time. */
void hal_encoder_reset(void);

/* Atomically copy the current counters. When reset_delta is true the
 * per-sample deltas are cleared after the copy. */
void hal_encoder_snapshot(hal_encoder_snapshot_t *snapshot,
                          bool reset_delta);

#endif
