#ifndef HAL_HAL_TIME_H
#define HAL_HAL_TIME_H

#include <stdint.h>

/* Millisecond tick source. Driven by the SysTick interrupt; safe to call
 * from any context. */
uint32_t hal_time_millis(void);

/* Busy-wait delay in microseconds (coarse; used for sensor startup). */
void hal_time_delay_us(uint32_t microseconds);

#endif
