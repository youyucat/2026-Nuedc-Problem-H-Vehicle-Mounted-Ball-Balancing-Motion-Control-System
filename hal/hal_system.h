#ifndef HAL_HAL_SYSTEM_H
#define HAL_HAL_SYSTEM_H

#include <stdint.h>

/* System-level operations tied to the MCU/board. */

/* Power/clock/peripheral bring-up（SysConfig 生成的 initPower / GPIO /
 * SYSCTL 80 MHz / ADC_0 / SysTick，没有 UART/I2C 引脚上拉那两段）。
 * 在 main() 里关中断调用一次。 */
void hal_system_bringup(void);

/* Critical-section helpers so the logic layers do not need CMSIS headers. */

/* Disable interrupts and return the previous PRIMASK value for restore. */
uint32_t hal_system_critical_save(void);

/* Restore the PRIMASK value returned by hal_system_critical_save(). */
void hal_system_critical_restore(uint32_t state);

void hal_system_critical_enter(void);
void hal_system_critical_exit(void);

/* Reset the device (used to enter the ROM BSL for firmware maintenance). The
 * implementation issues the required data-synchronization barrier first. */
void hal_system_reset(void);

#endif
