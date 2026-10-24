#ifndef HAL_HAL_H
#define HAL_HAL_H

/*
 * Hardware abstraction umbrella header（循迹专用工程）.
 *
 * Logic layers (app/, control/, services/, drivers/) include this header
 * (or the individual hal_*.h modules) and never include MCU SDK headers.
 * The bsp/ directory is the only place that implements these interfaces and
 * may use the MSPM0 DriverLib (DL_*) plus SysConfig generated code.
 *
 * 与源工程（循迹 + 步进球平衡一体机）相比，这里只保留「循迹」这一半的接口：
 *   hal_time / hal_gray / hal_motor / hal_encoder / hal_system
 * 已移除：hal_uart、hal_i2c、hal_flash、hal_stepper
 * （串口调参、IMU/OLED、Flash 参数区、步进闭环都归另一颗芯片或上位机了）。
 *
 * Porting to another MSPM0 board: edit line_follow_mspm0g3507.syscfg
 * (peripherals/pins), bsp/board_profile.h (pin/MUX map) and the
 * bsp/hal_*_mspm0.c implementations. Nothing in hal/ or above needs to change.
 */

#include "hal/hal_time.h"
#include "hal/hal_gray.h"
#include "hal/hal_motor.h"
#include "hal/hal_encoder.h"
#include "hal/hal_system.h"

/* Board-wide bring-up of the peripherals behind the hal_* interfaces:
 * gray MUX/ADC pins, motor GPIO/PWM and the wheel encoders. Called once
 * before the application loop starts. */
void hal_board_init(void);

#endif
