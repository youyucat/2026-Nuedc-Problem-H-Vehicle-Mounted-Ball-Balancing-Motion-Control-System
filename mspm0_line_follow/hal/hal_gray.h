#ifndef HAL_HAL_GRAY_H
#define HAL_HAL_GRAY_H

#include <stdbool.h>
#include <stdint.h>

/* Eight-channel grayscale sensor front end: external MUX address lines plus
 * a single analog output sampled by the MCU ADC. The physical channel order
 * is fixed by the sensor board harness, so the MUX address is remapped to the
 * logical left-to-right channel index via hal_gray_output_index(). */

/* Enable (or disable) the sensor board power/select gate. */
void hal_gray_enable(bool enabled);

/* Drive the MUX address lines for one sensor channel (0..N-1). */
void hal_gray_select(uint8_t address);

/* Sample the MUX analog output with the configured averaging. Returns false
 * on conversion timeout. (Synchronous, used at startup only -- the control
 * loop consumes the interrupt-driven frames via hal_gray_sampling_* below.) */
bool hal_gray_read_adc(uint16_t *value);

/* 异步采样（MSPM0G3507 ADC + MUX，中断驱动、不用 FIFO/DMA）：
 * hal_gray_sampling_begin() 发起一整帧（8 路 MUX，逐路切换，每路连续转换
 * 8 次，ADC 的 MEM0_RESULT_LOADED 中断推进状态机，控制环不再忙等转换）；
 * hal_gray_sampling_take() 在新帧完成时取走结果（values 按通道物理顺序），
 * 未就绪返回 false，调用方沿用上一帧。 */
bool hal_gray_sampling_begin(void);
bool hal_gray_sampling_take(uint16_t values[8], bool *error);

/* True when the sensor board reports an error line active. */
bool hal_gray_error_active(void);

/* Logical output index for a MUX address (harness remap), 0..N-1. */
uint8_t hal_gray_output_index(uint8_t address);

/* Physical left-to-right positions of the sensor channels, used by the line
 * estimator. */
const int16_t *hal_gray_positions(void);

#endif
