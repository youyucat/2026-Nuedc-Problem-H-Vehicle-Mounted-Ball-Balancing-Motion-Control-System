#ifndef DRIVERS_LINE_SENSOR_H
#define DRIVERS_LINE_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#include "config/template_config.h"

typedef struct {
    uint8_t channel_count;
    uint16_t values[TEMPLATE_SENSOR_MAX_CHANNELS];
    bool sensor_error;
} line_sensor_raw_frame_t;

void line_sensor_init(void);

/* 同步读取一帧（忙等 DMA 后台帧完成）。仅在启动/诊断场景使用；
 * 控制循环应使用下面的异步接口。 */
bool line_sensor_read(line_sensor_raw_frame_t *frame);

/* 异步帧驱动：每控制周期调用一次 —— 空闲时发起后台 DMA 采样，
 * 若新帧已就绪则写入 frame 并返回 true，否则返回 false（调用方沿用
 * 上一帧）。控制 tick 不再阻塞等待 ADC 转换。 */
bool line_sensor_async_step(line_sensor_raw_frame_t *frame);

#endif