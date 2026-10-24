#include "drivers/line_sensor.h"

#include <stddef.h>

#include "hal/hal.h"

void line_sensor_init(void)
{
    hal_gray_enable(true);
}

static bool take_frame(line_sensor_raw_frame_t *frame)
{
    uint16_t values[TEMPLATE_SENSOR_CHANNELS];
    bool error = false;

    if (frame == NULL) return false;
    if (!hal_gray_sampling_take(values, &error)) return false;

    frame->channel_count = TEMPLATE_SENSOR_CHANNELS;
    frame->sensor_error = error;
    /* values 按通道序号（0..N-1）给出，经 hal_gray_output_index 重映射到
     * 物理左→右顺序（板级 MUX 扫描顺序修正）。 */
    for (uint8_t i = 0U; i < TEMPLATE_SENSOR_CHANNELS; ++i) {
        uint8_t output = hal_gray_output_index(i);
        if (output >= TEMPLATE_SENSOR_CHANNELS) {
            frame->sensor_error = true;
            continue;
        }
        frame->values[output] = values[i];
    }
    return true;
}

bool line_sensor_read(line_sensor_raw_frame_t *frame)
{
    /* 同步兜底：发起后台帧并忙等其完成（启动/诊断路径）。 */
    hal_gray_sampling_begin();
    for (uint32_t guard = 0U; guard < 1000000U; ++guard) {
        if (take_frame(frame)) return true;
    }
    if (frame != NULL) {
        frame->channel_count = TEMPLATE_SENSOR_CHANNELS;
        frame->sensor_error = true;
    }
    return false;
}

bool line_sensor_async_step(line_sensor_raw_frame_t *frame)
{
    /* 必须先取走上一帧、再发新帧。若先 begin()，hal_gray_sampling_begin()
     * 会把上一帧刚置位的 s_gray_frame_ready 清零并立刻开始新一帧，导致
     * take() 永远读到"未就绪"——所有通道恒为 0、sensor_error 恒置位。 */
    bool ready = take_frame(frame);
    hal_gray_sampling_begin(); /* 空闲则发起下一帧 */
    return ready;
}