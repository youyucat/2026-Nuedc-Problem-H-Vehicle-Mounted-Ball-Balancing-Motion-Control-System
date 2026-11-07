#include "control/line_estimator.h"

#include <stddef.h>

/* 邻域一致性过滤：反射式灰度在车体晃动 / 电机干扰 / 探头脏污下会出现
 * 孤立误判。2026-11-07 实车数据（tools/runs 一轮 101 帧）：
 *   - 57% 的黑通道是孤立点（左右邻居都白）
 *   - 出现 [2,5,6]（跨 4 路）、[1,2,4,7]（跨 6 路）这类组合，
 *     而线宽 24mm、探头间距 10mm，物理上不可能
 *   - ch4（黑电平异常 1265、阈值 2211）单独误判黑 12 次
 * 这些假黑点把质心来回拉，误差以 0.8Hz 摆动 → 车摆头。
 * 只保留与左右邻居至少一个连续的黑通道；若全被过滤掉则退回原 mask
 * （宁可带着噪声，也不要误判成丢线）。 */
static uint32_t neighborhood_filter(uint32_t mask, uint8_t count)
{
    uint32_t filtered = 0U;
    uint8_t i;

    for (i = 0U; i < count; ++i) {
        if ((mask & ((uint32_t)1U << i)) == 0U) continue;
        if (((i > 0U) && ((mask & ((uint32_t)1U << (i - 1U))) != 0U)) ||
            ((i + 1U < count) && ((mask & ((uint32_t)1U << (i + 1U))) != 0U))) {
            filtered |= (uint32_t)1U << i;
        }
    }
    return (filtered != 0U) ? filtered : mask;
}

bool line_estimator_compute(const line_frame_t *frame,
                            const int16_t *positions,
                            int16_t *error)
{
    int64_t weighted = 0;
    uint32_t strength = 0U;
    uint32_t mask;
    int32_t maximum_position = 0;

    if ((frame == NULL) || (positions == NULL) || (error == NULL) ||
        frame->sensor_error || (frame->active_mask == 0U)) {
        return false;
    }
    mask = neighborhood_filter(frame->active_mask, frame->channel_count);
    for (uint8_t i = 0U; i < frame->channel_count; ++i) {
        int32_t absolute_position = positions[i];
        uint32_t bit = (uint32_t)1U << i;
        uint16_t value;

        if (absolute_position < 0) absolute_position = -absolute_position;
        if (absolute_position > maximum_position) {
            maximum_position = absolute_position;
        }
        value = frame->normalized[i];
        /* 假黑清零：判了黑（active_mask）但被邻域一致性剔除的通道，
         * 是车体晃动/干扰引起的误判，不许参与质心。 */
        if (((frame->active_mask & bit) != 0U) && ((mask & bit) == 0U)) {
            continue;
        }
        /* 亚像素插值：不再要求「已判黑」。线压在两个探头之间时，两路
         * 都有中间黑度，用连续值加权才能得到连续误差；原实现只取二值
         * mask，误差只能落在 8 个离散台阶上（实测 err 全是 143 的倍数），
         * PD 在每个台阶跳变处都踢一次 → 0.8Hz 摆头。
         * NORM_MIN 挡掉白底噪声（白底 normalized≈0，噪声折算约 ±140）。 */
        if (value < 300U) continue;
        weighted += (int64_t)positions[i] * value;
        strength += value;
    }
    if ((strength == 0U) || (maximum_position == 0)) return false;
    weighted /= strength;
    weighted = weighted * 1000 / maximum_position;
    if (weighted > 1000) weighted = 1000;
    if (weighted < -1000) weighted = -1000;
    *error = (int16_t)weighted;
    return true;
}
