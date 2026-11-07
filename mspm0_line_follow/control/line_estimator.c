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

/* 上一帧质心（像素单位），用于簇选择：线是连续移动的，与上一帧
 * 位置差得远的簇几乎必然是干扰假簇。 */
static bool     s_have_prev;
static int32_t  s_prev_pos;

bool line_estimator_compute(const line_frame_t *frame,
                            const int16_t *positions,
                            int16_t *error)
{
    int32_t maximum_position = 0;
    uint32_t mask;
    uint8_t i;

    if ((frame == NULL) || (positions == NULL) || (error == NULL) ||
        frame->sensor_error || (frame->active_mask == 0U)) {
        return false;
    }
    for (i = 0U; i < frame->channel_count; ++i) {
        int32_t absolute_position = positions[i];

        if (absolute_position < 0) absolute_position = -absolute_position;
        if (absolute_position > maximum_position) {
            maximum_position = absolute_position;
        }
    }
    if (maximum_position == 0) return false;

    mask = neighborhood_filter(frame->active_mask, frame->channel_count);

    /* ---- 分簇质心（2026-11-07 实车第二轮数据）----
     * 邻域过滤只能删孤立点，删不掉「整簇假黑」：实测丢线前
     * mask 在 [3,4,5]（真线）与 [1,4,5,6]、[1,2,4,5]（真线+远处
     * 假簇，中间隔 2~3 个白通道）之间跳变。两簇一起算质心会把
     * 误差拉向两簇中间（err 66→129 跳变），转向被带歪，车滑出线。
     *
     * 规则：mask 切成连续簇，每簇独立算加权质心与总黑度；
     * 有上一帧位置时选「离上一帧最近」的簇（线不会瞬移），
     * 没有历史（上电第一帧）选总黑度最大的簇。 */
    {
        int64_t best_weighted = 0;
        uint32_t best_strength = 0U;
        int32_t best_dist = 0;
        bool have_best = false;
        uint8_t count = frame->channel_count;

        i = 0U;
        while (i < count) {
            uint8_t j;
            int64_t cluster_weighted = 0;
            uint32_t cluster_strength = 0U;

            if ((mask & ((uint32_t)1U << i)) == 0U) {
                ++i;
                continue;
            }
            j = i;
            while ((j < count) && ((mask & ((uint32_t)1U << j)) != 0U)) {
                uint16_t value = frame->normalized[j];

                /* 亚像素插值：normalized 是连续黑度，阈值 300 挡白底
                 * 噪声（白底 normalized≈0、噪声折算约 ±140）。 */
                if (value >= 300U) {
                    cluster_weighted += (int64_t)positions[j] * value;
                    cluster_strength += value;
                }
                ++j;
            }
            if (cluster_strength > 0U) {
                int32_t cluster_pos =
                    (int32_t)(cluster_weighted / cluster_strength);

                /* 选簇：总黑度最大（2026-11-07 实车全圈数据定稿）。
                 * 之前用「离上一帧最近」，实测 21 次被弱假簇抢走锁定
                 * （[1,3] 里选了 ch1），err 在 -142/-714 间横跳 93 次。
                 * 假簇是干扰弱信号、真线是强信号，黑度权重天然区分；
                 * 距离仅在权重相同（并列）时作次级判据。 */
                if (s_have_prev) {
                    int32_t dist = cluster_pos - s_prev_pos;

                    if (dist < 0) dist = -dist;
                    if ((!have_best) ||
                        (cluster_strength > best_strength) ||
                        ((cluster_strength == best_strength) &&
                         (dist < best_dist))) {
                        best_dist = dist;
                        best_weighted = cluster_weighted;
                        best_strength = cluster_strength;
                        have_best = true;
                    }
                } else {
                    if ((!have_best) ||
                        (cluster_strength > best_strength)) {
                        best_weighted = cluster_weighted;
                        best_strength = cluster_strength;
                        have_best = true;
                    }
                }
            }
            i = j;
        }

        if (!have_best) return false;
        s_prev_pos = (int32_t)(best_weighted / best_strength);
        s_have_prev = true;
        {
            int64_t out = s_prev_pos * 1000 / maximum_position;

            if (out > 1000) out = 1000;
            if (out < -1000) out = -1000;
            *error = (int16_t)out;
        }
    }
    return true;
}
