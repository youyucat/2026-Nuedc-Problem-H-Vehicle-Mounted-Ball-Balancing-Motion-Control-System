#ifndef SIM_TRACK_H
#define SIM_TRACK_H

#include <stdint.h>

/* 赛道几何：直线段 + 圆弧段的带符号距离模型。
 * signed distance 约定：沿赛道行进方向，点在线的**左侧**为负、右侧为正
 * （与 line_error 的「右偏为正」同向）。 */

typedef struct {
    uint8_t  type;        /* 0 = LINE, 1 = ARC */
    double   ax, ay;      /* LINE: 起点 / ARC: 圆心 */
    double   ux, uy;      /* LINE: 单位方向向量 */
    double   len;         /* LINE: 长度 mm */
    double   radius;      /* ARC: 半径 mm（正=逆时针弧，负=顺时针弧） */
    double   ang0, span;  /* ARC: 起始角与扫过角（rad，span 带 dir 符号） */
} track_seg_t;

void  track_set(const track_seg_t *segs, int count);
/* 点到中心线的带符号距离（mm）。取所有段中 |d| 最小者。 */
double track_signed_dist(double x, double y);
/* 沿最近段的行进进度（mm）：直线为投影长度，圆弧为 |ang|*R。 */
double track_progress(double x, double y);

#endif
