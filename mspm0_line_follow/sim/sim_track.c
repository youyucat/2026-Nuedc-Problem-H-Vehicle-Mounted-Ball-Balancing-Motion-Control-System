#include "sim_track.h"

#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static track_seg_t s_segs[4];
static int s_count;

static double wrap_pi(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

void track_set(const track_seg_t *segs, int count)
{
    int i;
    if ((segs == NULL) || (count <= 0)) { s_count = 0; return; }
    if (count > (int)(sizeof(s_segs) / sizeof(s_segs[0]))) {
        count = (int)(sizeof(s_segs) / sizeof(s_segs[0]));
    }
    for (i = 0; i < count; ++i) s_segs[i] = segs[i];
    s_count = count;
}

static double seg_signed_dist(const track_seg_t *s, double x, double y)
{
    double vx = x - s->ax, vy = y - s->ay;
    double t, r, phi, rel, ex, ey, tv, crossv;

    if (s->type == 0U) {                       /* ---- 直线 ---- */
        t = vx * s->ux + vy * s->uy;
        if (t <= 0.0) {
            crossv = s->ux * vy - s->uy * vx;
            return sqrt(vx * vx + vy * vy) *
                   ((crossv >= 0.0) ? -1.0 : 1.0);
        }
        if (t >= s->len) {
            ex = s->ax + s->len * s->ux;
            ey = s->ay + s->len * s->uy;
            vx = x - ex; vy = y - ey;
            crossv = s->ux * vy - s->uy * vx;
            return sqrt(vx * vx + vy * vy) *
                   ((crossv >= 0.0) ? -1.0 : 1.0);
        }
        return -(s->ux * vy - s->uy * vx);     /* 右侧为正 */
    }

    /* ---- 圆弧（radius 带符号：正=逆时针） ---- */
    r = sqrt(vx * vx + vy * vy);
    phi = atan2(vy, vx);
    rel = wrap_pi(phi - s->ang0);
    if (s->span >= 0.0) {
        if ((rel >= 0.0) && (rel <= s->span)) {
            return (s->radius > 0.0) ? (r - s->radius)
                                      : (s->radius + r);
        }
    } else {
        if ((rel <= 0.0) && (rel >= s->span)) {
            return (s->radius > 0.0) ? (r - s->radius)
                                      : (s->radius + r);
        }
    }
    /* 到最近端点的距离（符号按该端点的切向） */
    if (fabs(rel) < fabs(rel - s->span)) {
        ex = s->ax + s->radius * cos(s->ang0);
        ey = s->ay + s->radius * sin(s->ang0);
        tv = s->ang0 + ((s->span >= 0.0) ? M_PI / 2.0 : -M_PI / 2.0);
    } else {
        double a1 = s->ang0 + s->span;
        ex = s->ax + s->radius * cos(a1);
        ey = s->ay + s->radius * sin(a1);
        tv = a1 + ((s->span >= 0.0) ? M_PI / 2.0 : -M_PI / 2.0);
    }
    vx = x - ex; vy = y - ey;
    crossv = cos(tv) * vy - sin(tv) * vx;
    return sqrt(vx * vx + vy * vy) * ((crossv >= 0.0) ? 1.0 : -1.0);
}

double track_signed_dist(double x, double y)
{
    int i;
    double best = 1e12, best_signed = 0.0;

    for (i = 0; i < s_count; ++i) {
        double d = seg_signed_dist(&s_segs[i], x, y);
        if (fabs(d) < best) { best = fabs(d); best_signed = d; }
    }
    return (s_count > 0) ? best_signed : 0.0;
}

double track_progress(double x, double y)
{
    int i;
    double best = 1e12, prog = 0.0;

    for (i = 0; i < s_count; ++i) {
        const track_seg_t *s = &s_segs[i];
        double vx = x - s->ax, vy = y - s->ay;
        double d, p;

        if (s->type == 0U) {
            double t = vx * s->ux + vy * s->uy;
            if (t < 0.0) t = 0.0;
            if (t > s->len) t = s->len;
            d = fabs(track_signed_dist(x, y));
            p = t;
        } else {
            double r = sqrt(vx * vx + vy * vy);
            double phi = atan2(vy, vx);
            double rel = wrap_pi(phi - s->ang0);
            double a = rel;
            if (a < 0.0) a = 0.0;
            if (a > fabs(s->span)) a = fabs(s->span);
            d = fabs(r - fabs(s->radius));
            p = a * fabs(s->radius);
        }
        (void)d;
        if (fabs(track_signed_dist(x, y)) < best) {
            best = fabs(track_signed_dist(x, y));
            prog = p;
        }
    }
    return prog;
}
