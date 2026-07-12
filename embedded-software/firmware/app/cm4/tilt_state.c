/**
 * @file    tilt_state.c
 * @brief   CM4-local tilt estimate slot (seqlock-style publish/get).
 *
 * UBC Rocket, 2026
 */
#include "tilt_state.h"

#include <stddef.h>

static volatile struct {
    uint32_t seq;
    float    tilt_x_rad;
    float    tilt_y_rad;
    float    rate_x_rad_s;
    float    rate_y_rad_s;
    uint64_t t_us;
} s_slot;

void tilt_state_publish(const tilt_kf_t *kf, uint64_t t_us,
                        float rate_x_rad_s, float rate_y_rad_s)
{
    if (kf == NULL) {
        return;
    }

    s_slot.tilt_x_rad     = kf->x.angle;
    s_slot.tilt_y_rad     = kf->y.angle;
    s_slot.rate_x_rad_s   = rate_x_rad_s;
    s_slot.rate_y_rad_s   = rate_y_rad_s;
    s_slot.t_us           = t_us;
    s_slot.seq++;
}

bool tilt_state_get(tilt_state_t *out)
{
    if (out == NULL) {
        return false;
    }

    uint32_t seq1;
    uint32_t seq2;

    do {
        seq1 = s_slot.seq;
        if (seq1 == 0U) {
            return false;
        }
        out->tilt_x_rad     = s_slot.tilt_x_rad;
        out->tilt_y_rad     = s_slot.tilt_y_rad;
        out->rate_x_rad_s   = s_slot.rate_x_rad_s;
        out->rate_y_rad_s   = s_slot.rate_y_rad_s;
        out->t_us           = s_slot.t_us;
        out->seq        = seq1;
        seq2            = s_slot.seq;
    } while (seq1 != seq2);

    return true;
}
