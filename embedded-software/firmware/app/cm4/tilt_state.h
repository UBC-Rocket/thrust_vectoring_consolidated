/**
 * @file    tilt_state.h
 * @brief   CM4-local tilt estimate slot (state_estimation_task -> actuator_task).
 *
 * UBC Rocket, 2026
 */
#ifndef APP_TILT_STATE_H
#define APP_TILT_STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "state_estimation/tilt_kf.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float    tilt_x_rad;
    float    tilt_y_rad;
    float    rate_x_rad_s;
    float    rate_y_rad_s;
    uint64_t t_us;
    uint32_t seq;
} tilt_state_t;

void tilt_state_publish(const tilt_kf_t *kf, uint64_t t_us,
                        float rate_x_rad_s, float rate_y_rad_s);
bool tilt_state_get(tilt_state_t *out);

#ifdef __cplusplus
}
#endif

#endif /* APP_TILT_STATE_H */
