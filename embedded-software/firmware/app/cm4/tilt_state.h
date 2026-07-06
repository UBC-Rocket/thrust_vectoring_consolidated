/**
 * @file    tilt_state.h
 * @brief   CM4-local single-writer tilt estimate slot (state task -> actuator task).
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
    uint64_t t_us;
    uint32_t seq;
} tilt_state_t;

void tilt_state_publish(const tilt_kf_t *kf, uint64_t t_us);
bool tilt_state_get(tilt_state_t *out);

#ifdef __cplusplus
}
#endif

#endif /* APP_TILT_STATE_H */
