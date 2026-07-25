/**
 * @file    calibration.c
 * @brief   Stationary calibration — accumulates per-IMU sensor biases during startup.
 *
 * Assumes the vehicle is stationary: zero angular rate, accel reads gravity only.
 * When calibration completes, seeds the ESKF per-IMU bias estimates and mag reference.
 */
#include "state_estimation/calibration.h"
#include "state_estimation/state.h"
#include <string.h>
#include <math.h>

void eskf_calibration_update(eskf_t *eskf, const float gyro[3],
                             const float accel[3], uint8_t imu_idx)
{
    if (eskf->cal.calibrated) return;
    if (imu_idx >= ESKF_MAX_IMUS) return;

    if (gyro) {
        uint32_t n = eskf->cal.gyro_count[imu_idx];
        if (n == 0) {
            memcpy(eskf->cal.gyro_accum[imu_idx], gyro, sizeof(float) * 3);
        } else {
            float w_old = (float)n / (float)(n + 1);
            float w_new = 1.0f / (float)(n + 1);
            for (int i = 0; i < 3; i++)
                eskf->cal.gyro_accum[imu_idx][i] =
                    eskf->cal.gyro_accum[imu_idx][i] * w_old + gyro[i] * w_new;
        }
        eskf->cal.gyro_count[imu_idx]++;
    }

    if (accel) {
        uint32_t n = eskf->cal.accel_count[imu_idx];
        if (n == 0) {
            for (int i = 0; i < 3; i++)
                eskf->cal.accel_accum[imu_idx][i] = accel[i] - eskf->expected_g[i];
        } else {
            float w_old = (float)n / (float)(n + 1);
            float w_new = 1.0f / (float)(n + 1);
            for (int i = 0; i < 3; i++)
                eskf->cal.accel_accum[imu_idx][i] =
                    eskf->cal.accel_accum[imu_idx][i] * w_old
                    + (accel[i] - eskf->expected_g[i]) * w_new;
        }
        eskf->cal.accel_count[imu_idx]++;
    }

    /* Complete when all active IMUs have enough samples */
    bool all_done = true;
    for (int k = 0; k < eskf->orientation.num_imus; k++) {
        if (eskf->cal.gyro_count[k] < eskf->cal.calibration_target ||
            eskf->cal.accel_count[k] < eskf->cal.calibration_target) {
            all_done = false;
            break;
        }
    }

    if (all_done) {
        for (int k = 0; k < eskf->orientation.num_imus; k++) {
            memcpy(eskf->orientation.b_gyro[k], eskf->cal.gyro_accum[k],
                   sizeof(float) * 3);
            memcpy(eskf->orientation.b_accel[k], eskf->cal.accel_accum[k],
                   sizeof(float) * 3);
        }

        /* Reset bias covariance to reflect calibration quality.
         * Clear cross-covariance between bias and attitude states to
         * prevent the large initial P from coupling accel updates into
         * spurious bias corrections. */
        for (int k = 0; k < eskf->orientation.num_imus; k++) {
            for (int i = 0; i < 3; i++) {
                int bg_idx = 3 + 6 * k + i;
                int ba_idx = 6 + 6 * k + i;
                for (int j = 0; j < ESKF_ERR_DIM; j++) {
                    eskf->orientation.covar[bg_idx][j] = 0.0f;
                    eskf->orientation.covar[j][bg_idx] = 0.0f;
                    eskf->orientation.covar[ba_idx][j] = 0.0f;
                    eskf->orientation.covar[j][ba_idx] = 0.0f;
                }
                eskf->orientation.covar[bg_idx][bg_idx] = 1e-4f;
                eskf->orientation.covar[ba_idx][ba_idx] = 1e-4f;
            }
        }

        eskf->cal.calibrated = true;
    }
}

void eskf_calibration_update_mag(eskf_t *eskf, const float mag[3])
{
    if (eskf->cal.calibrated) return;

    uint32_t n = eskf->cal.mag_count;

    if (n == 0) {
        memcpy(eskf->cal.mag_accum, mag, sizeof(float) * 3);
    } else {
        float w_old = (float)n / (float)(n + 1);
        float w_new = 1.0f / (float)(n + 1);
        for (int i = 0; i < 3; i++)
            eskf->cal.mag_accum[i] = eskf->cal.mag_accum[i] * w_old
                                   + mag[i] * w_new;
    }
    eskf->cal.mag_count++;

    float m[3] = { eskf->cal.mag_accum[0],
                   eskf->cal.mag_accum[1],
                   eskf->cal.mag_accum[2] };
    float mag_norm = vec3_normalize(m);
    if (mag_norm > 0.1f) {
        memcpy(eskf->mag_ref, m, sizeof(float) * 3);
    }
}
