#include "state_estimation/calibration.h"
#include <math.h>

#define STATIONARY_ACCEL_NORM_MIN_G   0.95f
#define STATIONARY_ACCEL_NORM_MAX_G   1.05f
#define STATIONARY_GYRO_NORM_MAX_RAD_S 0.08f

bool imu_is_stationary(const float g_data_raw[3], const float a_data_raw[3])
{
    float g_norm = sqrtf(g_data_raw[0] * g_data_raw[0] +
                         g_data_raw[1] * g_data_raw[1] +
                         g_data_raw[2] * g_data_raw[2]);

    float a_norm = sqrtf(a_data_raw[0] * a_data_raw[0] +
                         a_data_raw[1] * a_data_raw[1] +
                         a_data_raw[2] * a_data_raw[2]);

    return (g_norm < STATIONARY_GYRO_NORM_MAX_RAD_S) &&
           (a_norm > STATIONARY_ACCEL_NORM_MIN_G) &&
           (a_norm < STATIONARY_ACCEL_NORM_MAX_G);
}

void update_bias(float g_bias[3], float g_data_raw[3],
                 float a_bias[3], float a_data_raw[3],
                 float expected_g[3], int64_t ticks)
{
    if (ticks == 0) {
        for (int i = 0; i < 3; i++) g_bias[i] = g_data_raw[i];
        for (int i = 0; i < 3; i++) a_bias[i] = a_data_raw[i] - expected_g[i];
    }
    for (int i = 0; i < 3; i++) {
        g_bias[i] = g_bias[i] * (((float)ticks - 1.0f) / (float)ticks)
                   + g_data_raw[i] * (1.0f / (float)ticks);
        a_bias[i] = a_bias[i] * (((float)ticks - 1.0f) / (float)ticks)
                   + (a_data_raw[i] - expected_g[i]) * (1.0f / (float)ticks);
    }
}
