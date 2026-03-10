#include "state_estimation/calibration.h"

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
