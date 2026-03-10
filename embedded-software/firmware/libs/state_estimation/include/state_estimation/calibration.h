#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>

void update_bias(float g_bias[3], float g_data_raw[3],
                 float a_bias[3], float a_data_raw[3],
                 float expected_g[3], int64_t ticks);

#endif
