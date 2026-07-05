#include "unity.h"
#include "state_estimation/tilt_kf.h"

#include <math.h>

#define GRAV   9.80665f
#define DEG2RAD (3.14159265358979323846f / 180.0f)
#define TOL_DEG 3.0f

void test_tilt_kf_stationary_near_zero(void)
{
    tilt_kf_t kf;
    tilt_kf_init(&kf);

    /* Ideal upright: ax ~= g, ay ~= 0, az ~= 0. */
    uint64_t t_us = 0;
    for (uint32_t i = 0; i < 200; i++) {
        t_us += 1000;
        tilt_kf_update(&kf, t_us,
                       GRAV, 0.0f, 0.0f,
                       0.0f, 0.0f, 0.0f);
    }

    TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, 0.0f, tilt_kf_angle_x_deg(&kf));
    TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, 0.0f, tilt_kf_angle_y_deg(&kf));
    TEST_ASSERT_TRUE(tilt_kf_is_upright(&kf, TILT_KF_UPRIGHT_DEG));
}

void test_tilt_kf_real_sensor_offsets_small(void)
{
    tilt_kf_t kf;
    tilt_kf_init(&kf);

    /* Measured bring-up sample: small cross-axis accel on a roughly upright board. */
    uint64_t t_us = 0;
    for (uint32_t i = 0; i < 200; i++) {
        t_us += 1000;
        tilt_kf_update(&kf, t_us,
                       9.754f, 0.077f, 1.039f,
                       -0.003f, -0.012f, 0.005f);
    }

    TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, 0.0f, tilt_kf_angle_x_deg(&kf));
    TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, 6.0f, tilt_kf_angle_y_deg(&kf));
}

void test_tilt_kf_tilt_x_converges(void)
{
    tilt_kf_t kf;
    tilt_kf_init(&kf);

    const float tilt_rad = 30.0f * DEG2RAD;
    const float ax       = GRAV * cosf(tilt_rad);
    const float ay       = GRAV * sinf(tilt_rad);

    uint64_t t_us = 0;
    for (uint32_t i = 0; i < 300; i++) {
        t_us += 1000;
        tilt_kf_update(&kf, t_us, ax, ay, 0.0f, 0.0f, 0.0f, 0.0f);
    }

    TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, 30.0f, tilt_kf_angle_x_deg(&kf));
    TEST_ASSERT_FALSE(tilt_kf_is_upright(&kf, TILT_KF_UPRIGHT_DEG));
}

void test_tilt_kf_gyro_integration_tracks_rate(void)
{
    tilt_kf_t kf;
    tilt_kf_init(&kf);

    const float rate_rad_s = 0.5f; /* ~28.6 deg/s */
    float angle_rad = 0.0f;
    uint64_t t_us = 0;

    for (uint32_t i = 0; i < 250; i++) {
        t_us += 1000;
        angle_rad += rate_rad_s * 0.001f;
        const float ax = GRAV * cosf(angle_rad);
        const float ay = GRAV * sinf(angle_rad);
        tilt_kf_update(&kf, t_us, ax, ay, 0.0f, 0.0f, 0.0f, rate_rad_s);
    }

    const float expected_deg = angle_rad * (180.0f / 3.14159265358979323846f);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, expected_deg, tilt_kf_angle_x_deg(&kf));
}
