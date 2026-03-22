#include "unity.h"
#include "state_estimation/ekf.h"
#include <math.h>
#include <string.h>

#define TOL 1e-4f

/* ---------- Helper: create a default config ---------- */

static eskf_sensor_noise_config_t default_sensor_configs[] = {
    { ESKF_SENSOR_ACCEL, 0, {{0.01f,0,0},{0,0.01f,0},{0,0,0.01f}} },
    { ESKF_SENSOR_GYRO,  0, {{0.001f,0,0},{0,0.001f,0},{0,0,0.001f}} },
    { ESKF_SENSOR_GPS,   0, {{15.0f,0,0},{0,15.0f,0},{0,0,15.0f}} },
    { ESKF_SENSOR_BARO,  0, {{1.0f,0,0},{0,0,0},{0,0,0}} },
};

static void make_default_config(eskf_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));

    cfg->num_imus = 1;

    /* Orientation process noise (continuous-time, scaled by dt in predict) */
    for (int i = 0; i < 3; i++) cfg->orientation_process_noise[i][i] = 1.0f;
    for (int k = 0; k < ESKF_MAX_IMUS; k++) {
        for (int i = 0; i < 3; i++) {
            cfg->orientation_process_noise[3 + 6*k + i][3 + 6*k + i] = 1e-4f;
            cfg->orientation_process_noise[6 + 6*k + i][6 + 6*k + i] = 1e-4f;
        }
    }

    /* Body process noise (continuous-time, scaled by dt in predict) */
    for (int i = 0; i < 3; i++) cfg->body_process_noise[i][i] = 100.0f;
    for (int i = 3; i < 6; i++) cfg->body_process_noise[i][i] = 10.0f;

    cfg->sensors = default_sensor_configs;
    cfg->num_sensors = sizeof(default_sensor_configs) / sizeof(default_sensor_configs[0]);

    cfg->expected_g[0] = 0.0f;
    cfg->expected_g[1] = 0.0f;
    cfg->expected_g[2] = 1.0f;

    cfg->mag_ref[0] = 1.0f;
    cfg->mag_ref[1] = 0.0f;
    cfg->mag_ref[2] = 0.0f;

    cfg->calibration_samples = 0; /* No calibration for basic tests */
}

/* ---------- eskf_init + eskf_get_state ---------- */

void test_eskf_init_gives_identity_quaternion(void)
{
    eskf_t eskf;
    eskf_config_t cfg;
    make_default_config(&cfg);
    eskf_init(&eskf, &cfg);

    float q[4], pos[3], vel[3];
    eskf_get_state(&eskf, q, pos, vel);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, q[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, q[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, q[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, q[3]);
    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, pos[i]);
        TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, vel[i]);
    }
}

/* ---------- ESKF orientation: stable at rest ---------- */

void test_eskf_orientation_stable_at_rest(void)
{
    eskf_t eskf;
    eskf_config_t cfg;
    make_default_config(&cfg);
    eskf_init(&eskf, &cfg);

    /* Feed zero gyro and gravity-aligned accel for 100 steps */
    float dt_us = 10000; /* 10ms = 100 Hz */
    int n = 100;

    eskf_sample_t gyro_samples[100];
    eskf_sample_t accel_samples[100];

    for (int i = 0; i < n; i++) {
        uint64_t t = (uint64_t)(i + 1) * (uint64_t)dt_us;
        gyro_samples[i].timestamp_us = t;
        gyro_samples[i].data[0] = 0.0f;
        gyro_samples[i].data[1] = 0.0f;
        gyro_samples[i].data[2] = 0.0f;

        accel_samples[i].timestamp_us = t;
        accel_samples[i].data[0] = 0.0f;
        accel_samples[i].data[1] = 0.0f;
        accel_samples[i].data[2] = 1.0f; /* gravity in z */
    }

    eskf_sensor_channel_t channels[] = {
        { ESKF_SENSOR_GYRO,  0, gyro_samples,  n },
        { ESKF_SENSOR_ACCEL, 0, accel_samples, n },
    };
    eskf_input_t input = { channels, 2 };
    eskf_process(&eskf, &input);

    float q[4], pos[3], vel[3];
    eskf_get_state(&eskf, q, pos, vel);

    /* Quaternion should be near identity */
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, q[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, q[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, q[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, q[3]);

    /* Unit norm */
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, norm);
}

/* ---------- ESKF body: zero accel + GPS at origin ---------- */

void test_eskf_body_zero_accel_gps_origin(void)
{
    eskf_t eskf;
    eskf_config_t cfg;
    make_default_config(&cfg);
    eskf_init(&eskf, &cfg);

    int n = 50;
    float dt_us = 10000;

    eskf_sample_t gyro_samples[50];
    eskf_sample_t accel_samples[50];
    eskf_sample_t gps_samples[5]; /* GPS at 10 Hz -> every 10 steps */

    for (int i = 0; i < n; i++) {
        uint64_t t = (uint64_t)(i + 1) * (uint64_t)dt_us;
        gyro_samples[i].timestamp_us = t;
        gyro_samples[i].data[0] = 0.0f;
        gyro_samples[i].data[1] = 0.0f;
        gyro_samples[i].data[2] = 0.0f;

        accel_samples[i].timestamp_us = t;
        accel_samples[i].data[0] = 0.0f;
        accel_samples[i].data[1] = 0.0f;
        accel_samples[i].data[2] = 1.0f;
    }

    for (int i = 0; i < 5; i++) {
        gps_samples[i].timestamp_us = (uint64_t)(i + 1) * 10 * (uint64_t)dt_us;
        gps_samples[i].data[0] = 0.0f;
        gps_samples[i].data[1] = 0.0f;
        gps_samples[i].data[2] = 0.0f;
    }

    eskf_sensor_channel_t channels[] = {
        { ESKF_SENSOR_GYRO,  0, gyro_samples,  n },
        { ESKF_SENSOR_ACCEL, 0, accel_samples, n },
        { ESKF_SENSOR_GPS,   0, gps_samples,   5 },
    };
    eskf_input_t input = { channels, 3 };
    eskf_process(&eskf, &input);

    float q[4], pos[3], vel[3];
    eskf_get_state(&eskf, q, pos, vel);

    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, pos[i]);
        TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, vel[i]);
    }
}

/* ---------- Calibration ---------- */

void test_eskf_calibration_seeds_biases(void)
{
    eskf_t eskf;
    eskf_config_t cfg;
    make_default_config(&cfg);
    cfg.calibration_samples = 10;
    eskf_init(&eskf, &cfg);

    TEST_ASSERT_FALSE(eskf_is_calibrated(&eskf));

    /* Feed 10 gyro+accel samples with known bias */
    float gyro_bias[3] = {0.01f, -0.02f, 0.03f};
    float accel_bias[3] = {0.05f, -0.03f, 0.02f};

    eskf_sample_t gyro_samples[10];
    eskf_sample_t accel_samples[10];

    for (int i = 0; i < 10; i++) {
        uint64_t t = (uint64_t)(i + 1) * 10000;
        gyro_samples[i].timestamp_us = t;
        gyro_samples[i].data[0] = gyro_bias[0];
        gyro_samples[i].data[1] = gyro_bias[1];
        gyro_samples[i].data[2] = gyro_bias[2];

        accel_samples[i].timestamp_us = t;
        /* At rest, accel reads expected_g + bias */
        accel_samples[i].data[0] = 0.0f + accel_bias[0];
        accel_samples[i].data[1] = 0.0f + accel_bias[1];
        accel_samples[i].data[2] = 1.0f + accel_bias[2];
    }

    eskf_sensor_channel_t channels[] = {
        { ESKF_SENSOR_GYRO,  0, gyro_samples,  10 },
        { ESKF_SENSOR_ACCEL, 0, accel_samples, 10 },
    };
    eskf_input_t input = { channels, 2 };
    eskf_process(&eskf, &input);

    TEST_ASSERT_TRUE(eskf_is_calibrated(&eskf));

    /* Biases should be approximately the injected values */
    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_FLOAT_WITHIN(0.01f, gyro_bias[i],
                                 eskf.orientation.b_gyro[0][i]);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, accel_bias[i],
                                 eskf.orientation.b_accel[0][i]);
    }
}
