#include "unity.h"
#include "state_estimation/ekf.h"
#include "state_estimation/quaternion.h"
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

static void euler_zyx_to_quat(float roll, float pitch, float yaw, float q[4])
{
    const float hr = 0.5f * roll;
    const float hp = 0.5f * pitch;
    const float hy = 0.5f * yaw;

    const float cr = cosf(hr), sr = sinf(hr);
    const float cp = cosf(hp), sp = sinf(hp);
    const float cy = cosf(hy), sy = sinf(hy);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

static bool extract_nav_z_twist_from_quat(const float q[4], float q_twist[4])
{
    q_twist[0] = q[0];
    q_twist[1] = 0.0f;
    q_twist[2] = 0.0f;
    q_twist[3] = q[3];

    const float norm_sq = q_twist[0] * q_twist[0] + q_twist[3] * q_twist[3];
    if (norm_sq < 1e-10f) {
        return false;
    }

    const float inv_norm = 1.0f / sqrtf(norm_sq);
    q_twist[0] *= inv_norm;
    q_twist[3] *= inv_norm;
    return true;
}

static float quat_abs_dot(const float a[4], const float b[4])
{
    float dot = 0.0f;
    for (int i = 0; i < 4; i++) {
        dot += a[i] * b[i];
    }
    return fabsf(dot);
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

void test_eskf_body_free_fall_accel_predicts_downward_motion(void)
{
    eskf_t eskf;
    eskf_config_t cfg;
    make_default_config(&cfg);
    eskf_init(&eskf, &cfg);

    const int n = 50;
    const uint64_t dt_us = 10000;

    eskf_sample_t accel_samples[n];
    for (int i = 0; i < n; i++) {
        accel_samples[i].timestamp_us = (uint64_t)(i + 1) * dt_us;
        accel_samples[i].data[0] = 0.0f;
        accel_samples[i].data[1] = 0.0f;
        accel_samples[i].data[2] = 0.0f;
    }

    eskf_sensor_channel_t channels[] = {
        { ESKF_SENSOR_ACCEL, 0, accel_samples, n },
    };
    eskf_input_t input = { channels, 1 };
    eskf_process(&eskf, &input);

    float q[4], pos[3], vel[3];
    eskf_get_state(&eskf, q, pos, vel);

    const float total_dt = (float)(n - 1) * ((float)dt_us / 1e6f);
    const float expected_vz = -9.807f * total_dt;
    const float expected_pz = -0.5f * 9.807f * total_dt * total_dt;

    TEST_ASSERT_FLOAT_WITHIN(0.05f, expected_vz, vel[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, expected_pz, pos[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, pos[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, pos[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, vel[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, vel[1]);
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

void test_eskf_twist_lock_stable_near_vertical_pitch(void)
{
    eskf_t eskf;
    eskf_config_t cfg;
    make_default_config(&cfg);
    eskf_init(&eskf, &cfg);

    const float roll = 0.35f;
    const float pitch = 1.5533430f; /* ~89 deg, near Euler singularity */
    float twist_before[4];

    euler_zyx_to_quat(roll, pitch, -0.70f, eskf.orientation.q_nom);
    TEST_ASSERT_TRUE(extract_nav_z_twist_from_quat(eskf.orientation.q_nom, twist_before));
    memcpy(eskf.orientation.q_twist_nom, twist_before, sizeof(twist_before));

    eskf_sample_t accel_sample = {0};
    accel_sample.timestamp_us = 10000;
    eskf_predict_accel(eskf.orientation.q_nom, eskf.expected_g, accel_sample.data);
    accel_sample.data[0] += 1e-4f; /* tiny non-zero innovation */

    eskf_sensor_channel_t channels[] = {
        { ESKF_SENSOR_ACCEL, 0, &accel_sample, 1 },
    };
    eskf_input_t input = { channels, 1 };
    eskf_process(&eskf, &input);

    float q[4];
    memcpy(q, eskf.orientation.q_nom, sizeof(q));

    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_FALSE(isnan(q[i]));
        TEST_ASSERT_FALSE(isinf(q[i]));
    }

    const float norm =
        sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, norm);

    float twist_after[4];
    TEST_ASSERT_TRUE(extract_nav_z_twist_from_quat(q, twist_after));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f,
                             quat_abs_dot(twist_before, eskf.orientation.q_twist_nom));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f,
                             quat_abs_dot(twist_before, twist_after));
}
