/**
 * @file    ekf.c
 * @brief   Error-State Kalman Filter — multi-sensor batch processing.
 *
 * Orientation: ESKF_ERR_DIM error state [δθ, δb_g0, δb_a0, ..., δb_gN, δb_aN]
 * with per-IMU bias tracking and multiplicative quaternion updates.
 * Body: standard 6D KF [pos, vel].
 */
#include "state_estimation/ekf.h"
#include "state_estimation/quaternion.h"
#include "state_estimation/body.h"
#include "state_estimation/calibration.h"
#include "state_estimation/matrix.h"
#include "state_estimation/state.h"
#include <math.h>
#include <string.h>

// change to whatever you want the EKF yaw to output
// RADIANS
#define FIXED_YAW 0f

/* ========================================================================
 * Helpers
 * ====================================================================== */

static const eskf_sensor_noise_config_t *find_sensor_config(
    const eskf_t *eskf, eskf_sensor_type_t type, uint8_t id)
{
    for (size_t i = 0; i < eskf->num_sensor_configs; i++) {
        if (eskf->sensor_configs[i].type == type &&
            eskf->sensor_configs[i].id == id) {
            return &eskf->sensor_configs[i];
        }
    }
    return NULL;
}

typedef struct {
    uint64_t timestamp_us;
    eskf_sensor_type_t type;
    uint8_t id;
    float data[3];
} timeline_event_t;

static float yaw_from_quat(const float q[4])
{
    const float siny = 2.0f * (q[0] * q[3] + q[1] * q[2]);
    const float cosy = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
    return atan2f(siny, cosy);
}

static void set_quat_yaw(float q[4], float yaw_rad)
{
    const float sinr = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    const float cosr = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
    const float roll = atan2f(sinr, cosr);

    float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    const float pitch = asinf(sinp);

    const float hr = 0.5f * roll;
    const float hp = 0.5f * pitch;
    const float hy = 0.5f * yaw_rad;

    const float cr = cosf(hr), sr = sinf(hr);
    const float cp = cosf(hp), sp = sinf(hp);
    const float cy = cosf(hy), sy = sinf(hy);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
    normalize(q);
}

/* ========================================================================
 * Initialization
 * ====================================================================== */

void eskf_init(eskf_t *eskf, const eskf_config_t *config)
{
    memset(eskf, 0, sizeof(eskf_t));

    /* Orientation: identity quaternion, zero biases */
    eskf->orientation.q_nom[0] = 1.0f;
    eskf->orientation.yaw_nom = 0.0f;
    eskf->orientation.num_imus = config->num_imus;

    /* Initial covariance */
    for (int i = 0; i < ESKF_ERR_DIM; i++)
        eskf->orientation.covar[i][i] = 1.0f;

    /* Process noise */
    memcpy(eskf->orientation.process, config->orientation_process_noise,
           sizeof(eskf->orientation.process));

    /* Body: zero state, identity covariance */
    for (int i = 0; i < 6; i++)
        eskf->body.covar[i][i] = 1.0f;

    memcpy(eskf->body.process, config->body_process_noise,
           sizeof(eskf->body.process));

    /* Environment */
    memcpy(eskf->expected_g, config->expected_g, sizeof(eskf->expected_g));
    memcpy(eskf->mag_ref, config->mag_ref, sizeof(eskf->mag_ref));

    /* Calibration */
    eskf->cal.calibration_target = config->calibration_samples;
    eskf->cal.calibrated = (config->calibration_samples == 0);

    /* Sensor noise configs */
    size_t n = config->num_sensors;
    if (n > ESKF_MAX_SENSORS) n = ESKF_MAX_SENSORS;
    eskf->num_sensor_configs = n;
    for (size_t i = 0; i < n; i++)
        eskf->sensor_configs[i] = config->sensors[i];
}

/* ========================================================================
 * ESKF Orientation: Predict
 * ====================================================================== */

static void orientation_predict(eskf_t *eskf, const float gyro_raw[3],
                                float dt, uint8_t imu_idx)
{
    orientation_eskf_state_t *ori = &eskf->orientation;

    /* Correct gyro with this IMU's biasprocess_accel estimate */
    float gyro_corr[3] = { gyro_raw[0] - ori->b_gyro[imu_idx][0],
                           gyro_raw[1] - ori->b_gyro[imu_idx][1],
                           gyro_raw[2] - ori->b_gyro[imu_idx][2] };

    /* Propagate nominal quaternion */
    eskf_propagate_nominal(ori, gyro_corr, dt);
    ori->yaw_nom = yaw_from_quat(ori->q_nom);

    /* Error-state covariance prediction: P = F_d * P * F_d^T + Q */
    float F_d[ESKF_ERR_DIM][ESKF_ERR_DIM];
    eskf_get_F(gyro_corr, dt, F_d, imu_idx);

    float F_dT[ESKF_ERR_DIM][ESKF_ERR_DIM];
    mat_transpose((const float *)F_d, (float *)F_dT, ESKF_ERR_DIM, ESKF_ERR_DIM);

    float FP[ESKF_ERR_DIM][ESKF_ERR_DIM];
    mat_mul((const float *)F_d, (const float *)ori->covar, (float *)FP,
            ESKF_ERR_DIM, ESKF_ERR_DIM, ESKF_ERR_DIM);

    float FPFT[ESKF_ERR_DIM][ESKF_ERR_DIM];
    mat_mul((const float *)FP, (const float *)F_dT, (float *)FPFT,
            ESKF_ERR_DIM, ESKF_ERR_DIM, ESKF_ERR_DIM);

    for (int i = 0; i < ESKF_ERR_DIM; i++)
        for (int j = 0; j < ESKF_ERR_DIM; j++)
            ori->covar[i][j] = FPFT[i][j] + ori->process[i][j] * dt;
}

/* ========================================================================
 * ESKF Orientation: Measurement Update (generic 3 x ESKF_ERR_DIM)
 * ====================================================================== */

static void orientation_measurement_update(eskf_t *eskf,
                                           const float innovation[3],
                                           const float H[3][ESKF_ERR_DIM],
                                           const float R[3][3])
{
    orientation_eskf_state_t *ori = &eskf->orientation;

    /* S = H * P * H^T + R  (3x3) */
    float HT[ESKF_ERR_DIM][3];
    mat_transpose((const float *)H, (float *)HT, 3, ESKF_ERR_DIM);

    float PH_T[ESKF_ERR_DIM][3];
    mat_mul((const float *)ori->covar, (const float *)HT, (float *)PH_T,
            ESKF_ERR_DIM, ESKF_ERR_DIM, 3);

    float S[3][3];
    mat_mul((const float *)H, (const float *)PH_T, (float *)S,
            3, ESKF_ERR_DIM, 3);

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            S[i][j] += R[i][j];

    float S_inv[3][3];
    if (!inverse(S, S_inv)) return;

    /* K = P * H^T * S^-1  (ESKF_ERR_DIM x 3) */
    float K[ESKF_ERR_DIM][3];
    mat_mul((const float *)PH_T, (const float *)S_inv, (float *)K,
            ESKF_ERR_DIM, 3, 3);

    /* Error correction: δx = K * innovation */
    float dx[ESKF_ERR_DIM];
    for (int i = 0; i < ESKF_ERR_DIM; i++) {
        dx[i] = K[i][0] * innovation[0]
              + K[i][1] * innovation[1]
              + K[i][2] * innovation[2];
    }

    /* Inject: quaternion ← q_nom ⊗ exp(δθ/2) */
    quaternion_t dq;
    quaternion_from_rotation_vector(dx, &dq);  /* dx[0..2] = δθ */

    quaternion_t q_old;
    q_old.w = ori->q_nom[0];
    q_old.x = ori->q_nom[1];
    q_old.y = ori->q_nom[2];
    q_old.z = ori->q_nom[3];

    quaternion_t q_new;
    quaternion_multiply(&q_old, &dq, &q_new);

    ori->q_nom[0] = q_new.w;
    ori->q_nom[1] = q_new.x;
    ori->q_nom[2] = q_new.y;
    ori->q_nom[3] = q_new.z;
    normalize(ori->q_nom);

    /* Inject: per-IMU biases */
    for (int k = 0; k < ori->num_imus; k++) {
        for (int i = 0; i < 3; i++) {
            ori->b_gyro[k][i]  += dx[3 + 6 * k + i];
            ori->b_accel[k][i] += dx[6 + 6 * k + i];
        }
    }

    /* Covariance update: P = (I - K*H) * P */
    float KH[ESKF_ERR_DIM][ESKF_ERR_DIM];
    mat_mul((const float *)K, (const float *)H, (float *)KH,
            ESKF_ERR_DIM, 3, ESKF_ERR_DIM);

    float IKH[ESKF_ERR_DIM][ESKF_ERR_DIM];
    memset(IKH, 0, sizeof(IKH));
    for (int i = 0; i < ESKF_ERR_DIM; i++) IKH[i][i] = 1.0f;
    for (int i = 0; i < ESKF_ERR_DIM; i++)
        for (int j = 0; j < ESKF_ERR_DIM; j++)
            IKH[i][j] -= KH[i][j];

    float P_new[ESKF_ERR_DIM][ESKF_ERR_DIM];
    mat_mul((const float *)IKH, (const float *)ori->covar, (float *)P_new,
            ESKF_ERR_DIM, ESKF_ERR_DIM, ESKF_ERR_DIM);

    memcpy(ori->covar, P_new, sizeof(ori->covar));
}

/* ========================================================================
 * Body: GPS measurement update (position, 3x6)
 * ====================================================================== */

static void body_gps_update(eskf_t *eskf, const float gps_pos[3],
                            const float R[3][3])
{
    body_state_t *body = &eskf->body;

    float innovation[3];
    for (int i = 0; i < 3; i++)
        innovation[i] = gps_pos[i] - body->position[i];

    float S[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            S[i][j] = body->covar[i][j] + R[i][j];

    float S_inv[3][3];
    if (!inverse(S, S_inv)) return;

    float K[6][3];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++)
                sum += body->covar[i][k] * S_inv[k][j];
            K[i][j] = sum;
        }

    for (int i = 0; i < 3; i++) {
        float adj = K[i][0] * innovation[0]
                  + K[i][1] * innovation[1]
                  + K[i][2] * innovation[2];
        body->position[i] += adj;
    }
    for (int i = 0; i < 3; i++) {
        float adj = K[i + 3][0] * innovation[0]
                  + K[i + 3][1] * innovation[1]
                  + K[i + 3][2] * innovation[2];
        body->velocity[i] += adj;
    }

    float P_new[6][6];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            float kp = K[i][0] * body->covar[0][j]
                     + K[i][1] * body->covar[1][j]
                     + K[i][2] * body->covar[2][j];
            P_new[i][j] = body->covar[i][j] - kp;
        }

    memcpy(body->covar, P_new, sizeof(body->covar));
}

/* ========================================================================
 * Body: Barometer measurement update (scalar, z-position only)
 * ====================================================================== */

static void body_baro_update(eskf_t *eskf, float baro_height, float R_baro)
{
    body_state_t *body = &eskf->body;

    float innovation = baro_height - body->position[2];

    float S = body->covar[2][2] + R_baro;
    if (fabsf(S) < 1e-10f) return;
    float S_inv = 1.0f / S;

    float K[6];
    for (int i = 0; i < 6; i++)
        K[i] = body->covar[i][2] * S_inv;

    for (int i = 0; i < 3; i++)
        body->position[i] += K[i] * innovation;
    for (int i = 0; i < 3; i++)
        body->velocity[i] += K[i + 3] * innovation;

    float P_new[6][6];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            P_new[i][j] = body->covar[i][j] - K[i] * body->covar[2][j];

    memcpy(body->covar, P_new, sizeof(body->covar));

    /* Zero-velocity pseudo-measurement on Z when near stationary.
     * When baro innovation is small, position hasn't changed, so velocity
     * should be near zero. This damps velocity drift that accumulates from
     * tiny accel biases when no GPS velocity correction is available. */
    if (fabsf(innovation) < 0.5f) {
        float R_vel = 0.1f;
        float S_vel = body->covar[5][5] + R_vel;
        if (fabsf(S_vel) < 1e-10f) return;
        float S_vel_inv = 1.0f / S_vel;

        float Kv[6];
        for (int i = 0; i < 6; i++)
            Kv[i] = body->covar[i][5] * S_vel_inv;

        float vel_innov = 0.0f - body->velocity[2];

        body->position[2] += Kv[2] * vel_innov;
        
        for (int i = 0; i < 3; i++)
            body->velocity[i] += Kv[i + 3] * vel_innov;

        float P_zupt[6][6];
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                P_zupt[i][j] = body->covar[i][j] - Kv[i] * body->covar[5][j];

        memcpy(body->covar, P_zupt, sizeof(body->covar));
    }
}

/* ========================================================================
 * Body: Predict
 * ====================================================================== */

static void body_predict(eskf_t *eskf, const float a_nav[3], float dt)
{
    body_state_t *body = &eskf->body;

    float pred_p[3], pred_v[3];
    state_transition_body(body, dt, a_nav, pred_p, pred_v);

    float P_pred[6][6];
    predict_covar_body_sparse((const float (*)[6])body->covar,
                              (const float (*)[6])body->process,
                              dt, P_pred);

    memcpy(body->position, pred_p, sizeof(body->position));
    memcpy(body->velocity, pred_v, sizeof(body->velocity));
    memcpy(body->covar, P_pred, sizeof(body->covar));
}

/* ========================================================================
 * Batch Processing
 * ====================================================================== */

static int sensor_priority(eskf_sensor_type_t type)
{
    switch (type) {
    case ESKF_SENSOR_GYRO:  return 0;
    case ESKF_SENSOR_ACCEL: return 1;
    case ESKF_SENSOR_MAG:   return 2;
    case ESKF_SENSOR_GPS:   return 3;
    case ESKF_SENSOR_BARO:  return 4;
    default:                return 5;
    }
}

static int event_compare(const void *a, const void *b)
{
    const timeline_event_t *ea = (const timeline_event_t *)a;
    const timeline_event_t *eb = (const timeline_event_t *)b;
    if (ea->timestamp_us < eb->timestamp_us) return -1;
    if (ea->timestamp_us > eb->timestamp_us) return 1;
    return sensor_priority(ea->type) - sensor_priority(eb->type);
}

static void process_accel(eskf_t *eskf, const float accel_raw[3],
                          const float R[3][3], float dt, uint8_t imu_idx)
{
    orientation_eskf_state_t *ori = &eskf->orientation;
    float b_gyro_prev[ESKF_MAX_IMUS][3];
    float b_accel_prev[ESKF_MAX_IMUS][3];

    /* Correct accel with this IMU's bias */
    float a_corr[3] = { accel_raw[0] - ori->b_accel[imu_idx][0],
                        accel_raw[1] - ori->b_accel[imu_idx][1],
                        accel_raw[2] - ori->b_accel[imu_idx][2] };

    /* Normalize */
    float a_norm[3] = { a_corr[0], a_corr[1], a_corr[2] };
    float mag = vec3_normalize(a_norm);
    if (mag < 0.1f) return;

    /* Predicted accel from nominal quaternion */
    float a_pred[3];
    eskf_predict_accel(ori->q_nom, eskf->expected_g, a_pred);

    /* Innovation */
    float innov[3] = { a_norm[0] - a_pred[0],
                       a_norm[1] - a_pred[1],
                       a_norm[2] - a_pred[2] };

    /* H Jacobian */
    float H[3][ESKF_ERR_DIM];
    eskf_get_H_accel(ori->q_nom, eskf->expected_g, H, imu_idx);

    /* Measurement update.
     * Hard guard: accel must not inject into bias states (especially yaw via b_gz). */
    memcpy(b_gyro_prev, ori->b_gyro, sizeof(b_gyro_prev));
    memcpy(b_accel_prev, ori->b_accel, sizeof(b_accel_prev));
    orientation_measurement_update(eskf, innov, H, R);
    memcpy(ori->b_gyro, b_gyro_prev, sizeof(b_gyro_prev));
    memcpy(ori->b_accel, b_accel_prev, sizeof(b_accel_prev));

    /* Restore yaw from separately tracked yaw state. */
    set_quat_yaw(ori->q_nom, ori->yaw_nom);

    /* Body predict: transform accel to nav frame */
    if (dt > 0.0f) {
        float a_nav[3];
        transform_accel_data(a_corr, ori->q_nom, a_nav);
        body_predict(eskf, a_nav, dt);
    }
}

void eskf_process(eskf_t *eskf, const eskf_input_t *input)
{
    /* Count total events */
    size_t total = 0;
    for (size_t c = 0; c < input->num_channels; c++)
        total += input->channels[c].count;

    if (total == 0) return;

    /* Build timeline (static buffer avoids VLA stack overflow on embedded) */
#ifndef ESKF_MAX_TIMELINE_EVENTS
#define ESKF_MAX_TIMELINE_EVENTS 16000
#endif
    static timeline_event_t events[ESKF_MAX_TIMELINE_EVENTS];
    if (total > ESKF_MAX_TIMELINE_EVENTS) total = ESKF_MAX_TIMELINE_EVENTS;

    size_t idx = 0;
    for (size_t c = 0; c < input->num_channels; c++) {
        const eskf_sensor_channel_t *ch = &input->channels[c];
        for (size_t s = 0; s < ch->count; s++) {
            events[idx].timestamp_us = ch->samples[s].timestamp_us;
            events[idx].type = ch->type;
            events[idx].id = ch->id;
            memcpy(events[idx].data, ch->samples[s].data, sizeof(float) * 3);
            idx++;
        }
    }

    /* Sort chronologically with priority tie-break */
    for (size_t i = 1; i < total; i++) {
        timeline_event_t key = events[i];
        size_t j = i;
        while (j > 0 && event_compare(&key, &events[j - 1]) < 0) {
            events[j] = events[j - 1];
            j--;
        }
        events[j] = key;
    }

    /* Process events chronologically.
     * Timestamps persist in eskf->last_gyro_ts / last_accel_ts so that
     * dt is correct across consecutive eskf_process() calls (critical
     * when the caller feeds small batches of 1-2 samples per call). */

    for (size_t i = 0; i < total; i++) {
        const timeline_event_t *ev = &events[i];

        /* During calibration, only accumulate biases */
        if (!eskf->cal.calibrated) {
            if (ev->type == ESKF_SENSOR_GYRO) {
                eskf_calibration_update(eskf, ev->data, NULL, ev->id);
                eskf->last_gyro_ts = ev->timestamp_us;
            } else if (ev->type == ESKF_SENSOR_ACCEL) {
                eskf_calibration_update(eskf, NULL, ev->data, ev->id);
                eskf->last_accel_ts = ev->timestamp_us;
            } else if (ev->type == ESKF_SENSOR_MAG) {
                eskf_calibration_update_mag(eskf, ev->data);
            }
            continue;
        }

        const eskf_sensor_noise_config_t *cfg =
            find_sensor_config(eskf, ev->type, ev->id);

        switch (ev->type) {
        case ESKF_SENSOR_GYRO: {
            float dt = 0.0f;
            if (eskf->last_gyro_ts > 0 && ev->timestamp_us > eskf->last_gyro_ts)
                dt = (float)(ev->timestamp_us - eskf->last_gyro_ts) / 1e6f;
            if (dt > 0.0f)
                orientation_predict(eskf, ev->data, dt, ev->id);
            eskf->last_gyro_ts = ev->timestamp_us;
            break;
        }

        case ESKF_SENSOR_ACCEL: {
            float dt_body = 0.0f;
            if (eskf->last_accel_ts > 0 && ev->timestamp_us > eskf->last_accel_ts)
                dt_body = (float)(ev->timestamp_us - eskf->last_accel_ts) / 1e6f;
            if (cfg)
                process_accel(eskf, ev->data, cfg->noise, dt_body, ev->id);
            eskf->last_accel_ts = ev->timestamp_us;
            break;
        }

        case ESKF_SENSOR_GPS:
            if (cfg)
                body_gps_update(eskf, ev->data, cfg->noise);
            break;

        case ESKF_SENSOR_BARO:
            if (cfg)
                body_baro_update(eskf, ev->data[0], cfg->noise[0][0]);
            break;

        case ESKF_SENSOR_MAG:
            if (cfg) {
                float m_norm[3] = { ev->data[0], ev->data[1], ev->data[2] };
                float mag = vec3_normalize(m_norm);
                if (mag > 0.1f) {
                    float m_pred[3];
                    eskf_predict_mag(eskf->orientation.q_nom,
                                    eskf->mag_ref, m_pred);

                    float innov[3] = { m_norm[0] - m_pred[0],
                                       m_norm[1] - m_pred[1],
                                       m_norm[2] - m_pred[2] };

                    float H[3][ESKF_ERR_DIM];
                    eskf_get_H_mag(eskf->orientation.q_nom,
                                  eskf->mag_ref, H);

                    orientation_measurement_update(eskf, innov, H, cfg->noise);
                    eskf->orientation.yaw_nom = yaw_from_quat(eskf->orientation.q_nom);
                }
            }
            break;

        default:
            break;
        }
    }
}

/* ========================================================================
 * State Getters
 * ====================================================================== */

void eskf_get_state(const eskf_t *eskf, float quat[4], float pos[3],
                    float vel[3])
{
    memcpy(quat, eskf->orientation.q_nom, sizeof(float) * 4);
    set_quat_yaw(quat, FIXED_YAW);
    memcpy(pos, eskf->body.position, sizeof(float) * 3);
    memcpy(vel, eskf->body.velocity, sizeof(float) * 3);
}

bool eskf_is_calibrated(const eskf_t *eskf)
{
    return eskf->cal.calibrated;
}
