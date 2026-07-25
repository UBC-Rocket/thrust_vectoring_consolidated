/**
 * @file    tilt_kf.c
 * @brief   2-axis 1D Kalman inclination filter implementation.
 *
 * UBC Rocket, 2026
 */
#include "state_estimation/tilt_kf.h"

#include <math.h>
#include <string.h>

#define RAD2DEG (180.0f / 3.14159265358979323846f)

static void kf_1d_init(tilt_kf_1d_t *f)
{
    memset(f, 0, sizeof(*f));
    f->P[0][0] = 1.0f;
    f->P[1][1] = 1.0f;
}

static void kf_1d_predict(tilt_kf_1d_t *f, float rate_rad_s, float dt_s)
{
    const float angle_pred = f->angle + (rate_rad_s - f->bias) * dt_s;
    const float bias_pred  = f->bias;

    /* P = F P F^T + Q,  F = [[1, -dt], [0, 1]] */
    const float fp00 = f->P[0][0] - dt_s * f->P[1][0];
    const float fp01 = f->P[0][1] - dt_s * f->P[1][1];
    const float fp10 = f->P[1][0];
    const float fp11 = f->P[1][1];

    f->P[0][0] = fp00 - dt_s * fp01 + TILT_KF_Q_ANGLE;
    f->P[0][1] = fp01;
    f->P[1][0] = fp10 - dt_s * fp11;
    f->P[1][1] = fp11 + TILT_KF_Q_BIAS;

    f->angle = angle_pred;
    f->bias  = bias_pred;
}

static void kf_1d_update(tilt_kf_1d_t *f, float z_rad)
{
    /* H = [1, 0] */
    const float s = f->P[0][0] + TILT_KF_R_ANGLE;
    if (s <= 0.0f) {
        return;
    }

    const float k0 = f->P[0][0] / s;
    const float k1 = f->P[1][0] / s;
    const float innov = z_rad - f->angle;

    f->angle += k0 * innov;
    f->bias  += k1 * innov;

    const float p00 = f->P[0][0];
    const float p01 = f->P[0][1];
    const float p10 = f->P[1][0];
    const float p11 = f->P[1][1];

    f->P[0][0] = p00 - k0 * p00;
    f->P[0][1] = p01 - k0 * p01;
    f->P[1][0] = p10 - k1 * p00;
    f->P[1][1] = p11 - k1 * p01;
}

static float clamp_dt_s(float dt_s)
{
    if (dt_s < TILT_KF_DT_MIN_S) {
        return TILT_KF_DT_MIN_S;
    }
    if (dt_s > TILT_KF_DT_MAX_S) {
        return TILT_KF_DT_MAX_S;
    }
    return dt_s;
}

void tilt_kf_init(tilt_kf_t *kf)
{
    memset(kf, 0, sizeof(*kf));
    kf_1d_init(&kf->x);
    kf_1d_init(&kf->y);
}

void tilt_kf_update(tilt_kf_t *kf,
                    uint64_t t_us,
                    float ax, float ay, float az,
                    float gx, float gy, float gz)
{
    if (kf == NULL) {
        return;
    }

    if (!kf->x.initialised) {
        kf->x.last_t_us    = t_us;
        kf->x.initialised  = true;
        kf->y.last_t_us    = t_us;
        kf->y.initialised  = true;

        const float tilt_x_meas = atan2f(ay, ax);
        const float tilt_y_meas = atan2f(az, ax);
        kf->x.angle = tilt_x_meas;
        kf->y.angle = tilt_y_meas;
        return;
    }

    float dt_s = (float)((int64_t)t_us - (int64_t)kf->x.last_t_us) * 1e-6f;
    if (dt_s <= 0.0f) {
        return;
    }
    dt_s = clamp_dt_s(dt_s);

    kf->x.last_t_us = t_us;
    kf->y.last_t_us = t_us;

    const float tilt_x_meas = atan2f(ay, ax);
    const float tilt_y_meas = atan2f(az, ax);
    const float rate_x      = TILT_KF_X_RATE_SIGN * gz;
    const float rate_y      = TILT_KF_Y_RATE_SIGN * gy;

    kf_1d_predict(&kf->x, rate_x, dt_s);
    kf_1d_predict(&kf->y, rate_y, dt_s);
    kf_1d_update(&kf->x, tilt_x_meas);
    kf_1d_update(&kf->y, tilt_y_meas);

    const float horiz_mps2 = sqrtf(ay * ay + az * az);
    kf->horiz_accel_g = horiz_mps2 / TILT_KF_GRAV_MPS2;
}

float tilt_kf_angle_x_deg(const tilt_kf_t *kf)
{
    if (kf == NULL) {
        return 0.0f;
    }
    return kf->x.angle * RAD2DEG;
}

float tilt_kf_angle_y_deg(const tilt_kf_t *kf)
{
    if (kf == NULL) {
        return 0.0f;
    }
    return kf->y.angle * RAD2DEG;
}

bool tilt_kf_is_upright(const tilt_kf_t *kf, float threshold_deg)
{
    if (kf == NULL) {
        return false;
    }

    const float thresh = (threshold_deg > 0.0f) ? threshold_deg : TILT_KF_UPRIGHT_DEG;
    const float x_deg  = fabsf(tilt_kf_angle_x_deg(kf));
    const float y_deg  = fabsf(tilt_kf_angle_y_deg(kf));

    return (x_deg < thresh) &&
           (y_deg < thresh) &&
           (kf->horiz_accel_g < TILT_KF_HORIZ_ACCEL_G);
}
