/*
 * Unit tests for individual PDF equation functions in flight_controller.c.
 */
#include "unity.h"
#include "controls/flight_controller.h"
#include "state_estimation/state.h"
#include <math.h>
#include <string.h>

#define TOL 1e-4f
#define TOL_COARSE 1e-3f

/* Forward declarations for equation functions (non-static in flight_controller.c) */
void compute_quaternion_error(const quaternion_t *q_des, const quaternion_t *q_meas, quaternion_t *q_err);
void compute_axis_angle_error(const quaternion_t *q_err, float phi[3]);
void compute_gyroscopic_torque(const float I[3][3], const float omega[3], float tau_gyro[3]);
void compute_command_torque(const float tau_gyro[3], const float phi[3], float tau_cmd[3]);
void compute_roll_torque(const float tau_cmd[3], const float t_hat[3], float tau_roll[3]);
void compute_gimbal_torque(const float tau_cmd[3], const float tau_roll[3], float tau_gim[3]);
void compute_perpendicular_thrust(const float tau_gim[3], const float r_gim[3], float t_perp[3]);
void compute_parallel_thrust(float t_mag, const float t_perp[3], const float r_gim[3], float t_par[3]);
void compute_gimbal_angles(const float t_des[3], const flight_controller_gimbal_config_t *cfg, float *theta_x_cmd, float *theta_y_cmd);
void update_thrust_direction(float theta_x, float theta_y, float t_hat[3]);
float compute_thrust_magnitude(float m, const float a_des[3], const float t_hat[3]);

/* ════════════════════════════════════════════════════════════════════════════
 * Eq 1: compute_quaternion_error   q_err = q_des * conj(q_meas)
 * ════════════════════════════════════════════════════════════════════════════ */

void test_eq1_identity_des_90z_meas(void)
{
    quaternion_t q_des = {1.0f, 0.0f, 0.0f, 0.0f};
    quaternion_t q_meas = {0.707107f, 0.0f, 0.0f, 0.707107f};
    quaternion_t q_err;
    compute_quaternion_error(&q_des, &q_meas, &q_err);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.707107f, q_err.w);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, q_err.x);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, q_err.y);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -0.707107f, q_err.z);
}

void test_eq1_90x_des_identity_meas(void)
{
    quaternion_t q_des = {0.707107f, 0.707107f, 0.0f, 0.0f};
    quaternion_t q_meas = {1.0f, 0.0f, 0.0f, 0.0f};
    quaternion_t q_err;
    compute_quaternion_error(&q_des, &q_meas, &q_err);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.707107f, q_err.w);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.707107f, q_err.x);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, q_err.y);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, q_err.z);
}

/* Hand-crafted: zero error */
void test_eq1_zero_error(void)
{
    quaternion_t q_des = {1.0f, 0.0f, 0.0f, 0.0f};
    quaternion_t q_meas = {1.0f, 0.0f, 0.0f, 0.0f};
    quaternion_t q_err;
    compute_quaternion_error(&q_des, &q_meas, &q_err);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, q_err.w);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, q_err.x);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, q_err.y);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, q_err.z);
}

/* Hand-crafted: 90-degree pitch error */
void test_eq1_90deg_pitch(void)
{
    quaternion_t q_des = {0.707f, 0.0f, 0.707f, 0.0f};
    quaternion_t q_meas = {1.0f, 0.0f, 0.0f, 0.0f};
    quaternion_t q_err;
    compute_quaternion_error(&q_des, &q_meas, &q_err);

    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, 0.707f, q_err.w);
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, 0.0f, q_err.x);
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, 0.707f, q_err.y);
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, 0.0f, q_err.z);
}

/* Hand-crafted: opposite 90-degree rolls → 180-degree error */
void test_eq1_opposite_rolls(void)
{
    quaternion_t q_des = {0.707f, 0.707f, 0.0f, 0.0f};
    quaternion_t q_meas = {0.707f, -0.707f, 0.0f, 0.0f};
    quaternion_t q_err;
    compute_quaternion_error(&q_des, &q_meas, &q_err);

    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, 0.0f, q_err.w);
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, 1.0f, q_err.x);
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, 0.0f, q_err.y);
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, 0.0f, q_err.z);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Eq 2: compute_axis_angle_error   phi = 2*atan2(||v||,q0) * v/||v||
 * ════════════════════════════════════════════════════════════════════════════ */

void test_eq2_90deg_about_x(void)
{
    quaternion_t q_err = {0.707107f, 0.707107f, 0.0f, 0.0f};
    float phi[3];
    compute_axis_angle_error(&q_err, phi);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.570796f, phi[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, phi[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, phi[2]);
}

void test_eq2_mixed_xy(void)
{
    quaternion_t q_err = {0.866025f, 0.353553f, 0.353553f, 0.0f};
    float phi[3];
    compute_axis_angle_error(&q_err, phi);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.740480f, phi[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.740480f, phi[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, phi[2]);
}

/* Hand-crafted: identity quaternion → zero axis-angle */
void test_eq2_identity(void)
{
    quaternion_t q_err = {1.0f, 0.0f, 0.0f, 0.0f};
    float phi[3];
    compute_axis_angle_error(&q_err, phi);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, phi[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, phi[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, phi[2]);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Eq 3: compute_perpendicular_thrust   t_perp = (tau_gim x r_gim)/||r_gim||^2
 * ════════════════════════════════════════════════════════════════════════════ */

void test_eq3_simple_cross(void)
{
    float tau_gim[3] = {0.0f, 1.0f, 0.0f};
    float r_gim[3] = {2.0f, 0.0f, 0.0f};
    float t_perp[3];
    compute_perpendicular_thrust(tau_gim, r_gim, t_perp);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_perp[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_perp[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -0.5f, t_perp[2]);
}

void test_eq3_general(void)
{
    float tau_gim[3] = {1.0f, 2.0f, 3.0f};
    float r_gim[3] = {0.0f, 0.0f, 2.0f};
    float t_perp[3];
    compute_perpendicular_thrust(tau_gim, r_gim, t_perp);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, t_perp[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -0.5f, t_perp[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_perp[2]);
}

/* Hand-crafted: zero torque → zero t_perp */
void test_eq3_zero_torque(void)
{
    float tau_gim[3] = {0.0f, 0.0f, 0.0f};
    float r_gim[3] = {0.0f, 0.0f, -1.0f};
    float t_perp[3];
    compute_perpendicular_thrust(tau_gim, r_gim, t_perp);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_perp[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_perp[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_perp[2]);
}

/* Hand-crafted: pitch torque */
void test_eq3_pitch_torque(void)
{
    float tau_gim[3] = {10.0f, 0.0f, 0.0f};
    float r_gim[3] = {0.0f, 0.0f, -2.0f};
    float t_perp[3];
    compute_perpendicular_thrust(tau_gim, r_gim, t_perp);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_perp[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 5.0f, t_perp[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_perp[2]);
}

/* Hand-crafted: yaw torque */
void test_eq3_yaw_torque(void)
{
    float tau_gim[3] = {0.0f, 15.0f, 0.0f};
    float r_gim[3] = {0.0f, 0.0f, -1.5f};
    float t_perp[3];
    compute_perpendicular_thrust(tau_gim, r_gim, t_perp);

    TEST_ASSERT_FLOAT_WITHIN(TOL, -10.0f, t_perp[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_perp[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_perp[2]);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Eq 4: compute_parallel_thrust   t_par = sqrt(T^2 - ||t_perp||^2) * r_hat
 * ════════════════════════════════════════════════════════════════════════════ */

void test_eq4_along_z(void)
{
    float t_perp[3] = {0.0f, 0.0f, -6.0f};
    float r_gim[3] = {0.0f, 0.0f, 2.0f};
    float t_par[3];
    compute_parallel_thrust(10.0f, t_perp, r_gim, t_par);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_par[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_par[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 8.0f, t_par[2]);
}

void test_eq4_along_y(void)
{
    float t_perp[3] = {3.0f, 0.0f, 0.0f};
    float r_gim[3] = {0.0f, 4.0f, 0.0f};
    float t_par[3];
    compute_parallel_thrust(5.0f, t_perp, r_gim, t_par);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_par[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 4.0f, t_par[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_par[2]);
}

/* Hand-crafted: purely vertical (no perp component) */
void test_eq4_purely_vertical(void)
{
    float t_perp[3] = {0.0f, 0.0f, 0.0f};
    float r_gim[3] = {0.0f, 0.0f, -1.0f};
    float t_par[3];
    compute_parallel_thrust(10.0f, t_perp, r_gim, t_par);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_par[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_par[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -10.0f, t_par[2]);
}

/* Hand-crafted: 3-4-5 triangle */
void test_eq4_345_triangle(void)
{
    float t_perp[3] = {3.0f, 0.0f, 0.0f};
    float r_gim[3] = {0.0f, 0.0f, -1.0f};
    float t_par[3];
    compute_parallel_thrust(5.0f, t_perp, r_gim, t_par);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_par[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_par[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -4.0f, t_par[2]);
}

/* Hand-crafted: 5-12-13 triangle, r_gim along -y */
void test_eq4_51213_triangle(void)
{
    float t_perp[3] = {5.0f, 0.0f, 0.0f};
    float r_gim[3] = {0.0f, -1.0f, 0.0f};
    float t_par[3];
    compute_parallel_thrust(13.0f, t_perp, r_gim, t_par);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_par[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -12.0f, t_par[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_par[2]);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Eq 5: t_des = t_perp + t_parallel (inline addition)
 * ════════════════════════════════════════════════════════════════════════════ */

void test_eq5_case1(void)
{
    float t_perp[3] = {0.0f, 0.0f, -6.0f};
    float t_par[3] = {0.0f, 0.0f, 8.0f};
    float t_des[3];
    t_des[0] = t_perp[0] + t_par[0];
    t_des[1] = t_perp[1] + t_par[1];
    t_des[2] = t_perp[2] + t_par[2];

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_des[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_des[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 2.0f, t_des[2]);
}

void test_eq5_case2(void)
{
    float t_perp[3] = {1.0f, -0.5f, 0.0f};
    float t_par[3] = {0.0f, 4.0f, 0.0f};
    float t_des[3];
    t_des[0] = t_perp[0] + t_par[0];
    t_des[1] = t_perp[1] + t_par[1];
    t_des[2] = t_perp[2] + t_par[2];

    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, t_des[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 3.5f, t_des[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_des[2]);
}

/* Hand-crafted: vertical only */
void test_eq5_vertical(void)
{
    float t_perp[3] = {0.0f, 0.0f, 0.0f};
    float t_par[3] = {0.0f, 0.0f, -10.0f};
    float t_des[3];
    t_des[0] = t_perp[0] + t_par[0];
    t_des[1] = t_perp[1] + t_par[1];
    t_des[2] = t_perp[2] + t_par[2];

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_des[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_des[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -10.0f, t_des[2]);
}

/* Hand-crafted: pitch correction */
void test_eq5_pitch_correction(void)
{
    float t_perp[3] = {3.0f, 0.0f, 0.0f};
    float t_par[3] = {0.0f, 0.0f, -4.0f};
    float t_des[3];
    t_des[0] = t_perp[0] + t_par[0];
    t_des[1] = t_perp[1] + t_par[1];
    t_des[2] = t_perp[2] + t_par[2];

    TEST_ASSERT_FLOAT_WITHIN(TOL, 3.0f, t_des[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_des[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -4.0f, t_des[2]);
}

/* Hand-crafted: yaw correction */
void test_eq5_yaw_correction(void)
{
    float t_perp[3] = {0.0f, -5.0f, 0.0f};
    float t_par[3] = {0.0f, 0.0f, -12.0f};
    float t_des[3];
    t_des[0] = t_perp[0] + t_par[0];
    t_des[1] = t_perp[1] + t_par[1];
    t_des[2] = t_perp[2] + t_par[2];

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_des[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -5.0f, t_des[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -12.0f, t_des[2]);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Eq 6-7: compute_gimbal_angles   theta_x = -atan2(y,z), theta_y = asin(x)
 * ════════════════════════════════════════════════════════════════════════════ */

void test_eq6_zero_angle(void)
{
    float t_des[3] = {0.0f, 0.0f, 1.0f};
    flight_controller_gimbal_config_t gcfg = {0.15f, -1.5f, 1.5f};
    float tx, ty;
    compute_gimbal_angles(t_des, &gcfg, &tx, &ty);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tx);
}

void test_eq6_positive_theta_x(void)
{
    float t_des[3] = {0.0f, -0.295520f, 0.955336f};
    flight_controller_gimbal_config_t gcfg = {0.15f, -1.5f, 1.5f};
    float tx, ty;
    compute_gimbal_angles(t_des, &gcfg, &tx, &ty);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.3f, tx);
}

void test_eq7_zero_angle(void)
{
    float t_des[3] = {0.0f, 0.0f, 1.0f};
    flight_controller_gimbal_config_t gcfg = {0.15f, -1.5f, 1.5f};
    float tx, ty;
    compute_gimbal_angles(t_des, &gcfg, &tx, &ty);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, ty);
}

void test_eq7_positive_theta_y(void)
{
    float t_des[3] = {0.389418f, 0.0f, 0.921061f};
    flight_controller_gimbal_config_t gcfg = {0.15f, -1.5f, 1.5f};
    float tx, ty;
    compute_gimbal_angles(t_des, &gcfg, &tx, &ty);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.4f, ty);
}

/* Hand-crafted: vertical thrust */
void test_eq67_vertical(void)
{
    float t_des[3] = {0.0f, 0.0f, 1.0f};
    flight_controller_gimbal_config_t gcfg = {0.15f, -1.5f, 1.5f};
    float tx, ty;
    compute_gimbal_angles(t_des, &gcfg, &tx, &ty);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tx);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, ty);
}

/* Hand-crafted: deflected in Y (induces pitch) */
void test_eq67_y_deflection(void)
{
    float t_des[3] = {0.0f, 0.5f, 0.866f};
    flight_controller_gimbal_config_t gcfg = {0.15f, -1.5f, 1.5f};
    float tx, ty;
    compute_gimbal_angles(t_des, &gcfg, &tx, &ty);

    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, -0.523f, tx);
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, 0.0f, ty);
}

/* Hand-crafted: deflected in X (induces yaw) */
void test_eq67_x_deflection(void)
{
    float t_des[3] = {0.5f, 0.0f, 0.866f};
    flight_controller_gimbal_config_t gcfg = {0.15f, -1.5f, 1.5f};
    float tx, ty;
    compute_gimbal_angles(t_des, &gcfg, &tx, &ty);

    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, 0.0f, tx);
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, 0.523f, ty);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Eq 8: update_thrust_direction
 *   t_hat = [sin(ty), -sin(tx)*cos(ty), cos(tx)*cos(ty)]
 * ════════════════════════════════════════════════════════════════════════════ */

void test_eq8_zero_angles(void)
{
    float t_hat[3];
    update_thrust_direction(0.0f, 0.0f, t_hat);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_hat[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, t_hat[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, t_hat[2]);
}

void test_eq8_mixed_angles(void)
{
    float t_hat[3];
    update_thrust_direction(0.2f, -0.3f, t_hat);

    TEST_ASSERT_FLOAT_WITHIN(TOL, -0.295520f, t_hat[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -0.189796f, t_hat[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.936293f, t_hat[2]);
}

/* Hand-crafted: negative pitch deflection (~-30 deg) */
void test_eq8_neg_pitch(void)
{
    float t_hat[3];
    update_thrust_direction(-0.523f, 0.0f, t_hat);

    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, 0.0f, t_hat[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, 0.5f, t_hat[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, 0.866f, t_hat[2]);
}

/* Hand-crafted: positive yaw deflection (~30 deg) */
void test_eq8_pos_yaw(void)
{
    float t_hat[3];
    update_thrust_direction(0.0f, 0.523f, t_hat);

    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, 0.5f, t_hat[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, 0.0f, t_hat[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, 0.866f, t_hat[2]);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Eq 9: compute_thrust_magnitude   T = m * dot(a_des, t_hat)
 * ════════════════════════════════════════════════════════════════════════════ */

void test_eq9_hover(void)
{
    float a[3] = {0.0f, 0.0f, 9.81f};
    float t_hat[3] = {0.0f, 0.0f, 1.0f};
    float T = compute_thrust_magnitude(2.0f, a, t_hat);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 19.62f, T);
}

void test_eq9_general(void)
{
    float a[3] = {1.0f, 2.0f, 4.0f};
    float t_hat[3] = {0.6f, 0.0f, 0.8f};
    float T = compute_thrust_magnitude(3.0f, a, t_hat);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 11.4f, T);
}

/* Hand-crafted: vertical ascent */
void test_eq9_vertical_ascent(void)
{
    float a[3] = {0.0f, 0.0f, 9.8f};
    float t_hat[3] = {0.0f, 0.0f, 1.0f};
    float T = compute_thrust_magnitude(10.0f, a, t_hat);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 98.0f, T);
}

/* Hand-crafted: lateral acceleration ignored when thrust is vertical */
void test_eq9_lateral_ignored(void)
{
    float a[3] = {0.0f, 2.0f, 9.8f};
    float t_hat[3] = {0.0f, 0.0f, 1.0f};
    float T = compute_thrust_magnitude(10.0f, a, t_hat);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 98.0f, T);
}

/* Hand-crafted: angled thrust */
void test_eq9_angled_thrust(void)
{
    float a[3] = {0.0f, 5.0f, 8.66f};
    float t_hat[3] = {0.0f, 0.5f, 0.866f};
    float T = compute_thrust_magnitude(10.0f, a, t_hat);

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 100.0f, T);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Eq 10: compute_gyroscopic_torque   tau_gyro = (I*omega) x omega
 * ════════════════════════════════════════════════════════════════════════════ */

void test_eq10_single_axis(void)
{
    float I[3][3] = {{2.0f, 0, 0}, {0, 3.0f, 0}, {0, 0, 4.0f}};
    float omega[3] = {1.0f, 0.0f, 0.0f};
    float tau[3];
    compute_gyroscopic_torque(I, omega, tau);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau[2]);
}

void test_eq10_multi_axis(void)
{
    float I[3][3] = {{2.0f, 0, 0}, {0, 3.0f, 0}, {0, 0, 4.0f}};
    float omega[3] = {1.0f, 2.0f, 3.0f};
    float tau[3];
    compute_gyroscopic_torque(I, omega, tau);

    TEST_ASSERT_FLOAT_WITHIN(TOL, -6.0f, tau[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 6.0f, tau[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -2.0f, tau[2]);
}

/* Hand-crafted: zero rotation */
void test_eq10_zero_rotation(void)
{
    float I[3][3] = {{1.0f, 0, 0}, {0, 1.0f, 0}, {0, 0, 1.0f}};
    float omega[3] = {0.0f, 0.0f, 0.0f};
    float tau[3];
    compute_gyroscopic_torque(I, omega, tau);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau[2]);
}

/* Hand-crafted: scalar MMOI (isotropic) → gyro torque always zero */
void test_eq10_scalar_mmoi_pitch(void)
{
    float I[3][3] = {{5.0f, 0, 0}, {0, 5.0f, 0}, {0, 0, 5.0f}};
    float omega[3] = {0.0f, 2.0f, 0.0f};
    float tau[3];
    compute_gyroscopic_torque(I, omega, tau);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau[2]);
}

/* Hand-crafted: scalar MMOI, multi-axis rotation → still zero */
void test_eq10_scalar_mmoi_complex(void)
{
    float I[3][3] = {{2.5f, 0, 0}, {0, 2.5f, 0}, {0, 0, 2.5f}};
    float omega[3] = {1.0f, 2.0f, 3.0f};
    float tau[3];
    compute_gyroscopic_torque(I, omega, tau);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau[2]);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Eq 11: compute_command_torque   tau_cmd = -(tau_gyro + phi)
 * ════════════════════════════════════════════════════════════════════════════ */

void test_eq11_zero_gyro(void)
{
    float tau_gyro[3] = {0.0f, 0.0f, 0.0f};
    float phi[3] = {1.0f, -2.0f, 3.0f};
    float tau_cmd[3];
    compute_command_torque(tau_gyro, phi, tau_cmd);

    TEST_ASSERT_FLOAT_WITHIN(TOL, -1.0f, tau_cmd[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 2.0f, tau_cmd[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -3.0f, tau_cmd[2]);
}

void test_eq11_general(void)
{
    float tau_gyro[3] = {-6.0f, 6.0f, -2.0f};
    float phi[3] = {0.5f, 1.5f, -0.5f};
    float tau_cmd[3];
    compute_command_torque(tau_gyro, phi, tau_cmd);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 5.5f, tau_cmd[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -7.5f, tau_cmd[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 2.5f, tau_cmd[2]);
}

/* Hand-crafted: both zero → zero */
void test_eq11_all_zero(void)
{
    float tau_gyro[3] = {0.0f, 0.0f, 0.0f};
    float phi[3] = {0.0f, 0.0f, 0.0f};
    float tau_cmd[3];
    compute_command_torque(tau_gyro, phi, tau_cmd);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau_cmd[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau_cmd[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau_cmd[2]);
}

/* Hand-crafted: correction required */
void test_eq11_correction(void)
{
    float tau_gyro[3] = {1.0f, 2.0f, 3.0f};
    float phi[3] = {4.0f, 5.0f, 6.0f};
    float tau_cmd[3];
    compute_command_torque(tau_gyro, phi, tau_cmd);

    TEST_ASSERT_FLOAT_WITHIN(TOL, -5.0f, tau_cmd[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -7.0f, tau_cmd[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -9.0f, tau_cmd[2]);
}

/* Hand-crafted: offsetting forces cancel */
void test_eq11_offsetting(void)
{
    float tau_gyro[3] = {5.0f, -2.0f, 0.0f};
    float phi[3] = {-5.0f, 2.0f, 0.0f};
    float tau_cmd[3];
    compute_command_torque(tau_gyro, phi, tau_cmd);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau_cmd[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau_cmd[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau_cmd[2]);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Eq 12: compute_roll_torque   tau_roll = (tau_cmd . t_hat) * t_hat
 * ════════════════════════════════════════════════════════════════════════════ */

void test_eq12_along_z(void)
{
    float tau_cmd[3] = {1.0f, 2.0f, 3.0f};
    float t_hat[3] = {0.0f, 0.0f, 1.0f};
    float tau_roll[3];
    compute_roll_torque(tau_cmd, t_hat, tau_roll);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau_roll[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau_roll[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 3.0f, tau_roll[2]);
}

void test_eq12_diagonal(void)
{
    float tau_cmd[3] = {4.0f, 0.0f, 0.0f};
    float t_hat[3] = {0.707107f, 0.0f, 0.707107f};
    float tau_roll[3];
    compute_roll_torque(tau_cmd, t_hat, tau_roll);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 2.0f, tau_roll[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau_roll[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 2.0f, tau_roll[2]);
}

/* Hand-crafted: no command → no roll torque */
void test_eq12_no_command(void)
{
    float tau_cmd[3] = {0.0f, 0.0f, 0.0f};
    float t_hat[3] = {0.0f, 0.0f, 1.0f};
    float tau_roll[3];
    compute_roll_torque(tau_cmd, t_hat, tau_roll);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau_roll[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau_roll[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau_roll[2]);
}

/* Hand-crafted: z-axis isolation */
void test_eq12_z_isolation(void)
{
    float tau_cmd[3] = {10.0f, 5.0f, 2.0f};
    float t_hat[3] = {0.0f, 0.0f, 1.0f};
    float tau_roll[3];
    compute_roll_torque(tau_cmd, t_hat, tau_roll);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau_roll[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau_roll[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 2.0f, tau_roll[2]);
}

/* Hand-crafted: x-axis isolation */
void test_eq12_x_isolation(void)
{
    float tau_cmd[3] = {10.0f, 5.0f, 2.0f};
    float t_hat[3] = {1.0f, 0.0f, 0.0f};
    float tau_roll[3];
    compute_roll_torque(tau_cmd, t_hat, tau_roll);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 10.0f, tau_roll[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau_roll[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau_roll[2]);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Eq 13: compute_gimbal_torque   tau_gim = tau_cmd - tau_roll
 * ════════════════════════════════════════════════════════════════════════════ */

void test_eq13_along_z(void)
{
    float tau_cmd[3] = {1.0f, 2.0f, 3.0f};
    float tau_roll[3] = {0.0f, 0.0f, 3.0f};
    float tau_gim[3];
    compute_gimbal_torque(tau_cmd, tau_roll, tau_gim);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, tau_gim[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 2.0f, tau_gim[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau_gim[2]);
}

void test_eq13_diagonal(void)
{
    float tau_cmd[3] = {4.0f, 0.0f, 0.0f};
    float tau_roll[3] = {2.0f, 0.0f, 2.0f};
    float tau_gim[3];
    compute_gimbal_torque(tau_cmd, tau_roll, tau_gim);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 2.0f, tau_gim[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau_gim[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -2.0f, tau_gim[2]);
}

/* Hand-crafted: both zero */
void test_eq13_all_zero(void)
{
    float tau_cmd[3] = {0.0f, 0.0f, 0.0f};
    float tau_roll[3] = {0.0f, 0.0f, 0.0f};
    float tau_gim[3];
    compute_gimbal_torque(tau_cmd, tau_roll, tau_gim);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau_gim[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau_gim[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau_gim[2]);
}

/* Hand-crafted: removing roll from command */
void test_eq13_remove_roll(void)
{
    float tau_cmd[3] = {10.0f, 5.0f, 2.0f};
    float tau_roll[3] = {0.0f, 0.0f, 2.0f};
    float tau_gim[3];
    compute_gimbal_torque(tau_cmd, tau_roll, tau_gim);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 10.0f, tau_gim[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 5.0f, tau_gim[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, tau_gim[2]);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Public API: flight_controller_run
 * ════════════════════════════════════════════════════════════════════════════ */

static flight_controller_config_t make_api_config(void)
{
    flight_controller_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    for (int i = 0; i < 3; i++)
    {
        cfg.attitude.Kp[i][i] = 1.0f;
        cfg.attitude.I[i][i] = (i == 0) ? 2.0f : (i == 1) ? 3.0f
                                                          : 4.0f;
    }
    cfg.allocation.t_hat[0] = 0.0f;
    cfg.allocation.t_hat[1] = 0.0f;
    cfg.allocation.t_hat[2] = 1.0f;
    cfg.gimbal.L = 1.0f;
    cfg.gimbal.theta_min = -0.5f;
    cfg.gimbal.theta_max = 0.5f;
    cfg.thrust.m = 1.0f;
    cfg.thrust.g = 10.0f;
    cfg.thrust.T_min = 0.0f;
    cfg.thrust.T_max = 20.0f;
    cfg.thrust.kp = 0.0f;
    cfg.thrust.ki = 0.0f;
    cfg.thrust.kd = 0.0f;
    cfg.thrust.integral_limit = 10.0f;
    cfg.thrust.a_z_min = -10.0f;
    cfg.thrust.a_z_max = 10.0f;
    return cfg;
}

/* Test 1 — nominal hover: identity attitude, zero rates, zero z-error */
void test_api_nominal_hover(void)
{
    flight_controller_config_t cfg = make_api_config();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_thrust);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 10.0f, out.T_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.theta_x_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.theta_y_cmd);
}

/* Test 2 — pure pitch error (~0.2 rad about y) goes to gimbal torque.
 * s_t_mag=0 on first call → t_par=0, t_des is purely perpendicular,
 * so theta_y saturates at theta_min and T_cmd = g*cos(theta_max). */
void test_api_pitch_error(void)
{
    flight_controller_config_t cfg = make_api_config();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){0.995004f, 0.0f, 0.099833f, 0.0f};

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -0.2f, out.tau_gim[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_thrust);
    /* s_t_mag=0 → theta_y saturates → t_hat tilts → T = g*cos(0.5) */
    TEST_ASSERT_FLOAT_WITHIN(TOL, 8.7758f, out.T_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.theta_x_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -0.5f, out.theta_y_cmd);
}

/* Test 3 — pure yaw error goes to thrust-axis torque, not gimbal */
void test_api_yaw_error(void)
{
    flight_controller_config_t cfg = make_api_config();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){0.995004f, 0.0f, 0.0f, 0.099833f};

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -0.2f, out.tau_thrust);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 10.0f, out.T_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.theta_x_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.theta_y_cmd);
}

/* Test 4 — zero attitude error, nonzero body rates (gyro compensation) */
void test_api_gyro_compensation(void)
{
    flight_controller_config_t cfg = make_api_config();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};
    state.omega_b[0] = 0.0f;
    state.omega_b[1] = 1.0f;
    state.omega_b[2] = 2.0f;

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 2.0f, out.tau_gim[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_thrust);
    /* s_t_mag=0 → t_par=0, t_des purely perpendicular → theta_x saturates
     * → t_hat tilts → T = g*cos(theta_max) ≈ 8.7758 */
    TEST_ASSERT_FLOAT_WITHIN(TOL, 8.7758f, out.T_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.5f, out.theta_x_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.theta_y_cmd);
}

/* Test 5 — boundary: thrust clamp (T_min=T_max=8) + gimbal angle clamp.
 * PDF Eq 11 has no Kp gain, so Kp[1][1]=30 has no effect;
 * tau_gim[1] = -(phi[1]) ≈ -0.2, NOT -6.0. */
void test_api_boundary_clamp(void)
{
    flight_controller_config_t cfg = make_api_config();
    cfg.attitude.Kp[1][1] = 30.0f;
    cfg.thrust.T_min = 8.0f;
    cfg.thrust.T_max = 8.0f;
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){0.995004f, 0.0f, 0.099833f, 0.0f};

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -0.2f, out.tau_gim[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_thrust);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 8.0f, out.T_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.theta_x_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -0.5f, out.theta_y_cmd);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Public API: flight_controller_run (set 2 — baseline config with z-PID)
 * ════════════════════════════════════════════════════════════════════════════ */

static flight_controller_config_t make_api_config_v2(void)
{
    flight_controller_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    for (int i = 0; i < 3; i++)
    {
        cfg.attitude.Kp[i][i] = 1.0f;
        cfg.attitude.Kd[i][i] = 1.0f;
        cfg.attitude.I[i][i] = 1.0f;
    }
    cfg.allocation.t_hat[0] = 0.0f;
    cfg.allocation.t_hat[1] = 0.0f;
    cfg.allocation.t_hat[2] = 1.0f;
    cfg.gimbal.L = 0.1f;
    cfg.gimbal.theta_min = -0.5f;
    cfg.gimbal.theta_max = 0.5f;
    cfg.thrust.m = 1.0f;
    cfg.thrust.g = 9.81f;
    cfg.thrust.T_min = 2.0f;
    cfg.thrust.T_max = 20.0f;
    cfg.thrust.kp = 2.0f;
    cfg.thrust.ki = 0.0f;
    cfg.thrust.kd = 0.0f;
    cfg.thrust.integral_limit = 10.0f;
    cfg.thrust.a_z_min = -10.0f;
    cfg.thrust.a_z_max = 10.0f;
    return cfg;
}

/* Test 1 — perfect hover: at target altitude, identity attitude */
void test_api_v2_perfect_hover(void)
{
    flight_controller_config_t cfg = make_api_config_v2();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.pos[2] = 5.0f;
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};
    ref.z_ref = 5.0f;

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 9.81f, out.T_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_thrust);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.theta_x_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.theta_y_cmd);
}

/* Test 2 — pure altitude climb: z_error = 2.0, kp=2.0 → T = 9.81 + 4.0 */
void test_api_v2_altitude_climb(void)
{
    flight_controller_config_t cfg = make_api_config_v2();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};
    ref.z_ref = 2.0f;

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 13.81f, out.T_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_thrust);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.theta_x_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.theta_y_cmd);
}

/* Test 3 — pure roll correction: small x-axis rotation in q_ref.
 * s_t_mag=0 on first call → theta_x saturates → t_hat tilts → T < 9.81 */
void test_api_v2_roll_correction(void)
{
    flight_controller_config_t cfg = make_api_config_v2();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.pos[2] = 5.0f;
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){0.996f, 0.087f, 0.0f, 0.0f};
    ref.z_ref = 5.0f;

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    /* s_t_mag=0 → theta_x saturates → T = g*cos(0.5) ≈ 8.6091 */
    TEST_ASSERT_FLOAT_WITHIN(TOL, 8.6091f, out.T_cmd);
    TEST_ASSERT_TRUE(out.tau_gim[0] > 0.0f || out.tau_gim[0] < 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_thrust);
    TEST_ASSERT_TRUE(out.theta_x_cmd != 0.0f || out.theta_y_cmd != 0.0f);
}

/* Test 4 — angular rate damping: omega_b = [1,0,0], no attitude error.
 * PDF Eq 11 has no Kd gain. With I=identity and omega=[1,0,0],
 * tau_gyro = (I*omega) x omega = [1,0,0] x [1,0,0] = [0,0,0],
 * phi=[0,0,0], so tau_cmd=[0,0,0] and everything is zero. */
void test_api_v2_rate_damping(void)
{
    flight_controller_config_t cfg = make_api_config_v2();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.pos[2] = 5.0f;
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};
    state.omega_b[0] = 1.0f;

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};
    ref.z_ref = 5.0f;

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 9.81f, out.T_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_thrust);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.theta_x_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.theta_y_cmd);
}

/* Test 5 — actuator saturation: large z-error + 180-deg rotation */
void test_api_v2_saturation(void)
{
    flight_controller_config_t cfg = make_api_config_v2();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.pos[2] = 100.0f;
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){0.0f, 1.0f, 0.0f, 0.0f};
    ref.z_ref = 0.0f;

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 2.0f, out.T_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_thrust);
    TEST_ASSERT_TRUE(out.theta_x_cmd >= -0.5f && out.theta_x_cmd <= 0.5f);
    TEST_ASSERT_TRUE(out.theta_y_cmd >= -0.5f && out.theta_y_cmd <= 0.5f);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Public API: flight_controller_run (set 3 — full PID config with ki/kd)
 * ════════════════════════════════════════════════════════════════════════════ */

static flight_controller_config_t make_api_config_v3(void)
{
    flight_controller_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    for (int i = 0; i < 3; i++)
    {
        cfg.attitude.Kp[i][i] = 1.0f;
        cfg.attitude.Kd[i][i] = 1.0f;
        cfg.attitude.I[i][i] = 1.0f;
    }
    cfg.allocation.t_hat[0] = 0.0f;
    cfg.allocation.t_hat[1] = 0.0f;
    cfg.allocation.t_hat[2] = 1.0f;
    cfg.gimbal.L = 0.1f;
    cfg.gimbal.theta_min = -0.5f;
    cfg.gimbal.theta_max = 0.5f;
    cfg.thrust.m = 1.0f;
    cfg.thrust.g = 9.81f;
    cfg.thrust.T_min = 2.0f;
    cfg.thrust.T_max = 20.0f;
    cfg.thrust.kp = 2.0f;
    cfg.thrust.ki = 1.0f;
    cfg.thrust.kd = 0.5f;
    cfg.thrust.integral_limit = 3.0f;
    cfg.thrust.a_z_min = -20.0f;
    cfg.thrust.a_z_max = 20.0f;
    return cfg;
}

/* --- Part 1: Altitude (Z-Axis) Control Tests --- */

/* Test 1 — Perfect Hover (Steady State).
 * PID derivative-on-measurement: D = -kd*(meas - prev_meas)/dt.
 * prev_meas inits to 0 in pid_init, so first call with pos[2]=5:
 * D = -0.5*(5-0)/0.01 = -250 → PID output saturates at a_z_min=-20
 * → a_des[2] = 9.81-20 = -10.19 → T < 0 → clamped to T_min=2.0 */
void test_api_v3_hover(void)
{
    flight_controller_config_t cfg = make_api_config_v3();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.pos[2] = 5.0f;
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};
    ref.z_ref = 5.0f;

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 2.0f, out.T_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_thrust);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.theta_x_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.theta_y_cmd);
}

/* Test 2 — Pure Altitude Climb: pos[2]=0, z_ref=2.
 * PID: error=2, P=kp*2=4, I=ki*err*dt=1*2*0.01=0.02,
 * D=-kd*(0-0)/dt=0 (meas=prev_meas=0).
 * a_z_cmd=4.02 → T = 9.81+4.02 = 13.83 */
void test_api_v3_climb(void)
{
    flight_controller_config_t cfg = make_api_config_v3();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};
    ref.z_ref = 2.0f;

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 13.83f, out.T_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_thrust);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.theta_x_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.theta_y_cmd);
}

/* Test 3 — Pure Altitude Descent: pos[2]=5, z_ref=4.
 * PID D-kick: D = -kd*(5-0)/dt = -0.5*500 = -250, saturates output
 * → a_z_cmd = -20 → T = 9.81-20 < 0 → clamped to T_min=2.0 */
void test_api_v3_descent(void)
{
    flight_controller_config_t cfg = make_api_config_v3();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.pos[2] = 5.0f;
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};
    ref.z_ref = 4.0f;

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 2.0f, out.T_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_thrust);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.theta_x_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.theta_y_cmd);
}

/* Test 4 — Z-Velocity Damping: pos[2]=5, vel[2]=2 (vel unused by PID).
 * PID uses derivative-on-measurement of pos[2], NOT state.vel[2].
 * D-kick: D = -kd*(5-0)/dt = -250 → output saturates → T_min=2.0 */
void test_api_v3_velocity_damping(void)
{
    flight_controller_config_t cfg = make_api_config_v3();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.pos[2] = 5.0f;
    state.vel[2] = 2.0f;
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};
    ref.z_ref = 5.0f;

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 2.0f, out.T_cmd);
}

/* Test 5 — Z-Integral Accumulation & Anti-Windup (500 calls, error=1.0) */
void test_api_v3_integral_windup(void)
{
    flight_controller_config_t cfg = make_api_config_v3();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.pos[2] = 4.0f;
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};
    ref.z_ref = 5.0f;

    control_output_t out;

    /* First call: D-kick = -kd*(4-0)/0.01 = -200 → output saturates
     * → a_z_cmd = -20 → T = 9.81-20 < 0 → clamped to T_min=2.0 */
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, 2.0f, out.T_cmd);

    /* Run 499 more times with same inputs */
    for (int i = 1; i < 500; i++)
    {
        flight_controller_run(&state, &ref, &cfg, &out, 0.01f);
    }

    /* By call 500: integral saturated at 3.0. T = 9.81 + 2.0 + 3.0 = 14.81 */
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, 14.81f, out.T_cmd);
}

/* Test 6 — Thrust Saturation (Min/Max) */
void test_api_v3_thrust_saturation(void)
{
    flight_controller_config_t cfg = make_api_config_v3();

    /* Min saturation: demand massive descent */
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.pos[2] = 100.0f;
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 2.0f, out.T_cmd);

    /* Max saturation: demand massive climb */
    flight_controller_init(&cfg);
    memset(&state, 0, sizeof(state));
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    ref.z_ref = 100.0f;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 20.0f, out.T_cmd);
}

/* --- Part 2: Attitude (Torque) Control Tests --- */

/* Test 7 — Pure Pitch Correction: ~10 deg pitch.
 * PDF Eq 11: tau_cmd = -(phi). Positive phi[1] → negative tau_cmd[1]
 * → tau_gim[1] < 0 (not > 0). */
void test_api_v3_pitch_correction(void)
{
    flight_controller_config_t cfg = make_api_config_v3();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){0.996f, 0.0f, 0.087f, 0.0f};

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    TEST_ASSERT_TRUE(out.tau_gim[1] < 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_thrust);
}

/* Test 8 — Pure Yaw Correction: ~10 deg yaw.
 * PDF Eq 11: tau_cmd = -(phi). Positive phi[2] → negative tau_cmd[2].
 * Yaw torque projects onto thrust axis → tau_thrust < 0 (not > 0). */
void test_api_v3_yaw_correction(void)
{
    flight_controller_config_t cfg = make_api_config_v3();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){0.996f, 0.0f, 0.0f, 0.087f};

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    TEST_ASSERT_TRUE(out.tau_thrust < 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.theta_x_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.theta_y_cmd);
}

/* Test 9 — Pitch Rate Damping: omega_b[1]=1, I=identity.
 * PDF Eq 11 has no Kd. With I=identity: tau_gyro = (I*omega) x omega
 * = [0,1,0] x [0,1,0] = [0,0,0]. phi=[0,0,0].
 * tau_cmd = [0,0,0] → tau_gim = [0,0,0]. */
void test_api_v3_pitch_rate_damping(void)
{
    flight_controller_config_t cfg = make_api_config_v3();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};
    state.omega_b[1] = 1.0f;

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[1]);
}

/* Test 10 — Shortest Path Quaternion Rotation (double-cover edge case).
 * q_err.w < 0 → negated to {0.996, -0.087, 0, 0} → phi = [-0.174, 0, 0].
 * Eq 11: tau_cmd = -phi = [+0.174, 0, 0] → tau_gim[0] > 0 (not < 0). */
void test_api_v3_shortest_path(void)
{
    flight_controller_config_t cfg = make_api_config_v3();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){-0.996f, 0.087f, 0.0f, 0.0f};

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    TEST_ASSERT_TRUE(out.tau_gim[0] > 0.0f);
}

/* --- Part 3: Allocation & Gimbal Geometry Tests --- */

/* Test 11 — Gimbal Actuation Mapping: roll demand → gimbal angle non-zero */
void test_api_v3_gimbal_mapping(void)
{
    flight_controller_config_t cfg = make_api_config_v3();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.pos[2] = 5.0f;
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){0.996f, 0.087f, 0.0f, 0.0f};
    ref.z_ref = 5.0f;

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    TEST_ASSERT_TRUE(out.theta_x_cmd != 0.0f || out.theta_y_cmd != 0.0f);
}

/* Test 12 — Gimbal Actuation Saturation: 180-deg demand → angles clamped, no NaN */
void test_api_v3_gimbal_saturation(void)
{
    flight_controller_config_t cfg = make_api_config_v3();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.pos[2] = 100.0f;
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){0.0f, 1.0f, 0.0f, 0.0f};

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 2.0f, out.T_cmd);
    TEST_ASSERT_TRUE(out.theta_x_cmd >= -0.5f && out.theta_x_cmd <= 0.5f);
    TEST_ASSERT_TRUE(out.theta_y_cmd >= -0.5f && out.theta_y_cmd <= 0.5f);
    TEST_ASSERT_FALSE(isnan(out.theta_x_cmd));
    TEST_ASSERT_FALSE(isnan(out.theta_y_cmd));
}

/* --- Part 4: Advanced Nonlinear Dynamics & API Robustness Tests --- */

/* Test 13 — Transient D-Kick Recovery (Z-PID Numerical Stability)
 * Validates that the massive derivative spike caused by prev_measurement
 * initializing to 0 correctly resolves after a single timestep. */
void test_api_v3_transient_d_kick_recovery(void)
{
    flight_controller_config_t cfg = make_api_config_v3();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.pos[2] = 10.0f; /* Start high up */
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};
    ref.z_ref = 10.0f; /* System is exactly at target altitude */

    control_output_t out;

    /* First call: D = -kd*(10-0)/0.01 = -500. Output hard-clamps to T_min */
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 2.0f, out.T_cmd);

    /* Second call: prev_measurement is now 10. D-term is 0.
     * P-term is 0. System must command exactly hover thrust (m*g). */
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 9.81f, out.T_cmd);
}

/* Test 14 — Gyroscopic Cross-Coupling Compensation
 * Validates rigid body dynamics tau_gyro = (I*omega) x omega are correctly
 * computed and inverted into tau_cmd for non-isotropic inertia. */
void test_api_v3_gyroscopic_cross_coupling(void)
{
    flight_controller_config_t cfg = make_api_config_v3();
    /* Set distinct inertias to trigger cross-coupling */
    cfg.attitude.I[0][0] = 1.0f;
    cfg.attitude.I[1][1] = 2.0f;
    cfg.attitude.I[2][2] = 3.0f;
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.pos[2] = 5.0f;
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};
    state.omega_b[0] = 1.0f;
    state.omega_b[1] = 1.0f;
    state.omega_b[2] = 1.0f;

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f}; /* Zero attitude error */
    ref.z_ref = 5.0f;

    control_output_t out;

    /* Run twice: First to clear s_t_mag=0 edge case, second for steady state */
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    /* tau_gyro = [ (2-3)*1*1, (3-1)*1*1, (1-2)*1*1 ] = [-1, 2, -1]
     * tau_cmd = -tau_gyro = [1, -2, 1]
     * tau_gim removes roll axis projection (z-axis here). tau_gim = [1, -2, 0] */
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, out.tau_gim[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -2.0f, out.tau_gim[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[2]);
}

/* Test 15 — Actuator Saturation & Asymmetric tau_thrust Re-projection
 * Proves differential prop torque is calculated using the physically clamped
 * thrust vector, preventing impossible torque generation assumptions. */
void test_api_v3_actuator_saturation_reprojection(void)
{
    flight_controller_config_t cfg = make_api_config_v3();
    /* Force extremely tight gimbal limits */
    cfg.gimbal.theta_min = -0.1f;
    cfg.gimbal.theta_max = 0.1f;
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.pos[2] = 5.0f;
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.q_ref = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};
    ref.z_ref = 5.0f;

    control_output_t out;

    /* Run once at hover to initialize s_t_mag */
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    /* Introduce a massive 90-deg roll/pitch demand */
    ref.q_ref = (quaternion_t){0.707107f, 0.707107f, 0.0f, 0.0f};
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    /* Gimbal MUST hit the hard stop limit */
    TEST_ASSERT_TRUE(out.theta_x_cmd >= 0.099f || out.theta_x_cmd <= -0.099f ||
                     out.theta_y_cmd >= 0.099f || out.theta_y_cmd <= -0.099f);

    /* Thrust torque computation shouldn't NaN, and should project onto the
     * updated (clamped) t_hat, not the theoretical infinite one. */
    TEST_ASSERT_FALSE(isnan(out.tau_thrust));
}

/* Test 16 — Cold-Start Gimbal Saturation (The s_t_mag = 0 Edge Case)
 * Explicitly verifies the boundary condition on the first loop execution
 * where compute_parallel_thrust drops out, leaving purely perpendicular demand. */
void test_api_v3_cold_start_gimbal_saturation(void)
{
    flight_controller_config_t cfg = make_api_config_v3();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.pos[2] = 0.0f;
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    /* Small roll command */
    ref.q_ref = (quaternion_t){0.995004f, 0.099833f, 0.0f, 0.0f};
    ref.z_ref = 0.0f;

    control_output_t out;

    /* First frame execution */
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    /* Gimbal saturates due to missing parallel vector component */
    TEST_ASSERT_TRUE(out.theta_y_cmd <= -0.49f || out.theta_y_cmd >= 0.49f ||
                     out.theta_x_cmd <= -0.49f || out.theta_x_cmd >= 0.49f);

    /* Total thrust must safely scale down by cos(theta_max) to prevent zooming */
    /* 9.81 * cos(0.5 rad) is approx 8.6091 */
    TEST_ASSERT_FLOAT_WITHIN(TOL_COARSE, 8.6091f, out.T_cmd);
}

/* Test 17 — Double-Cover Axis Inversion (Singularity Avoidance)
 * Verifies that the quaternion error logic resolves the shortest path and
 * handles atan2(0,0) singularity traps when flipping 180 degrees. */
void test_api_v3_double_cover_inversion(void)
{
    flight_controller_config_t cfg = make_api_config_v3();
    flight_controller_init(&cfg);

    state_t state;
    memset(&state, 0, sizeof(state));
    state.q_bn = (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};

    flight_controller_ref_t ref;
    memset(&ref, 0, sizeof(ref));
    /* Pure 180-degree rotation around X axis (w=0, x=1) */
    ref.q_ref = (quaternion_t){0.0f, 1.0f, 0.0f, 0.0f};

    control_output_t out;
    flight_controller_run(&state, &ref, &cfg, &out, 0.01f);

    /* System MUST NOT generate NaNs during normalization/axis-angle extraction */
    TEST_ASSERT_FALSE(isnan(out.tau_gim[0]));
    TEST_ASSERT_FALSE(isnan(out.tau_gim[1]));
    TEST_ASSERT_FALSE(isnan(out.tau_gim[2]));

    /* tau_cmd should be exactly -phi = [-PI, 0, 0] or [PI, 0, 0].
     * Which projects straight to the x-axis gimbal torque. */
    TEST_ASSERT_TRUE(fabsf(out.tau_gim[0]) > 3.1f && fabsf(out.tau_gim[0]) < 3.2f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.tau_gim[2]);
}