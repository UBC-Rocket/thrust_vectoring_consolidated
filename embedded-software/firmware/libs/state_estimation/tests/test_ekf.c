#include "unity.h"
#include "state_estimation/ekf.h"
#include <math.h>
#include <string.h>

#define TOL 1e-4f

static void zero_4x4(float m[4][4]) { memset(m, 0, sizeof(float) * 16); }
static void zero_6x6(float m[6][6]) { memset(m, 0, sizeof(float) * 36); }
static void zero_3x3(float m[3][3]) { memset(m, 0, sizeof(float) * 9); }

static void diag_4x4(float m[4][4], float v)
{
    zero_4x4(m);
    for (int i = 0; i < 4; i++) m[i][i] = v;
}

static void diag_6x6(float m[6][6], float v)
{
    zero_6x6(m);
    for (int i = 0; i < 6; i++) m[i][i] = v;
}

static void diag_3x3(float m[3][3], float v)
{
    zero_3x3(m);
    for (int i = 0; i < 3; i++) m[i][i] = v;
}

/* ---------- init_ekf + get_state ---------- */

void test_ekf_init_gives_identity_quaternion(void)
{
    float pn_q[4][4], mn_q[3][3], pn_b[6][6], mn_b[3][3];
    diag_4x4(pn_q, 0.001f);
    diag_3x3(mn_q, 0.01f);
    diag_6x6(pn_b, 0.001f);
    diag_3x3(mn_b, 0.1f);
    float eg[3] = {0, 0, -1};

    init_ekf(pn_q, mn_q, pn_b, mn_b, eg);

    float q[4], pos[3], vel[3];
    get_state(q, pos, vel);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, q[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, q[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, q[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, q[3]);
    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, pos[i]);
        TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, vel[i]);
    }
}

/* ---------- EKF orientation tick smoke test ---------- */

void test_ekf_orientation_tick_stable_at_rest(void)
{
    float pn_q[4][4], mn_q[3][3], pn_b[6][6], mn_b[3][3];
    diag_4x4(pn_q, 0.001f);
    diag_3x3(mn_q, 0.01f);
    diag_6x6(pn_b, 0.001f);
    diag_3x3(mn_b, 0.1f);
    float eg[3] = {0, 0, -1};

    init_ekf(pn_q, mn_q, pn_b, mn_b, eg);

    /* Feed zero gyro and gravity-aligned accel for several ticks */
    float gyro[3] = {0, 0, 0};
    float accel[3] = {0, 0, -1};
    float dt = 0.01f;

    for (int i = 0; i < 100; i++) {
        tick_ekf_orientation(dt, gyro, accel);
    }

    float q[4], pos[3], vel[3];
    get_state(q, pos, vel);

    /* Quaternion should still be near identity */
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, q[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, q[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, q[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, q[3]);

    /* Quaternion should remain unit length */
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, norm);
}

/* ---------- EKF body tick smoke test ---------- */

void test_ekf_body_tick_zero_accel(void)
{
    float pn_q[4][4], mn_q[3][3], pn_b[6][6], mn_b[3][3];
    diag_4x4(pn_q, 0.001f);
    diag_3x3(mn_q, 0.01f);
    diag_6x6(pn_b, 0.001f);
    diag_3x3(mn_b, 0.1f);
    float eg[3] = {0, 0, -1};

    init_ekf(pn_q, mn_q, pn_b, mn_b, eg);

    float accel[3] = {0, 0, 0};
    float gps[3] = {0, 0, 0};
    float dt = 0.01f;

    for (int i = 0; i < 50; i++) {
        tick_ekf_body(dt, accel, gps);
    }

    float q[4], pos[3], vel[3];
    get_state(q, pos, vel);

    /* Position and velocity should stay near zero */
    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, pos[i]);
        TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, vel[i]);
    }
}
