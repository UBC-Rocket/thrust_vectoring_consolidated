#ifndef EKF_H
#define EKF_H

#define STATE_DIM 4
#define UPDATE_COVAR -1

typedef struct {
    float vals[4];            // real, i, j, k (or w, x, y, z)
    float covar[4][4];        // covariance matrix
    int index;
} quaternion_state;

typedef struct {
    float velocity[3];
    float position[3];            // x, y, z
    float covar[6][6];        // covariance matrix
    int index;
} body_state;

typedef struct {
    quaternion_state quaternion;
    body_state body;
    float expected_g[3];
} EKF;

void init_ekf(const float expected_g[3]);

void get_state(float quat[4], float pos[3], float vel[3]);

void tick_ekf_orientation(float deltaTime, float gyro[3], float accel[3]);

void predict_ekf_body(float dt, float accel_nav[3]);
void update_ekf_body(float gps_pos[3]);
void update_ekf_body_baro(float altitude_m);

/* Macro to multiply any given A (r1 x c1) by B (c1 x c2), producing (r1 x c2) */
#define MAT_MUL(A, B, C, r1, c1, c2)                          \
    do {                                                      \
        for (int i = 0; i < (r1); ++i) {                      \
            for (int j = 0; j < (c2); ++j) {                  \
                (C)[i][j] = 0;                                \
                for (int k = 0; k < (c1); ++k) {              \
                    (C)[i][j] += (A)[i][k] * (B)[k][j];       \
                }                                             \
            }                                                 \
        }                                                     \
    } while (0)

/**
 * @brief shared orientation data
 */
typedef struct {
    /** 
     * @brief Quaternion
     * @details Stored in the order [w, x, y, z]
     */
    float quat[4]; // real, i, j, k
    /** 
     * @brief Euler angle in degrees
     * @details Stored as [roll, pitch]
     */
    float euler[2]; // roll, pitch
} orientation_t;

/**
 * @brief returns orientation data in orientation_t
 */
void get_orientation(orientation_t v);

#endif