#ifndef EKF_H
#define EKF_H

#define STATE_DIM 4
#define UPDATE_COVAR 10

typedef struct {
    float vals[4];            // real, i, j, k (or w, x, y, z)
    float covar[4][4];        // covariance matrix
    float process[4][4];      // process noise
    float measurement[3][3];  // measurement noise (of accel)
    int index;
} quaternion_state;

typedef struct {
    float velocity[3];
    float position[3];            // x, y, z
    float covar[6][6];        // covariance matrix
    float process[6][6];      // process noise
    float measurement[3][3];  // measurement noise (of gps)
    int index;
} body_state;

typedef struct {
    quaternion_state quaternion;  
    body_state body;  
    float expected_g[3];               
} EKF;

void init_ekf_body(
    float process_noise[6][6],
    float measurement_noise[3][3]
);

void init_ekf_orientation(
    float process_noise[4][4],
    float measurement_noise[3][3],
    float expected_g[3]
);

void get_state(float quat[4], float pos[3], float vel[3]);

void init_ekf(
    float process_noise_quaternion[4][4],
    float measurement_noise_quaternion[3][3],
    float process_noise_body[6][6],
    float measurement_noise_body[3][3],
    float expected_g[3]
);

void tick_ekf_orientation(float deltaTime, float gyro[3], float accel[3]);

void tick_ekf_body(float deltaTime, float accel[3], float gps_pos[3]);

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