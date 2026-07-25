/**
 * @file    state.h
 * @brief   State types and quaternion/vector math for estimation and control.
 */
#ifndef STATE_H
#define STATE_H

#include <stdint.h>

/** Unit quaternion (Hamilton w + xi + yj + zk). */
typedef struct {
    float w;  /**< Scalar component */
    float x;  /**< Vector i component */
    float y;  /**< Vector j component */
    float z;  /**< Vector k component */
} quaternion_t;

/** Estimated state: nav-frame position/velocity, body-frame attitude/rate. */
typedef struct {
    float pos[3];       /**< Position [m] in nav frame {x, y, z} (z-up) */
    float vel[3];       /**< Velocity [m/s] in nav frame */
    quaternion_t q_bn;  /**< Body-to-nav attitude (unit) */
    float omega_b[3];   /**< Angular velocity [rad/s] body frame {p, q, r} */
    uint64_t u_s;       /**< Timestamp */
} state_t;

/** 3x3 rotation matrix (row-major). */
typedef struct {
    float R[3][3];
} rotation_matrix_t;

/**
 * @brief Convert a unit quaternion to a 3x3 rotation matrix.
 *
 * The quaternion is assumed to follow the Hamilton convention (w + xi + yj + zk)
 * and describe the rotation from the body frame to the navigation frame.
 *
 * @param q   Pointer to input quaternion.
 * @param out Pointer to rotation matrix to fill.
 */
void quaternion_to_rotation_matrix(const quaternion_t *q, rotation_matrix_t *out);

/**
 * @brief Multiply a rotation matrix by a 3-vector.
 *
 * Computes out = R * v.
 *
 * @param R     Pointer to rotation matrix.
 * @param v     Pointer to vector (length 3).
 * @param out   Destination for result (length 3).
 */
void rotation_matrix_vector_mul(const rotation_matrix_t *R,
                                const float v[3],
                                float out[3]);

/**
 * @brief Quaternion conjugate (inverse for unit quaternion).
 * @param q   Input quaternion.
 * @param out Output quaternion (q*).
 */
void quaternion_conjugate(const quaternion_t *q, quaternion_t *out);

/**
 * @brief Hamilton quaternion product: out = a * b.
 * @param a   First quaternion.
 * @param b   Second quaternion.
 * @param out Result quaternion.
 */
void quaternion_multiply(const quaternion_t *a, const quaternion_t *b, quaternion_t *out);

/**
 * @brief 3-vector cross product: out = a x b.
 * @param a   First vector (length 3).
 * @param b   Second vector (length 3).
 * @param out Result vector (length 3).
 */
void vec3_cross(const float a[3], const float b[3], float out[3]);

/**
 * @brief 3-vector dot product.
 * @param a First vector (length 3).
 * @param b Second vector (length 3).
 * @return a . b
 */
float vec3_dot(const float a[3], const float b[3]);

/**
 * @brief 3x3 matrix times 3-vector: out = M * v.
 * @param M   Row-major matrix M[3][3].
 * @param v   Vector (length 3).
 * @param out Result (length 3).
 */
void matrix33_vec3_mul(const float M[3][3], const float v[3], float out[3]);

/**
 * @brief Convert a rotation vector to a unit quaternion (exponential map).
 *
 * q = [cos(|rv|/2), sin(|rv|/2) * rv/|rv|]
 * For small |rv|, uses first-order Taylor expansion.
 *
 * @param rv  Rotation vector [3] (axis * angle in radians).
 * @param out Output unit quaternion [w,x,y,z].
 */
void quaternion_from_rotation_vector(const float rv[3], quaternion_t *out);

/**
 * @brief Build 3x3 skew-symmetric (cross-product) matrix from vector.
 *
 * [v]ₓ = [  0  -v2   v1 ]
 *        [  v2   0  -v0 ]
 *        [ -v1  v0   0  ]
 *
 * @param v   Input vector [3].
 * @param out Output 3x3 matrix.
 */
void skew_symmetric_3x3(const float v[3], float out[3][3]);

/**
 * @brief Normalize a 3-vector in-place.
 * @param v  Vector [3] to normalize.
 * @return   The original magnitude (0 if degenerate).
 */
float vec3_normalize(float v[3]);

#endif
