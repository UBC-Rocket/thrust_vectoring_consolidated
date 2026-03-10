/**
 * @file    state_math.c
 * @brief   Quaternion and vector math (conjugate, product, cross, dot, matrix-vec).
 */
#include "state_estimation/state.h"
#include <math.h>

/**
 * @brief Convert unit quaternion to 3x3 rotation matrix (body-to-nav).
 * @param q   Input quaternion (Hamilton).
 * @param out Output rotation matrix.
 */
void quaternion_to_rotation_matrix(const quaternion_t *q, rotation_matrix_t *out)
{
    if (!q || !out) return;

    const float w = q->w;
    const float x = q->x;
    const float y = q->y;
    const float z = q->z;

    const float ww = w * w;
    const float xx = x * x;
    const float yy = y * y;
    const float zz = z * z;

    const float xy = x * y;
    const float xz = x * z;
    const float yz = y * z;
    const float wx = w * x;
    const float wy = w * y;
    const float wz = w * z;

    out->R[0][0] = ww + xx - yy - zz;
    out->R[0][1] = 2.0f * (xy - wz);
    out->R[0][2] = 2.0f * (xz + wy);

    out->R[1][0] = 2.0f * (xy + wz);
    out->R[1][1] = ww - xx + yy - zz;
    out->R[1][2] = 2.0f * (yz - wx);

    out->R[2][0] = 2.0f * (xz - wy);
    out->R[2][1] = 2.0f * (yz + wx);
    out->R[2][2] = ww - xx - yy + zz;
}

/**
 * @brief Multiply rotation matrix by 3-vector: out = R * v.
 * @param R   Rotation matrix.
 * @param v   Input vector [3].
 * @param out Output vector [3].
 */
void rotation_matrix_vector_mul(const rotation_matrix_t *R,
                                const float v[3],
                                float out[3])
{
    if (!R || !v || !out) return;

    out[0] = R->R[0][0] * v[0] + R->R[0][1] * v[1] + R->R[0][2] * v[2];
    out[1] = R->R[1][0] * v[0] + R->R[1][1] * v[1] + R->R[1][2] * v[2];
    out[2] = R->R[2][0] * v[0] + R->R[2][1] * v[1] + R->R[2][2] * v[2];
}

/**
 * @brief Quaternion conjugate (inverse for unit quaternion).
 * @param q   Input quaternion.
 * @param out Output q* (out may equal q).
 */
void quaternion_conjugate(const quaternion_t *q, quaternion_t *out)
{
    if (!q || !out) return;
    out->w = q->w;
    out->x = -q->x;
    out->y = -q->y;
    out->z = -q->z;
}

/**
 * @brief Hamilton quaternion product: out = a * b.
 * @param a   First quaternion.
 * @param b   Second quaternion.
 * @param out Result (must not alias a or b).
 */
void quaternion_multiply(const quaternion_t *a, const quaternion_t *b, quaternion_t *out)
{
    if (!a || !b || !out) return;
    const float aw = a->w, ax = a->x, ay = a->y, az = a->z;
    const float bw = b->w, bx = b->x, by = b->y, bz = b->z;
    out->w = aw * bw - ax * bx - ay * by - az * bz;
    out->x = aw * bx + ax * bw + ay * bz - az * by;
    out->y = aw * by - ax * bz + ay * bw + az * bx;
    out->z = aw * bz + ax * by - ay * bx + az * bw;
}

/**
 * @brief 3-vector cross product: out = a x b.
 * @param a   First vector [3].
 * @param b   Second vector [3].
 * @param out Result [3].
 */
void vec3_cross(const float a[3], const float b[3], float out[3])
{
    if (!a || !b || !out) return;
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

/**
 * @brief 3-vector dot product.
 * @param a First vector [3].
 * @param b Second vector [3].
 * @return a . b
 */
float vec3_dot(const float a[3], const float b[3])
{
    if (!a || !b) return 0.0f;
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/**
 * @brief 3x3 matrix times 3-vector: out = M * v (row-major M).
 * @param M   Matrix [3][3].
 * @param v   Vector [3].
 * @param out Result [3].
 */
void matrix33_vec3_mul(const float M[3][3], const float v[3], float out[3])
{
    if (!M || !v || !out) return;
    out[0] = M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2];
    out[1] = M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2];
    out[2] = M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2];
}
