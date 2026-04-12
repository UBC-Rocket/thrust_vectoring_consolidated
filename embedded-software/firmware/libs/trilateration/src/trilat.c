#include <math.h>
#include <stddef.h>

#include <trilat.h>

#define TRILAT_MIN_TAGS 4U
#define TRILAT_MAX_ITERS 32
#define TRILAT_MAX_BACKTRACK 8
#define TRILAT_EPSILON 1.0e-6f
#define TRILAT_GRAD_TOL 1.0e-4f
#define TRILAT_STEP_FRACTION 0.25f

static float squaref(float value)
{
    return value * value;
}

static float vec3_norm(const float v[3])
{
    return sqrtf(squaref(v[0]) + squaref(v[1]) + squaref(v[2]));
}

static float distance_between(const float a[3], const float b[3])
{
    const float delta[3] = {
        a[0] - b[0],
        a[1] - b[1],
        a[2] - b[2],
    };

    return vec3_norm(delta);
}

static int solve_3x3(float a[3][3], float b[3], float x[3])
{
    float aug[3][4];

    for (size_t row = 0; row < 3; ++row) {
        for (size_t col = 0; col < 3; ++col) {
            aug[row][col] = a[row][col];
        }
        aug[row][3] = b[row];
    }

    for (size_t pivot = 0; pivot < 3; ++pivot) {
        size_t best_row = pivot;
        float best_value = fabsf(aug[pivot][pivot]);

        for (size_t row = pivot + 1; row < 3; ++row) {
            const float candidate = fabsf(aug[row][pivot]);
            if (candidate > best_value) {
                best_value = candidate;
                best_row = row;
            }
        }

        if (best_value < TRILAT_EPSILON) {
            return 0;
        }

        if (best_row != pivot) {
            for (size_t col = 0; col < 4; ++col) {
                const float tmp = aug[pivot][col];
                aug[pivot][col] = aug[best_row][col];
                aug[best_row][col] = tmp;
            }
        }

        const float pivot_value = aug[pivot][pivot];
        for (size_t col = pivot; col < 4; ++col) {
            aug[pivot][col] /= pivot_value;
        }

        for (size_t row = 0; row < 3; ++row) {
            if (row == pivot) {
                continue;
            }

            const float factor = aug[row][pivot];
            for (size_t col = pivot; col < 4; ++col) {
                aug[row][col] -= factor * aug[pivot][col];
            }
        }
    }

    for (size_t row = 0; row < 3; ++row) {
        x[row] = aug[row][3];
    }

    return 1;
}

static int linearized_seed(const tag arr[], size_t num_tags, float seed[3])
{
    float ata[3][3] = {{0.0f}};
    float atb[3] = {0.0f, 0.0f, 0.0f};
    const float *reference = arr[0].position;
    const float reference_norm_sq =
        squaref(reference[0]) + squaref(reference[1]) + squaref(reference[2]);
    const float reference_distance_sq = squaref(arr[0].distance);

    for (size_t i = 1; i < num_tags; ++i) {
        float a_row[3];
        const float *position = arr[i].position;
        const float position_norm_sq =
            squaref(position[0]) + squaref(position[1]) + squaref(position[2]);
        const float rhs =
            reference_distance_sq - squaref(arr[i].distance) +
            position_norm_sq - reference_norm_sq;

        for (size_t axis = 0; axis < 3; ++axis) {
            a_row[axis] = 2.0f * (position[axis] - reference[axis]);
        }

        for (size_t row = 0; row < 3; ++row) {
            atb[row] += a_row[row] * rhs;
            for (size_t col = 0; col < 3; ++col) {
                ata[row][col] += a_row[row] * a_row[col];
            }
        }
    }

    return solve_3x3(ata, atb, seed);
}

static float mean_squared_error(const tag arr[], size_t num_tags,
                                const float position[3])
{
    float sum = 0.0f;

    for (size_t i = 0; i < num_tags; ++i) {
        const float residual =
            distance_between(position, arr[i].position) - arr[i].distance;
        sum += residual * residual;
    }

    return sum / (float)num_tags;
}

static void error_gradient(const tag arr[], size_t num_tags,
                           const float position[3], float *error,
                           float gradient[3], float *mean_distance)
{
    float err_sum = 0.0f;
    float distance_sum = 0.0f;

    gradient[0] = 0.0f;
    gradient[1] = 0.0f;
    gradient[2] = 0.0f;

    for (size_t i = 0; i < num_tags; ++i) {
        const float delta[3] = {
            position[0] - arr[i].position[0],
            position[1] - arr[i].position[1],
            position[2] - arr[i].position[2],
        };
        float predicted = vec3_norm(delta);
        const float residual = predicted - arr[i].distance;

        err_sum += residual * residual;
        distance_sum += arr[i].distance;

        if (predicted < TRILAT_EPSILON) {
            predicted = TRILAT_EPSILON;
        }

        const float scale = 2.0f * residual / predicted;
        gradient[0] += scale * delta[0];
        gradient[1] += scale * delta[1];
        gradient[2] += scale * delta[2];
    }

    *error = err_sum / (float)num_tags;
    *mean_distance = distance_sum / (float)num_tags;
    gradient[0] /= (float)num_tags;
    gradient[1] /= (float)num_tags;
    gradient[2] /= (float)num_tags;
}

int trilaterate_n_tags(const tag arr[], size_t num_tags, float output_pos[3])
{
    float position[3];
    float error = 0.0f;

    if (arr == NULL || output_pos == NULL || num_tags < TRILAT_MIN_TAGS) {
        return 0;
    }

    if (!linearized_seed(arr, num_tags, position)) {
        return 0;
    }

    error = mean_squared_error(arr, num_tags, position);

    for (size_t iter = 0; iter < TRILAT_MAX_ITERS; ++iter) {
        float gradient[3];
        float grad_norm;
        float mean_distance = 0.0f;
        float step;
        int improved = 0;

        error_gradient(arr, num_tags, position, &error, gradient,
                       &mean_distance);

        grad_norm = vec3_norm(gradient);
        if (grad_norm < TRILAT_GRAD_TOL) {
            break;
        }

        step = TRILAT_STEP_FRACTION *
               fmaxf(mean_distance, 1.0f) / fmaxf(grad_norm, TRILAT_EPSILON);

        for (size_t attempt = 0; attempt < TRILAT_MAX_BACKTRACK; ++attempt) {
            float candidate[3];
            float candidate_error;

            candidate[0] = position[0] - step * gradient[0];
            candidate[1] = position[1] - step * gradient[1];
            candidate[2] = position[2] - step * gradient[2];
            candidate_error = mean_squared_error(arr, num_tags, candidate);

            if (candidate_error <= error) {
                position[0] = candidate[0];
                position[1] = candidate[1];
                position[2] = candidate[2];
                error = candidate_error;
                improved = 1;
                break;
            }

            step *= 0.5f;
        }

        if (!improved || step * grad_norm < TRILAT_GRAD_TOL) {
            break;
        }
    }

    output_pos[0] = position[0];
    output_pos[1] = position[1];
    output_pos[2] = position[2];

    return 1;
}
