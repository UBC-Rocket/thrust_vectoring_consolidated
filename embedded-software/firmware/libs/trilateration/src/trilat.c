#include <math.h>
#include <stddef.h>
#include <float.h>

#include <trilat.h>

#define TRILAT_MIN_TAGS 4U
#define TRILAT_MAX_ITERS 48
#define TRILAT_MAX_LM_ATTEMPTS 12
#define TRILAT_EPSILON 1.0e-6f
#define TRILAT_GRAD_TOL 1.0e-4f
#define TRILAT_STEP_TOL 1.0e-5f
#define TRILAT_ERROR_TOL 1.0e-10f
#define TRILAT_DAMPING_INITIAL 1.0e-3f
#define TRILAT_DAMPING_MIN 1.0e-7f
#define TRILAT_DAMPING_INCREASE 10.0f
#define TRILAT_DAMPING_DECREASE 0.3f
#define TRILAT_WEAK_AXIS_RATIO 0.05f
#define TRILAT_WEAK_AXIS_REGULARIZATION 1.0e-2f

// HELPERS

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

static int is_valid_float(float value)
{
    return isfinite(value);
}

static int is_valid_vec3(const float v[3])
{
    return is_valid_float(v[0]) && is_valid_float(v[1]) &&
           is_valid_float(v[2]);
}

static int is_valid_tag(const tag *item)
{
    return item != NULL && is_valid_vec3(item->position) &&
           is_valid_float(item->distance) && item->distance >= 0.0f;
}

static int validate_problem(const tag arr[], size_t num_tags, float output_pos[3])
{
    if (arr == NULL || output_pos == NULL || num_tags < TRILAT_MIN_TAGS) {
        return 0;
    }

    for (size_t i = 0; i < num_tags; ++i) {
        if (!is_valid_tag(&arr[i])) {
            return 0;
        }
    }

    return 1;
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

static void centroid_and_spread(const tag arr[], size_t num_tags,
                                float centroid[3], float spread[3])
{
    centroid[0] = 0.0f;
    centroid[1] = 0.0f;
    centroid[2] = 0.0f;

    for (size_t i = 0; i < num_tags; ++i) {
        centroid[0] += arr[i].position[0];
        centroid[1] += arr[i].position[1];
        centroid[2] += arr[i].position[2];
    }

    centroid[0] /= (float)num_tags;
    centroid[1] /= (float)num_tags;
    centroid[2] /= (float)num_tags;

    spread[0] = 0.0f;
    spread[1] = 0.0f;
    spread[2] = 0.0f;

    for (size_t i = 0; i < num_tags; ++i) {
        spread[0] += squaref(arr[i].position[0] - centroid[0]);
        spread[1] += squaref(arr[i].position[1] - centroid[1]);
        spread[2] += squaref(arr[i].position[2] - centroid[2]);
    }

    spread[0] = sqrtf(spread[0] / (float)num_tags);
    spread[1] = sqrtf(spread[1] / (float)num_tags);
    spread[2] = sqrtf(spread[2] / (float)num_tags);
}

static void weak_axis_regularization(const float spread[3], float regularization[3])
{
    const float max_spread = fmaxf(spread[0], fmaxf(spread[1], spread[2]));

    regularization[0] = 0.0f;
    regularization[1] = 0.0f;
    regularization[2] = 0.0f;

    if (max_spread < TRILAT_EPSILON) {
        return;
    }

    for (size_t axis = 0; axis < 3; ++axis) {
        const float ratio = spread[axis] / max_spread;

        if (ratio < TRILAT_WEAK_AXIS_RATIO) {
            const float weakness =
                (TRILAT_WEAK_AXIS_RATIO - ratio) / TRILAT_WEAK_AXIS_RATIO;
            regularization[axis] =
                TRILAT_WEAK_AXIS_REGULARIZATION * squaref(weakness);
        }
    }
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

    if (!is_valid_vec3(position)) {
        return FLT_MAX;
    }

    for (size_t i = 0; i < num_tags; ++i) {
        const float residual =
            distance_between(position, arr[i].position) - arr[i].distance;

        if (!is_valid_float(residual)) {
            return FLT_MAX;
        }

        sum += residual * residual;
    }

    return sum / (float)num_tags;
}

static int normal_equations(const tag arr[], size_t num_tags,
                            const float position[3], float jtj[3][3],
                            float jtr[3], float *error)
{
    float err_sum = 0.0f;

    if (!is_valid_vec3(position) || error == NULL) {
        return 0;
    }

    for (size_t row = 0; row < 3; ++row) {
        jtr[row] = 0.0f;
        for (size_t col = 0; col < 3; ++col) {
            jtj[row][col] = 0.0f;
        }
    }

    for (size_t i = 0; i < num_tags; ++i) {
        const float delta[3] = {
            position[0] - arr[i].position[0],
            position[1] - arr[i].position[1],
            position[2] - arr[i].position[2],
        };
        float predicted = vec3_norm(delta);
        const float residual = predicted - arr[i].distance;
        float jacobian[3];

        if (!is_valid_float(predicted) || !is_valid_float(residual)) {
            return 0;
        }

        err_sum += residual * residual;

        if (predicted < TRILAT_EPSILON) {
            continue;
        }

        jacobian[0] = delta[0] / predicted;
        jacobian[1] = delta[1] / predicted;
        jacobian[2] = delta[2] / predicted;

        for (size_t row = 0; row < 3; ++row) {
            jtr[row] += jacobian[row] * residual;
            for (size_t col = 0; col < 3; ++col) {
                jtj[row][col] += jacobian[row] * jacobian[col];
            }
        }
    }

    *error = err_sum / (float)num_tags;

    for (size_t row = 0; row < 3; ++row) {
        jtr[row] /= (float)num_tags;
        for (size_t col = 0; col < 3; ++col) {
            jtj[row][col] /= (float)num_tags;
        }
    }

    return is_valid_float(*error);
}

static int automatic_initial_position(const tag arr[], size_t num_tags,
                                      float initial_pos[3])
{
    float centroid[3];
    float spread[3];

    centroid_and_spread(arr, num_tags, centroid, spread);
    if (fmaxf(spread[0], fmaxf(spread[1], spread[2])) < TRILAT_EPSILON) {
        return 0;
    }

    initial_pos[0] = centroid[0];
    initial_pos[1] = centroid[1];
    initial_pos[2] = centroid[2];

    {
        float seed[3];
        if (linearized_seed(arr, num_tags, seed) && is_valid_vec3(seed) &&
            mean_squared_error(arr, num_tags, seed) <=
                mean_squared_error(arr, num_tags, initial_pos)) {
            initial_pos[0] = seed[0];
            initial_pos[1] = seed[1];
            initial_pos[2] = seed[2];
        }
    }

    return 1;
}

static int refine_from_initial_position(const tag arr[], size_t num_tags,
                                        const float initial_pos[3],
                                        float output_pos[3],
                                        int regularize_weak_axes)
{
    float position[3];
    float centroid[3];
    float spread[3];
    float axis_regularization[3];
    float error = 0.0f;
    float damping = TRILAT_DAMPING_INITIAL;

    if (!is_valid_vec3(initial_pos)) {
        return 0;
    }

    centroid_and_spread(arr, num_tags, centroid, spread);
    if (fmaxf(spread[0], fmaxf(spread[1], spread[2])) < TRILAT_EPSILON) {
        return 0;
    }
    if (regularize_weak_axes) {
        weak_axis_regularization(spread, axis_regularization);
    } else {
        axis_regularization[0] = 0.0f;
        axis_regularization[1] = 0.0f;
        axis_regularization[2] = 0.0f;
    }

    position[0] = initial_pos[0];
    position[1] = initial_pos[1];
    position[2] = initial_pos[2];

    error = mean_squared_error(arr, num_tags, position);
    if (!is_valid_float(error) || error == FLT_MAX) {
        return 0;
    }

    for (size_t iter = 0; iter < TRILAT_MAX_ITERS; ++iter) {
        float jtj[3][3];
        float jtr[3];
        float grad_norm;
        int improved = 0;
        int converged = 0;

        if (!normal_equations(arr, num_tags, position, jtj, jtr, &error)) {
            return 0;
        }

        grad_norm = vec3_norm(jtr);
        if (grad_norm < TRILAT_GRAD_TOL) {
            break;
        }

        for (size_t attempt = 0; attempt < TRILAT_MAX_LM_ATTEMPTS; ++attempt) {
            float system[3][3];
            float rhs[3];
            float step[3];
            float candidate[3];
            float candidate_error;
            float improvement;
            const float previous_error = error;

            for (size_t row = 0; row < 3; ++row) {
                rhs[row] = -jtr[row];
                for (size_t col = 0; col < 3; ++col) {
                    system[row][col] = jtj[row][col];
                }
                system[row][row] += damping + axis_regularization[row];
            }

            if (!solve_3x3(system, rhs, step) || !is_valid_vec3(step)) {
                damping *= TRILAT_DAMPING_INCREASE;
                continue;
            }

            candidate[0] = position[0] + step[0];
            candidate[1] = position[1] + step[1];
            candidate[2] = position[2] + step[2];
            candidate_error = mean_squared_error(arr, num_tags, candidate);

            if (is_valid_float(candidate_error) && candidate_error <= error) {
                position[0] = candidate[0];
                position[1] = candidate[1];
                position[2] = candidate[2];
                error = candidate_error;
                improvement = previous_error - error;
                damping = fmaxf(damping * TRILAT_DAMPING_DECREASE,
                                TRILAT_DAMPING_MIN);
                improved = 1;

                if (vec3_norm(step) < TRILAT_STEP_TOL ||
                    improvement <= TRILAT_ERROR_TOL * fmaxf(previous_error, 1.0f)) {
                    converged = 1;
                }
                break;
            }

            damping *= TRILAT_DAMPING_INCREASE;
        }

        if (!improved || converged) {
            break;
        }
    }

    if (!is_valid_vec3(position)) {
        return 0;
    }

    output_pos[0] = position[0];
    output_pos[1] = position[1];
    output_pos[2] = position[2];

    return 1;
}

// ACTUALLY USEFUL FUNCTIONS

int trilaterate_n_tags_with_guess(const tag arr[], size_t num_tags,
                                  const float initial_pos[3],
                                  float output_pos[3])
{
    if (!validate_problem(arr, num_tags, output_pos)) {
        return 0;
    }

    return refine_from_initial_position(
        arr,
        num_tags,
        initial_pos,
        output_pos,
        0
    );
}

int trilaterate_n_tags(const tag arr[], size_t num_tags, float output_pos[3])
{
    float initial_pos[3];

    if (!validate_problem(arr, num_tags, output_pos)) {
        return 0;
    }

    if (!automatic_initial_position(arr, num_tags, initial_pos)) {
        return 0;
    }

    return refine_from_initial_position(
        arr,
        num_tags,
        initial_pos,
        output_pos,
        1
    );
}
