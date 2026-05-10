#ifndef TRILAT_H
#define TRILAT_H

#include <stddef.h>

typedef struct {
    float position[3];
    float distance;
} tag;

/*
 * Estimate the rocket position from at least four tags.
 *
 * The solver first computes a linear least-squares seed from the sphere
 * equations, then refines that estimate with a damped nonlinear least-squares
 * update on the range error.
 *
 * `output_pos` is written in the same units as `tag.position` and
 * `tag.distance`.
 *
 * Returns 1 on success, 0 if the inputs are invalid or the tag geometry is
 * singular.
 */
int trilaterate_n_tags(const tag arr[], size_t num_tags, float output_pos[3]);

/*
 * Estimate the rocket position using `initial_pos` as the nonlinear solver's
 * starting point.
 *
 * This is useful for running systems that want to seed the solve with the
 * previous position estimate so weakly observable geometry keeps the same
 * continuous solution branch.
 *
 * Returns 1 on success, 0 if the inputs are invalid or the tag geometry is
 * singular.
 */
int trilaterate_n_tags_with_guess(const tag arr[], size_t num_tags,
                                  const float initial_pos[3],
                                  float output_pos[3]);

#endif
