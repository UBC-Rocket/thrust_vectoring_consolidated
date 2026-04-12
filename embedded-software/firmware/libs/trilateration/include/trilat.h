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
 * equations, then refines that estimate with a first-order iterative update on
 * the nonlinear range error.
 *
 * `output_pos` is written in the same units as `tag.position` and
 * `tag.distance`.
 *
 * Returns 1 on success, 0 if the inputs are invalid or the tag geometry is
 * singular.
 */
int trilaterate_n_tags(const tag arr[], size_t num_tags, float output_pos[3]);

#endif
