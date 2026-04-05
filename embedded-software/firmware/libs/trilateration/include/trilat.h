#ifndef TRILAT_H
#define TRILAT_H

#include <stddef.h>

/*

TODO:

- define struct for each tag data (pos, distance)
- implement actual trilat function

*/

typedef struct {
    float position[3]; // position in 3d space relative to */ something */ (doesn't matter for now)
    float distance; // recorded distance from rocket
} tag;

void trilaterate_n_tags(tag arr[] ,size_t num_tags, int output_pos[3]);

#endif