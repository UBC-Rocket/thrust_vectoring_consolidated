#include "unity.h"
#include <math.h>

#include "trilat.h"

#define TOL 1e-4f

void test_trilaterate_n_tags_recovers_known_position(void)
{
    const tag tags[] = {
        {{0.0f, 0.0f, 0.0f}, sqrtf(3.0f)},
        {{2.0f, 0.0f, 0.0f}, sqrtf(3.0f)},
        {{0.0f, 2.0f, 0.0f}, sqrtf(3.0f)},
        {{0.0f, 0.0f, 2.0f}, sqrtf(3.0f)},
    };
    float position[3] = {0.0f, 0.0f, 0.0f};
    const int ok = trilaterate_n_tags(tags, 4U, position);

    TEST_ASSERT_EQUAL_INT(1, ok);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, position[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, position[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, position[2]);
}
