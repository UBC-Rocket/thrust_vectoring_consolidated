#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

void test_trilaterate_n_tags_recovers_known_position(void);

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_trilaterate_n_tags_recovers_known_position);

    return UNITY_END();
}
