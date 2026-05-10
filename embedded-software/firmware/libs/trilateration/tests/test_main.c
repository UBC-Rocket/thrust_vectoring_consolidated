#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

void test_trilaterate_n_tags_recovers_known_position(void);
void test_trilaterate_n_tags_with_guess_uses_initial_branch(void);

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_trilaterate_n_tags_recovers_known_position);
    RUN_TEST(test_trilaterate_n_tags_with_guess_uses_initial_branch);

    return UNITY_END();
}
