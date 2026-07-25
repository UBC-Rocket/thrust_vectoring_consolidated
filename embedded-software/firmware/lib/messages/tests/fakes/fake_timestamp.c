#include "timestamp/timestamp.h"

static uint64_t s_now_us = 0U;

uint64_t timestamp_us64(void)        { return s_now_us; }
void     fake_clock_set(uint64_t us) { s_now_us = us; }
void     fake_clock_advance(uint64_t us) { s_now_us += us; }
uint64_t fake_clock_get(void)        { return s_now_us; }
