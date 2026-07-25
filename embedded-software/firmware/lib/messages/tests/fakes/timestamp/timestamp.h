/* Test fake for timestamp/timestamp.h — controllable clock. */
#ifndef FAKE_TIMESTAMP_H
#define FAKE_TIMESTAMP_H

#include <stdint.h>

/* Public surface used by messages.c */
uint64_t timestamp_us64(void);

/* Test-only knobs (not part of the real library API). */
void     fake_clock_set(uint64_t us);
void     fake_clock_advance(uint64_t us);
uint64_t fake_clock_get(void);

#endif /* FAKE_TIMESTAMP_H */
