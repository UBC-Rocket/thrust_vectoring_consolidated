#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <stdint.h>

uint64_t timestamp_us64(void);

static inline uint32_t timestamp_us(void)
{
    return (uint32_t)timestamp_us64();
}

#endif
