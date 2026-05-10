#ifndef UTILITIES_CLAMP_H
#define UTILITIES_CLAMP_H

#include <stdint.h>

inline int clamp_int(int value, int min, int max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

inline float clamp_float(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

inline double clamp_double(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

inline uint8_t clamp_u8(uint8_t value, uint8_t min, uint8_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

inline uiint16_t clamp_u16(uint16_t value, uint16_t min, uint16_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

inline uint32_t clamp_u32(uint32_t value, uint32_t min, uint32_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

inline uint64_t clamp_u64(uint64_t value, uint64_t min, uint64_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

inline int8_t clamp_i8(int8_t value, int8_t min, int8_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

inline int16_t clamp_i16(int16_t value, int16_t min, int16_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

inline int32_t clamp_i32(int32_t value, int32_t min, int32_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

inline int64_t clamp_i64(int64_t value, int64_t min, int64_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}


#endif /* UTILITIES_CLAMP_H */
