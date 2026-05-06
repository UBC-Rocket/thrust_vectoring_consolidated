#ifndef CLAMP.H
#define CLAMP.H

#include <stdint.h>

int clamp_int(int value, int min, int max); 

float clamp_float(float value, float min, float max);

double clamp_double(double value, double min, double max);

uint8_t clamp_u8(uint8_t value, uint8_t min, uint8_t max);

uint16_t clamp_u16(uint16_t value, uint16_t min, uint16_t max);

uint32_t clamp_u32(uint32_t value, uint32_t min, uint32_t max);

uint64_t clamp_u64(uint64_t value, uint64_t min, uint64_t max);

int8_t clamp_i8(int8_t value, int8_t min, int8_t max);

int16_t clamp_i16(int16_t value, int16_t min, int16_t max);

int32_t clamp_i32(int32_t value, int32_t min, int32_t max);

int64_t clamp_i64(int64_t value, int64_t min, int64_t max);

#endif