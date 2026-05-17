/**
 * @file    dev_baro_ms5611.h
 * @brief   MS5611 driver. Opaque singleton.
 *
 * UBC Rocket, 2026
 */
#ifndef DEV_BARO_MS5611_H
#define DEV_BARO_MS5611_H

#include "io_sys/io_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t t_us;
    float    pressure_pa;
    float    temp_c;
} baro_sample_t;

typedef struct baro_ms5611 baro_ms5611_t;

bool           baro_ms5611_init      (void);
baro_ms5611_t *baro_ms5611_get       (void);
size_t         baro_ms5611_drain     (baro_ms5611_t *d, baro_sample_t *out, size_t max);
void           baro_ms5611_set_notify(baro_ms5611_t *d, io_task_handle_t t, uint32_t bit);
void           baro_ms5611_tick      (baro_ms5611_t *d);

#ifdef __cplusplus
}
#endif

#endif /* DEV_BARO_MS5611_H */
