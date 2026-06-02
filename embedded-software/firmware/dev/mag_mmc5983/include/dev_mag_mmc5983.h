/**
 * @file    dev_mag_mmc5983.h
 * @brief   MEMSIC MMC5983MA magnetometer driver. Opaque singleton.
 *
 * SPI device on IO_SPI_MMC (= SPI1 on the H747 board), DRDY input on
 * IO_EXTI_MMC_INT. Default config: 100 Hz continuous mode with periodic
 * SET (every 1 measurement) so offset stays cancelled. 18-bit raw
 * counts are converted to gauss (1 mG / LSB on the ±8 G range) on the
 * way into the ring.
 *
 * UBC Rocket, 2026
 */
#ifndef DEV_MAG_MMC5983_H
#define DEV_MAG_MMC5983_H

#include "io_sys/io_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t t_us;
    /** Magnetic field in gauss, sensor body frame (chip XYZ as marked).
     *  Apply the board's mounting matrix in the consumer if needed. */
    float    mx, my, mz;
    float    temp_c;
} mag_sample_t;

typedef struct mag_mmc5983 mag_mmc5983_t;

bool            mag_mmc5983_init       (void);
mag_mmc5983_t  *mag_mmc5983_get        (void);
size_t          mag_mmc5983_drain      (mag_mmc5983_t *d,
                                         mag_sample_t *out, size_t max);
void            mag_mmc5983_set_notify (mag_mmc5983_t *d,
                                         io_task_handle_t t, uint32_t bit);

#ifdef __cplusplus
}
#endif

#endif /* DEV_MAG_MMC5983_H */
