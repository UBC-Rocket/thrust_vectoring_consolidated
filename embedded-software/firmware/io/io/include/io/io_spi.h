/**
 * @file    io_spi.h
 * @brief   IO-layer transport: SPI submission against opaque device handles.
 *
 * UBC Rocket, 2026
 */
#ifndef IO_SPI_H
#define IO_SPI_H

#include "io_sys/io_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct io_spi_dev  io_spi_dev_t;       /* opaque, defined in target IO impl */
typedef struct io_spi_xfer io_spi_xfer_t;
typedef void (*io_spi_done_cb_t)(const io_spi_xfer_t *x, void *user, io_status_t st);

struct io_spi_xfer {
    const uint8_t    *tx;        ///< source bytes (caller owns; alive until done)
    uint8_t          *rx;        ///< destination bytes (caller owns)
    uint16_t          len;
    io_spi_done_cb_t  done;      ///< called from ISR after CS deasserted; may be NULL
    void             *user;
    uint64_t          t_sample;  ///< caller stamps; echoed to done
};

/**
 * @brief Submit an asynchronous SPI transfer. Returns IO_OK if queued,
 *        IO_ERR_BUSY if the bus queue is full.
 */
io_status_t io_spi_submit(const io_spi_dev_t *dev, const io_spi_xfer_t *x);

/**
 * @brief Blocking SPI transfer. Intended for driver init.
 */
io_status_t io_spi_xfer_blocking(const io_spi_dev_t *dev,
                                 const uint8_t *tx, uint8_t *rx,
                                 uint16_t len, uint32_t timeout_ms);

/* ------------------------------------------------------------------------- *
 * Board-wired SPI devices.
 *
 * Defined by the per-target IO impl (e.g. io/h747/CM4/io_spi.c). The extern
 * declaration is visible to both the definer and every driver via this
 * header — MISRA-C 2012 Rule 8.4 compliant.
 * ------------------------------------------------------------------------- */
extern const io_spi_dev_t IO_SPI_ICM40609;
extern const io_spi_dev_t IO_SPI_BMI088_ACC;
extern const io_spi_dev_t IO_SPI_BMI088_GYRO;
extern const io_spi_dev_t IO_SPI_MS5611;
extern const io_spi_dev_t IO_SPI_MMC;

#ifdef __cplusplus
}
#endif

#endif /* IO_SPI_H */
