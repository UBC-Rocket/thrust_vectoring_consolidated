/**
 * @file    io_dshot.h
 * @brief   IO-layer transport: send a 16-bit DShot frame.
 *
 * UBC Rocket, 2026
 */
#ifndef IO_DSHOT_H
#define IO_DSHOT_H

#include "io_sys/io_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct io_dshot_ch io_dshot_ch_t;

/**
 * @brief Send a pre-encoded 16-bit DShot frame.
 *        Returns IO_ERR_BUSY if the previous frame is still transmitting.
 */
io_status_t io_dshot_send(const io_dshot_ch_t *ch, uint16_t frame16);

bool        io_dshot_busy(const io_dshot_ch_t *ch);

/* Board-wired DShot channels (defined in per-target io_dshot_tim.c). MISRA 8.4. */
extern const io_dshot_ch_t IO_DSHOT_ESC_UPPER;
extern const io_dshot_ch_t IO_DSHOT_ESC_LOWER;

#ifdef __cplusplus
}
#endif

#endif /* IO_DSHOT_H */
