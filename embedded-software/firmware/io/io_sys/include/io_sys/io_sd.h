/**
 * @file io_sd.h
 * @brief SD card (SDMMC) interface — exposed at io_sys because the SD log
 *        task in the APP layer is its only consumer.
 *
 * UBC Rocket, 2026
 */
#ifndef IO_SD_H
#define IO_SD_H

#include "io_sys/io_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*io_sd_done_cb_t)(io_status_t status, void *arg);

void        io_sd_init(void);
bool        io_sd_card_present(void);

/**
 * @brief Submit an asynchronous write of @p n_blocks 512-byte blocks.
 *        Caller-owned data must stay valid until @p cb fires.
 */
io_status_t io_sd_write_blocks_async(uint32_t block_addr,
                                     const uint8_t *data,
                                     uint32_t n_blocks,
                                     io_sd_done_cb_t cb,
                                     void *arg);

io_status_t io_sd_erase(uint32_t start_block, uint32_t end_block);
uint64_t    io_sd_block_count(void);

#ifdef __cplusplus
}
#endif

#endif /* IO_SD_H */
