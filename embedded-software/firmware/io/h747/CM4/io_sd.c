/**
 * @file    io_sd.c
 * @brief   H747 CM4 SDMMC1 async writer behind io_sd.
 *
 * UBC Rocket, 2026
 */
#include "io_common.h"
#include "io_sys/io_sd.h"
#include "io_sys/io_test_hooks.h"

#include "sdmmc.h"

static io_sd_done_cb_t s_cb;
static void           *s_arg;
static volatile bool   s_busy;

/* Test-only access to module-private state. No-op in production builds. */
IO_TEST_HOOK_RW(s_busy, bool,            io_sd_busy)
IO_TEST_HOOK_RW(s_cb,   io_sd_done_cb_t, io_sd_cb)

void io_sd_init(void) {
    /* SDMMC peripheral itself is initialised by MX_SDMMC1_SD_Init(). */
}

bool io_sd_card_present(void) {
    /* SD_CARD_DETECT is active-low (CubeMX label PD4). */
    return HAL_GPIO_ReadPin(SD_CARD_DETECT_GPIO_Port, SD_CARD_DETECT_Pin)
           == GPIO_PIN_RESET;
}

io_status_t io_sd_write_blocks_async(uint32_t block_addr,
                                     const uint8_t *data,
                                     uint32_t n_blocks,
                                     io_sd_done_cb_t cb,
                                     void *arg) {
    if (s_busy) return IO_ERR_BUSY;
    s_cb  = cb;
    s_arg = arg;
    s_busy = true;
    HAL_StatusTypeDef st = HAL_SD_WriteBlocks_DMA(&hsd1,
                                                  (uint8_t *)data,
                                                  block_addr, n_blocks);
    if (st != HAL_OK) {
        s_busy = false;
        return (st == HAL_BUSY) ? IO_ERR_BUSY : IO_ERR_BUS;
    }
    return IO_OK;
}

io_status_t io_sd_erase(uint32_t start_block, uint32_t end_block) {
    return HAL_SD_Erase(&hsd1, start_block, end_block) == HAL_OK ? IO_OK : IO_ERR_BUS;
}

uint64_t io_sd_block_count(void) {
    HAL_SD_CardInfoTypeDef info;
    if (HAL_SD_GetCardInfo(&hsd1, &info) != HAL_OK) return 0;
    return (uint64_t)info.LogBlockNbr;
}

void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd) {
    if (hsd != &hsd1) return;
    s_busy = false;
    if (s_cb) s_cb(IO_OK, s_arg);
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd) {
    if (hsd != &hsd1) return;
    s_busy = false;
    if (s_cb) s_cb(IO_ERR_BUS, s_arg);
}

void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd) {
    if (hsd != &hsd1) return;
    s_busy = false;
    if (s_cb) s_cb(IO_ERR_BUS, s_arg);
}
