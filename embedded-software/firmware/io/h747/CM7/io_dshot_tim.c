/**
 * @file    io_dshot_tim.c
 * @brief   H747 CM7 IO impl: DShot transport. Defines struct io_dshot_ch
 *          plus the const IO_DSHOT_<NAME> instances.
 *
 * Skeleton — TIM+DMA wiring still TODO once the CubeMX IOC settles.
 *
 * UBC Rocket, 2026
 */
#include "io_common.h"
#include "io/io_dshot.h"

#define DSHOT_T0  7
#define DSHOT_T1  14
#define DSHOT_BITS 16
#define DSHOT_FRAME_LEN (DSHOT_BITS + 2)

struct io_dshot_ch {
    /* TODO: TIM_HandleTypeDef *htim; uint32_t channel; DMA_HandleTypeDef *hdma; */
    uint16_t       *dma_buf;        /* per-channel mutable DMA-source buffer */
    volatile bool  *busy_flag;      /* per-channel mutable busy */
};

/* Per-channel mutable storage. */
static uint16_t      s_dma_upper[DSHOT_FRAME_LEN];
static uint16_t      s_dma_lower[DSHOT_FRAME_LEN];
static volatile bool s_busy_upper;
static volatile bool s_busy_lower;

const io_dshot_ch_t IO_DSHOT_ESC_UPPER = { .dma_buf = s_dma_upper, .busy_flag = &s_busy_upper };
const io_dshot_ch_t IO_DSHOT_ESC_LOWER = { .dma_buf = s_dma_lower, .busy_flag = &s_busy_lower };

void io_dshot_init(void) {
    /* TODO: HAL_TIM_PWM_Init/Start() on the TIM channels driving each ESC,
     *       arm the DMA streams, set ARR/PSC for the chosen DShot rate. */
    s_busy_upper = false;
    s_busy_lower = false;
}

static void encode(uint16_t frame, uint16_t *out) {
    out[0] = 0;
    for (int i = 0; i < DSHOT_BITS; ++i) {
        out[1 + i] = (frame & (1u << (DSHOT_BITS - 1 - i))) ? DSHOT_T1 : DSHOT_T0;
    }
    out[DSHOT_FRAME_LEN - 1] = 0;
}

io_status_t io_dshot_send(const io_dshot_ch_t *ch, uint16_t frame16) {
    if (!ch) return IO_ERR_PARAM;
    if (*ch->busy_flag) return IO_ERR_BUSY;
    encode(frame16, ch->dma_buf);
    *ch->busy_flag = true;
    /* TODO: kick off TIM+DMA burst. */
    return IO_OK;
}

bool io_dshot_busy(const io_dshot_ch_t *ch) {
    return ch ? *ch->busy_flag : false;
}
