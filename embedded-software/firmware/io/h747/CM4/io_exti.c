/**
 * @file    io_exti.c
 * @brief   H747 CM4 IO impl: EXTI dispatch. Defines struct io_exti_line plus
 *          the const IO_EXTI_<NAME> symbols.
 *
 * UBC Rocket, 2026
 */
#include "io_common.h"
#include "io/io_exti.h"

#include <string.h>

#define MAX_EXTI_LINES 8

struct io_exti_line {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       slot;     /* index into s_state */
};

/* Public per-line symbols (referenced extern-const by drivers). */
const io_exti_line_t IO_EXTI_ICM40609_INT1    = { ICM_INT1_GPIO_Port,       ICM_INT1_Pin,       0 };
const io_exti_line_t IO_EXTI_ICM40609_INT2    = { ICM_INT2_GPIO_Port,       ICM_INT2_Pin,       5 };
const io_exti_line_t IO_EXTI_BMI088_ACC_INT1  = { BMI_ACC_INT1_GPIO_Port,   BMI_ACC_INT1_Pin,   1 };
const io_exti_line_t IO_EXTI_BMI088_GYRO_INT1 = { BMI_GYRO_INT1_GPIO_Port,  BMI_GYRO_INT1_Pin,  2 };
const io_exti_line_t IO_EXTI_MMC_INT          = { MMC_INT_GPIO_Port,        MMC_INT_Pin,        3 };
const io_exti_line_t IO_EXTI_SD_CARD_DETECT   = { SD_CARD_DETECT_GPIO_Port, SD_CARD_DETECT_Pin, 4 };

#define NUM_LINES 6
static const io_exti_line_t * const s_lines[NUM_LINES] = {
    &IO_EXTI_ICM40609_INT1,
    &IO_EXTI_BMI088_ACC_INT1,
    &IO_EXTI_BMI088_GYRO_INT1,
    &IO_EXTI_MMC_INT,
    &IO_EXTI_SD_CARD_DETECT,
    &IO_EXTI_ICM40609_INT2,   /* slot 5 — DEBUG: testing DATA_READY on INT2 (PE15) */
};

static struct {
    io_exti_cb_t cb;
    void        *user;
    bool         enabled;
} s_state[MAX_EXTI_LINES];

void io_exti_register(const io_exti_line_t *line, io_exti_cb_t cb, void *user) {
    if (!line) return;
    s_state[line->slot].cb   = cb;
    s_state[line->slot].user = user;
}

void io_exti_enable(const io_exti_line_t *line, bool enable) {
    if (!line) return;
    s_state[line->slot].enabled = enable;
}

/* HAL weak override — wired by CubeMX from each EXTIx_IRQHandler. */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t pin) {
    for (unsigned i = 0; i < NUM_LINES; ++i) {
        if (s_lines[i]->pin == pin && s_state[i].enabled && s_state[i].cb) {
            s_state[i].cb(s_state[i].user);
            return;
        }
    }
}
