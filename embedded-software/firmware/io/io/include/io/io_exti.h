/**
 * @file    io_exti.h
 * @brief   IO-layer transport: register a callback for a logical EXTI line.
 *
 * Drivers reference &IO_EXTI_<NAME>; defined in the per-target io_exti.c.
 *
 * UBC Rocket, 2026
 */
#ifndef IO_EXTI_H
#define IO_EXTI_H

#include "io_sys/io_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque — defined in the per-target io_exti.c. */
typedef struct io_exti_line io_exti_line_t;

typedef void (*io_exti_cb_t)(void *user);

void io_exti_register(const io_exti_line_t *line, io_exti_cb_t cb, void *user);
void io_exti_enable  (const io_exti_line_t *line, bool enable);

/* Board-wired EXTI lines (defined in per-target io_exti.c). MISRA 8.4. */
extern const io_exti_line_t IO_EXTI_ICM40609_INT1;
extern const io_exti_line_t IO_EXTI_ICM40609_INT2;
extern const io_exti_line_t IO_EXTI_BMI088_ACC_INT1;
extern const io_exti_line_t IO_EXTI_BMI088_GYRO_INT1;
extern const io_exti_line_t IO_EXTI_MMC_INT;
extern const io_exti_line_t IO_EXTI_SD_CARD_DETECT;

#ifdef __cplusplus
}
#endif

#endif /* IO_EXTI_H */
