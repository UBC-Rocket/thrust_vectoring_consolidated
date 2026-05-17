/**
 * @file    io_init.c
 * @brief   IO-layer phase for CM4. Brings up transport services in the
 *          right order. Scheduler hand-off lives in app_start_kernel
 *          (app/cm4/app_init_cm4.c).
 *
 * UBC Rocket, 2026
 */
#include "io_sys/io_init.h"
#include "io_sys/io_sd.h"
#include "io_sys/io_status_led.h"
#include "io_sys/io_timestamp.h"
#include "io_sys/io_intercore.h"

extern void io_spi_init(void);   /* declared in io_spi.c */

void io_init_cm4(void) {
    io_timestamp_init();
    io_status_led_init();
    io_intercore_init();

    io_spi_init();      /* per-bus spi_queue instances + dispatch table */
    io_sd_init();
    /* io_uart_open / io_exti_register happen lazily from each driver. */
}
