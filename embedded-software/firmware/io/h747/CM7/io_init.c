/**
 * @file    io_init.c
 * @brief   IO-layer phase for CM7. Scheduler hand-off lives in
 *          app_start_kernel (app/cm7/app_init_cm7.c).
 *
 * UBC Rocket, 2026
 */
#include "io_sys/io_init.h"
#include "io_sys/io_status_led.h"
#include "io_sys/io_timestamp.h"
#include "io_sys/io_intercore.h"

extern void io_dshot_init(void);  /* declared in io_dshot_tim.c */

void io_init_cm7(void) {
    io_timestamp_init();
    io_status_led_init();
    io_intercore_init();
    io_dshot_init();
}
