/**
 * @file    sd_log_task.c
 * @brief   CM4 task: drain local + CM7 log staging buffers, write to SD.
 *
 * Skeleton stub — body to be consolidated from depracated/.../SD_logging/
 * and depracated/.../trace/, and adapted to the io_sd async write API.
 *
 * UBC Rocket, 2026
 */

#include "app/tasks.h"
#include "app/log_service.h"
#include "app/crash_dump.h"
#include "io_sys/io_sd.h"
#include "io_sys/io_intercore.h"

void task_sd_log(void *arg) {
    (void)arg;
    /* TODO: bring up card, flush crash dump, loop: drain staging buffers,
     *       write asynchronously via io_sd_write_blocks_async. */
    for (;;) {
        /* placeholder */
    }
}
