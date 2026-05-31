/**
 * @file    sd_log_task.c
 * @brief   CM4 task: drain local + CM7 log staging buffers, write to SD.
 *
 * Skeleton stub — body to be consolidated from depracated/.../SD_logging/
 * and depracated/.../trace/, and adapted to the io_sd async write API.
 *
 * Phase-1 today: drives the messages-runtime 1 Hz drop-counter publish so
 * silent bus overruns surface in the log even before the real SD writer
 * is built. The drop-counter call is the only real work this task does
 * until the staging-buffer drain + io_sd write-back lands.
 *
 * UBC Rocket, 2026
 */

#include "app/tasks.h"
#include "app/log_service.h"
#include "app/crash_dump.h"
#include "io_sys/io_sd.h"
#include "io_sys/io_intercore.h"

#include "messages/messages.h"

#include "FreeRTOS.h"
#include "task.h"

#define SD_LOG_TICK_MS  1000U

void task_sd_log(void *arg) {
    (void)arg;
    /* TODO: bring up card, flush crash dump, loop: drain staging buffers,
     *       write asynchronously via io_sd_write_blocks_async. */
    for (;;) {
        messages_publish_drop_counters();
        vTaskDelay(pdMS_TO_TICKS(SD_LOG_TICK_MS));
    }
}
