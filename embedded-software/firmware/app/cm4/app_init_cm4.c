/**
 * @file    app_init_cm4.c
 * @brief   APP-layer phase for CM4: create FreeRTOS tasks, wire notifies.
 *          Drivers must already be initialised (dev_init_cm4 first).
 *
 * UBC Rocket, 2026
 */
#include "app/app_init.h"
#include "app/sensors_init.h"
#include "app/log_service.h"
#include "app/state_exchange.h"
#include "app/tasks.h"

#include "messages/messages.h"

#include "FreeRTOS.h"
#include "task.h"

/* Static task allocations. Stack sizes will need tuning per task. */
#define STK_STATE_EST  (8 * 1024 / sizeof(StackType_t))
#define STK_MISSION    (4 * 1024 / sizeof(StackType_t))
#define STK_SD_LOG     (4 * 1024 / sizeof(StackType_t))

static StaticTask_t s_tcb_state, s_tcb_mission, s_tcb_log;
static StackType_t  s_stk_state[STK_STATE_EST];
static StackType_t  s_stk_mission[STK_MISSION];
static StackType_t  s_stk_log[STK_SD_LOG];

static TaskHandle_t s_h_state, s_h_mission, s_h_log;

/* Sink shim: hand assembled message records into log_service's raw-append
 * path. messages_sink_fn_t and log_service_append_raw use the same
 * (const uint8_t*, len, -> bool) contract, but we keep the shim so the
 * coupling is explicit and easy to swap when the SD path firms up. */
static bool messages_sd_sink(const uint8_t *record, size_t record_len) {
    return log_service_append_raw(record, (uint32_t)record_len);
}

void app_init_cm4(void) {
    state_exchange_init();
    log_service_init();

    /* Bring up the structured messages runtime and route the SD channel
     * into log_service. Must happen AFTER log_service_init (the sink
     * touches log_service state) and before any task starts publishing. */
    messages_init();
    messages_sd_sink_set(messages_sd_sink);

    s_h_log     = xTaskCreateStatic(task_sd_log,           "sd_log",
                                    STK_SD_LOG,            NULL, 4,
                                    s_stk_log,             &s_tcb_log);
    s_h_mission = xTaskCreateStatic(task_mission_manager,  "mission",
                                    STK_MISSION,           NULL, 5,
                                    s_stk_mission,         &s_tcb_mission);
    s_h_state   = xTaskCreateStatic(task_state_estimation, "state",
                                    STK_STATE_EST,         NULL, 6,
                                    s_stk_state,           &s_tcb_state);

    sensors_bind_state_estimation_task((io_task_handle_t)s_h_state);
}

void app_start_kernel(void) {
    vTaskStartScheduler();
    for (;;) { __asm__ volatile ("bkpt 0"); }   /* should not reach here */
}
