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

void app_init_cm4(void) {
    state_exchange_init();
    log_service_init();

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
