/**
 * @file    app_init_cm7.c
 * @brief   APP-layer phase for CM7: create the controls task. Drivers must
 *          already be initialised (dev_init_cm7 first).
 *
 * UBC Rocket, 2026
 */
#include "app/app_init.h"
#include "app/messages_udp.h"
#include "app/state_exchange.h"
#include "app/tasks.h"

#include "messages/messages.h"

#include "FreeRTOS.h"
#include "task.h"

#define STK_CONTROLS (8 * 1024 / sizeof(StackType_t))

static StaticTask_t s_tcb_controls;
static StackType_t  s_stk_controls[STK_CONTROLS];

void app_init_cm7(void) {
    state_exchange_init();

    /* Bring up the messages runtime so the publish path / drop counters /
     * channel sinks all exist. UDP sink registers as a no-op until lwIP
     * lands; see messages_udp.c. */
    messages_init();
    messages_udp_init();

    xTaskCreateStatic(task_controls, "controls",
                      STK_CONTROLS, NULL, 7,
                      s_stk_controls, &s_tcb_controls);
}

void app_start_kernel(void) {
    vTaskStartScheduler();
    for (;;) { __asm__ volatile ("bkpt 0"); }
}
