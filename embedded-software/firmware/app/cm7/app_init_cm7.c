/**
 * @file    app_init_cm7.c
 * @brief   APP-layer phase for CM7: create the controls task. Drivers must
 *          already be initialised (dev_init_cm7 first).
 *
 * UBC Rocket, 2026
 */
#include "app/app_init.h"
#include "app/state_exchange.h"
#include "app/tasks.h"

#include "messages/messages.h"

#include "FreeRTOS.h"
#include "task.h"

#define STK_CONTROLS (8 * 1024 / sizeof(StackType_t))

static StaticTask_t s_tcb_controls;
static StackType_t  s_stk_controls[STK_CONTROLS];

/* Defined in controls_task.c. */
extern void controls_isr_init(void);

void app_init_cm7(void) {
    state_exchange_init();

    /* Bring up the messages runtime so the publish path / drop counters
     * are usable from controls. The UDP channel sink is installed later
     * from StartDefaultTask (after MX_LWIP_Init brings up tcpip_thread —
     * see messages_udp_init in cm7/messages_udp.c). */
    messages_init();

    /* Hard-real-time control loop in the TIM16 ISR (800 Hz). dev_init_cm7
     * MUST have run before this — controls_isr_init snapshots the
     * actuator handles + arms ESC + enables servo torque. */
    controls_isr_init();

    /* Solver task — body is a stub today; eventually publishes reference
     * setpoints the ISR consumes. */
    xTaskCreateStatic(task_controls, "controls",
                      STK_CONTROLS, NULL, 7,
                      s_stk_controls, &s_tcb_controls);
}

void app_start_kernel(void) {
    vTaskStartScheduler();
    for (;;) { __asm__ volatile ("bkpt 0"); }
}
