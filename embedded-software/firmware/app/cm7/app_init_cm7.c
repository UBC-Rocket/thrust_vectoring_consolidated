/**
 * @file    app_init_cm7.c
 * @brief   APP-layer phase for CM7: create the controls task. Drivers must
 *          already be initialised (dev_init_cm7 first).
 *
 * UBC Rocket, 2026
 */
#include "app/app_init.h"
#include "app/log_service.h"
#include "app/state_exchange.h"
#include "app/tasks.h"

#include "io_sys/io_debug.h"

#include "messages/messages.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define STK_CONTROLS (8 * 1024 / sizeof(StackType_t))

static StaticTask_t s_tcb_controls;
static StackType_t  s_stk_controls[STK_CONTROLS];

/* Defined in controls_task.c. */
extern void controls_isr_init(void);

/* Sink shim: hand assembled message records into log_service's raw-append
 * path. On CM7 this pushes the bytes into the SRAM4 SPSC ring shared with
 * CM4's sd_log_task. */
static bool messages_sd_sink_cm7(const uint8_t *record, size_t record_len) {
    return log_service_append_raw(record, (uint32_t)record_len);
}

void app_init_cm7(void) {
    state_exchange_init();

#ifdef DEBUG_TEXT_CONSOLE
    /* Bring up CM7's side of the shared debug console: io_debug_write() now
     * forwards text into the SRAM4 ring that CM4 drains to LPUART1/VCP. After
     * this, io_debug_printf() works from CM7 code and shows up in the same
     * serial terminal as CM4's output. */
    (void)io_debug_init();
    io_debug_printf("\r\n=== Ulysses CM7 online ===\r\n");
#endif

    /* Initialise the SRAM4 log handoff ring (CM7 is the one-shot zero of
     * the header). Must happen before messages_sd_sink_set so the first
     * publish doesn't race the magic write. */
    log_service_init();
    log_service_mark_ready();

    /* Bring up the messages runtime so the publish path / drop counters
     * are usable from controls. The UDP channel sink is installed later
     * from StartDefaultTask (after MX_LWIP_Init brings up tcpip_thread —
     * see messages_udp_init in cm7/messages_udp.c). */
    messages_init();
    messages_sd_sink_set(messages_sd_sink_cm7);

    /* Hard-real-time control loop in the TIM16 ISR (800 Hz). dev_init_cm7
     * MUST have run before this — controls_isr_init arms the ESC. Servos are
     * driven on CM4 from control_output_t published by the ISR. */
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
