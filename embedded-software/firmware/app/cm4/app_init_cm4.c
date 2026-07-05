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
#include "app/messages_vcp.h"
#include "app/state_exchange.h"
#include "app/tasks.h"

#include "messages/messages.h"
#include "storage/storage.h"

#include "io_sys/io_debug.h"
#include "io_sys/io_status_led.h"   /* TEMP bring-up tracer */

#ifdef USE_DYNAMIXEL_SERVO
#include "dev_servo_dynamixel.h"
#endif

#include "FreeRTOS.h"
#include "task.h"

/* Static task allocations. Stack sizes will need tuning per task. */
#define STK_STATE_EST  (8 * 1024 / sizeof(StackType_t))
#define STK_MISSION    (4 * 1024 / sizeof(StackType_t))
#define STK_SD_LOG     (4 * 1024 / sizeof(StackType_t))
#define STK_ACTUATOR   (4 * 1024 / sizeof(StackType_t))

static StaticTask_t s_tcb_state, s_tcb_mission, s_tcb_log, s_tcb_actuator;
static StackType_t  s_stk_state[STK_STATE_EST];
static StackType_t  s_stk_mission[STK_MISSION];
static StackType_t  s_stk_log[STK_SD_LOG];
static StackType_t  s_stk_actuator[STK_ACTUATOR];

static TaskHandle_t s_h_state, s_h_mission, s_h_log, s_h_actuator;

#ifdef DEBUG_TEXT_CONSOLE
/* Forwards CM7 console text to LPUART1: drains the SRAM4 console ring every
 * 20 ms and emits via io_debug_write. Console text is low-rate and latency-
 * tolerant, so a poll avoids adding an HSEM event for the wake. */
#define STK_DBGPUMP  (1 * 1024 / sizeof(StackType_t))
static StaticTask_t s_tcb_dbg;
static StackType_t  s_stk_dbg[STK_DBGPUMP];

static void task_debug_pump(void *arg) {
    (void)arg;
    for (;;) {
        io_debug_pump_remote();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
#endif

/* Sink shim: hand assembled message records into log_service's raw-append
 * path. messages_sink_fn_t and log_service_append_raw use the same
 * (const uint8_t*, len, -> bool) contract, but we keep the shim so the
 * coupling is explicit and easy to swap when the SD path firms up. */
static bool messages_sd_sink(const uint8_t *record, size_t record_len) {
    return log_service_append_raw(record, (uint32_t)record_len);
}

void app_init_cm4(void) {
    state_exchange_init();

    /* Seed every CM4-owned exchange slot with a benign default. The .shared
     * SRAM4 section is NOLOAD on both cores and nothing zeroes it, so until
     * a slot's first publish its seqlock header is power-on garbage — which
     * the reader happily returns as "valid" data. CM7's controls task polls
     * pid_gains/reference/vehicle_config and would merge random positive
     * floats into its live gains. Publishing once here writes coherent
     * headers before CM7's tasks ever poll. Zeros are deliberate no-ops on
     * the CM7 side: merge_* helpers guard on > 0, and q_valid=false leaves
     * the attitude reference at identity. */
    {
        const app_pid_gains_t      pid_defaults = {0};
        const app_reference_t      ref_defaults = {0};   /* q_valid = false */
        const app_vehicle_config_t cfg_defaults = {0};
        state_t state_default = {0};
        state_default.q_bn.w = 1.0f;                      /* identity attitude */

        (void)state_exchange_publish_pid_gains(&pid_defaults);
        (void)state_exchange_publish_reference(&ref_defaults);
        (void)state_exchange_publish_vehicle_config(&cfg_defaults);
        (void)state_exchange_publish_armed(false);
        (void)state_exchange_publish_flight_state(APP_FLIGHT_IDLE);
        (void)state_exchange_publish_state(&state_default);
    }

    log_service_init();

    /* Bring up persistent storage. Reads both flash sectors, picks
     * the higher-generation copy with a valid CRC, populates
     * g_storage, spawns the storage task. STORAGE_LOAD_FRESH on first
     * boot or both sectors corrupt — defaults are applied and the
     * first save() will program sector A with generation = 1. */
    (void)storage_init();

    /* Bring up the structured messages runtime and route the SD channel
     * into log_service. Must happen AFTER log_service_init (the sink
     * touches log_service state) and before any task starts publishing. */
    messages_init();
    messages_sd_sink_set(messages_sd_sink);
    messages_system_commands_register();

#ifdef DEBUG_TEXT_CONSOLE
    /* Bring-up / dev default: LPUART1 (PA9/PA10 → STLINK-V3 VCP) is a plain-
     * text debug console owned by io_debug. The binary "messages" VCP channel
     * is intentionally left without a sink here (its records still route to SD
     * and, once lwIP is up, UDP) so a serial terminal sees clean text. Open
     * CoolTerm on the STLINK-V3 VCP at 115200 8-N-1. Flip the DEBUG_TEXT_CONSOLE
     * CMake option OFF to restore the binary messages-console on VCP instead. */
    (void)io_debug_init();
    io_debug_printf("\r\n=== Ulysses CM4 online — hello world ===\r\n");

    /* Bring-up: one-line IMU status on the console. sensors_handles()->icm40609
     * is non-NULL iff imu_icm40609_init() fully succeeded (which includes the
     * WHO_AM_I == 0x3B identity gate). */
    {
        const sensors_t *sn = sensors_handles();
        io_debug_printf("[init] ICM40609 %s\r\n",
                        (sn != NULL && sn->icm40609 != NULL) ? "OK" : "FAIL");
    }

#ifdef USE_DYNAMIXEL_SERVO
    servo_dynamixel_log_status();
#endif
#else
    /* Open the VCP UART, COBS-frame on RX → dispatcher, COBS-frame on TX
     * out via the CH_VCP sink. After this, the host messages-console CLI
     * can connect and exchange commands. */
    messages_vcp_init();
#endif

    s_h_log     = xTaskCreateStatic(task_sd_log,           "sd_log",
                                    STK_SD_LOG,            NULL, 4,
                                    s_stk_log,             &s_tcb_log);
    s_h_mission = xTaskCreateStatic(task_mission_manager,  "mission",
                                    STK_MISSION,           NULL, 5,
                                    s_stk_mission,         &s_tcb_mission);
    s_h_state     = xTaskCreateStatic(task_state_estimation, "state",
                                    STK_STATE_EST,         NULL, 6,
                                    s_stk_state,           &s_tcb_state);
    s_h_actuator  = xTaskCreateStatic(task_actuator,         "actuator",
                                    STK_ACTUATOR,          NULL, 7,
                                    s_stk_actuator,        &s_tcb_actuator);
    (void)s_h_actuator;

    sensors_bind_state_estimation_task((io_task_handle_t)s_h_state);

#ifdef DEBUG_TEXT_CONSOLE
    /* Forward CM7's debug-console text to the shared VCP. Low priority — it
     * only shuttles bytes from the SRAM4 ring to LPUART1. */
    (void)xTaskCreateStatic(task_debug_pump, "dbgpump",
                            STK_DBGPUMP, NULL, 2,
                            s_stk_dbg, &s_tcb_dbg);
#endif

    /* TEMP bring-up tracer: RED solid = app_init_cm4 finished (tasks created,
     * about to start the scheduler). mission_manager then blinks RED once it
     * runs, so RED-solid vs RED-blinking distinguishes "scheduler never started"
     * from "scheduler + tasks running". Remove after bring-up. */
    io_status_led_set(IO_LED_RED, true);
}

void app_start_kernel(void) {
    vTaskStartScheduler();
    for (;;) { __asm__ volatile ("bkpt 0"); }   /* should not reach here */
}
