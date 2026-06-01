/**
 * @file    mission_manager_task.c
 * @brief   CM4 task: decode radio commands, run flight state machine.
 *
 * Loop:
 *   1) Block on a FreeRTOS direct-to-task notify (radio ISR fires
 *      MM_NOTIFY_RADIO when a framed packet has been deposited in the
 *      RFD900 ring) with a bounded timeout.
 *   2) Drain all available frames, decode each as tvr_FlightCommand,
 *      and step the flight state machine.
 *   3) Apply heartbeat watchdog: if no valid command has arrived within
 *      MM_HEARTBEAT_TIMEOUT_MS while ARMED, fall back to IDLE/disarm.
 *   4) Re-publish flight_state + armed every iteration so CM7 always
 *      has a fresh view (state_exchange seqlock is cheap).
 *
 * State machine (mirrors the deprecated H5 implementation, mapped to
 * the new app_flight_state_t enum exposed via state_exchange.h):
 *
 *   IDLE   --CMD_ARM-->     ARMED   (publishes armed=true)
 *   ARMED  --CMD_LAUNCH-->  RISE
 *   ARMED  --CMD_ABORT-->   IDLE    (publishes armed=false)
 *   RISE   --CMD_LAND-->    DESCENT
 *   RISE   --CMD_ABORT-->   ESTOP
 *   DESCENT--landed cond--> LANDED  (TODO: real touchdown detector;
 *                                    placeholder uses CMD_NONE timeout
 *                                    once CM7 controls signal landed)
 *   any    --CMD_ABORT-->   ESTOP   (terminal; requires reboot)
 *   ARMED  --no heartbeat--> IDLE   (watchdog disarm)
 *
 * ESTOP is sticky: once entered, only a reset clears it.
 *
 * UBC Rocket, 2026
 */

#include "app/tasks.h"
#include "app/state_exchange.h"
#include "app/sensors_init.h"

#include "dev_radio_rfd900.h"

#include "rp/codec.h"
#include "command.pb.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stdint.h>

/* --------------------------------------------------------------------------
 * Tunables
 * -------------------------------------------------------------------------- */

/* Notify bit reserved for the radio RX ISR -> mission manager path.
 * Picked to be distinct from SENSORS_NOTIFY_* (state-estimation's task);
 * lives in this task's notification value space so there is no overlap
 * concern across tasks, but we still use a high bit to keep audit easy. */
#define MM_NOTIFY_RADIO   (1U << 8)

/* How long we block before forcing a state-machine tick. Caps watchdog
 * latency and gives us a regular publish cadence even if the radio is
 * silent. */
#define MM_LOOP_TIMEOUT_MS    100U

/* Heartbeat watchdog: if we haven't seen a valid FlightCommand in this
 * many ms while ARMED or in flight (RISE/DESCENT), drop to IDLE and
 * disarm. The deprecated H5 build did not implement this; the spec
 * for the new system calls for ~500 ms. */
#define MM_HEARTBEAT_TIMEOUT_MS  500U

/* Max frames we'll drain in one pass. Bounded so a flood can't
 * starve the watchdog tick. The radio ring is 8 deep, so this also
 * mirrors a "drain everything" semantic in the common case. */
#define MM_MAX_DRAIN_PER_TICK  8U

/* --------------------------------------------------------------------------
 * State
 * -------------------------------------------------------------------------- */

static app_flight_state_t s_flight_state;
static bool               s_armed;
static TickType_t         s_last_cmd_tick;   /* of last valid command */

/* --------------------------------------------------------------------------
 * Helpers
 * -------------------------------------------------------------------------- */

static inline void mm_publish(void) {
    state_exchange_publish_flight_state(s_flight_state);
    state_exchange_publish_armed(s_armed);
}

static inline void mm_set_armed(bool armed) {
    s_armed = armed;
    state_exchange_publish_armed(armed);
}

static inline void mm_set_state(app_flight_state_t s) {
    s_flight_state = s;
    state_exchange_publish_flight_state(s);
}

/* Apply a StateCommand to the FSM. Returns true if it was accepted (and
 * thus refreshes the heartbeat watchdog). */
static bool mm_apply_state_cmd(tvr_StateCommand_Type cmd) {
    /* ESTOP is terminal — reject everything. */
    if (s_flight_state == APP_FLIGHT_ESTOP) {
        return false;
    }

    switch (cmd) {
        case tvr_StateCommand_Type_CMD_ARM:
            /* Only IDLE may arm. */
            if (s_flight_state == APP_FLIGHT_IDLE) {
                mm_set_state(APP_FLIGHT_ARMED);
                mm_set_armed(true);
                return true;
            }
            return false;

        case tvr_StateCommand_Type_CMD_LAUNCH:
            /* Launch from ARMED (or IDLE+armed as a courtesy, mirroring
             * the deprecated which allowed IDLE -> RISE directly). */
            if (s_flight_state == APP_FLIGHT_ARMED) {
                mm_set_state(APP_FLIGHT_RISE);
                return true;
            }
            return false;

        case tvr_StateCommand_Type_CMD_ABORT:
            /* From flight, abort is an E-stop. From pre-flight, drop
             * back to IDLE and disarm. */
            if (s_flight_state == APP_FLIGHT_RISE ||
                s_flight_state == APP_FLIGHT_DESCENT) {
                mm_set_state(APP_FLIGHT_ESTOP);
            } else {
                mm_set_state(APP_FLIGHT_IDLE);
            }
            mm_set_armed(false);
            return true;

        case tvr_StateCommand_Type_CMD_LAND:
            if (s_flight_state == APP_FLIGHT_RISE) {
                mm_set_state(APP_FLIGHT_DESCENT);
                return true;
            }
            return false;

        case tvr_StateCommand_Type_CMD_NONE:
        default:
            /* Treat as a keep-alive: refresh heartbeat but no transition. */
            return true;
    }
}

/* Process one decoded FlightCommand. Returns true if it should refresh
 * the heartbeat watchdog. */
static bool mm_handle_command(const tvr_FlightCommand *cmd) {
    switch (cmd->which_payload) {
        case tvr_FlightCommand_state_cmd_tag:
            return mm_apply_state_cmd(cmd->payload.state_cmd.type);

        case tvr_FlightCommand_set_pid_gains_tag:
        case tvr_FlightCommand_set_reference_tag:
        case tvr_FlightCommand_set_config_tag:
            /* TODO: forward to CM7 via state_exchange when those slots
             * exist in the new tree (deprecated had publish_pid_gains
             * / publish_flight_reference / publish_vehicle_config). For
             * now we accept them as valid traffic so the heartbeat
             * watchdog stays satisfied during a tuning session. */
            return true;

        default:
            return false;
    }
}

/* Drain the radio ring and dispatch every decoded command. Returns true
 * if at least one valid command was processed. */
static bool mm_drain_radio(radio_rfd900_t *radio) {
    if (radio == NULL) return false;

    radio_frame_t frames[MM_MAX_DRAIN_PER_TICK];
    size_t n = radio_rfd900_drain_frames(radio, frames, MM_MAX_DRAIN_PER_TICK);
    bool any_valid = false;

    for (size_t i = 0; i < n; ++i) {
        tvr_FlightCommand decoded = tvr_FlightCommand_init_zero;
        rp_packet_decode_result_t dec = rp_packet_decode(
            frames[i].payload, frames[i].len,
            tvr_FlightCommand_fields, &decoded);

        if (dec.status != RP_CODEC_OK) continue;

        if (mm_handle_command(&decoded)) {
            any_valid = true;
        }
    }

    return any_valid;
}

/* Heartbeat watchdog. If we've been ARMED (or further along) and no
 * valid command has arrived recently, disarm and fall back to IDLE.
 * Does not apply in IDLE (nothing to disarm) or in ESTOP/LANDED
 * (terminal-ish; no further action). */
static void mm_check_watchdog(TickType_t now) {
    const bool watch =
        s_flight_state == APP_FLIGHT_ARMED   ||
        s_flight_state == APP_FLIGHT_RISE    ||
        s_flight_state == APP_FLIGHT_DESCENT;
    if (!watch) return;

    const TickType_t elapsed = now - s_last_cmd_tick;
    if (elapsed >= pdMS_TO_TICKS(MM_HEARTBEAT_TIMEOUT_MS)) {
        /* In-flight timeout escalates; pre-flight just disarms. */
        if (s_flight_state == APP_FLIGHT_RISE ||
            s_flight_state == APP_FLIGHT_DESCENT) {
            mm_set_state(APP_FLIGHT_ESTOP);
        } else {
            mm_set_state(APP_FLIGHT_IDLE);
        }
        mm_set_armed(false);
        /* Reset so we don't spin re-firing the timeout. */
        s_last_cmd_tick = now;
    }
}

/* --------------------------------------------------------------------------
 * Task entry
 * -------------------------------------------------------------------------- */

void task_mission_manager(void *arg) {
    (void)arg;

    /* Wire the radio ISR to fire MM_NOTIFY_RADIO into THIS task. We do
     * this here (not in app_init_cm4.c) so the binding happens exactly
     * once the task is running and its handle is valid via
     * xTaskGetCurrentTaskHandle(). */
    const sensors_t *sensors = sensors_handles();
    if (sensors != NULL && sensors->radio != NULL) {
        radio_rfd900_set_notify(
            sensors->radio,
            (io_task_handle_t)xTaskGetCurrentTaskHandle(),
            MM_NOTIFY_RADIO);
    }

    /* Boot state: IDLE, disarmed. Publish so CM7 has a defined view
     * before any radio traffic arrives. */
    s_flight_state  = APP_FLIGHT_IDLE;
    s_armed         = false;
    s_last_cmd_tick = xTaskGetTickCount();
    mm_publish();

    for (;;) {
        uint32_t notify_value = 0;
        (void)xTaskNotifyWait(0, UINT32_MAX, &notify_value,
                              pdMS_TO_TICKS(MM_LOOP_TIMEOUT_MS));

        const TickType_t now = xTaskGetTickCount();

        if (notify_value & MM_NOTIFY_RADIO) {
            if (sensors != NULL && mm_drain_radio(sensors->radio)) {
                s_last_cmd_tick = now;
            }
        }

        mm_check_watchdog(now);
        mm_publish();
    }
}
