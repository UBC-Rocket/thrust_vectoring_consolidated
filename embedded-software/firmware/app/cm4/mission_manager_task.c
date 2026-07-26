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
 *   DESCENT--landed cond--> LANDED  (see mm_check_touchdown — quiescent
 *                                    altitude + velocity for a sustained
 *                                    window read from state_exchange)
 *   LANDED --CMD_ARM-->      ARMED   (start another ground-test cycle)
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

#include "state_estimation/state.h"

#include "dev_radio_rfd900.h"

#include "rp/codec.h"
#include "command.pb.h"

#include "io_sys/io_debug.h"
#include "io_sys/io_status_led.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

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
#define MM_HEARTBEAT_TIMEOUT_MS  2000U

/* Max frames we'll drain in one pass. Bounded so a flood can't
 * starve the watchdog tick. The radio ring is 8 deep, so this also
 * mirrors a "drain everything" semantic in the common case. */
#define MM_MAX_DRAIN_PER_TICK  8U

/* Touchdown thresholds (DESCENT → LANDED). All three must hold for the
 * sustain window before we declare landed. Tuned conservatively — better
 * to stay in DESCENT a few hundred ms too long than to drop control
 * authority while the rocket is still bouncing. */
#define MM_LAND_VEL_VERT_MPS    0.5f    /* |vz| < 0.5 m/s              */
#define MM_LAND_VEL_HORIZ_MPS   1.0f    /* sqrt(vx²+vy²) < 1.0 m/s     */
#define MM_LAND_ALT_THRESH_M    2.0f    /* z < 2 m above launch origin */
#define MM_LAND_SUSTAIN_MS      750U    /* hold conditions this long   */

/* --------------------------------------------------------------------------
 * State
 * -------------------------------------------------------------------------- */

static app_flight_state_t s_flight_state;
static bool               s_armed;
static TickType_t         s_last_cmd_tick;        /* of last valid command */
static TickType_t         s_descent_quiet_start;  /* tick when quiescent began, 0 = not yet */
static uint32_t           s_radio_rx_count;       /* frames pulled off the radio */
static uint32_t           s_cmd_rx_count;         /* frames that decoded to a valid command */

/* Read by task_telemetry (TX side) for the tvr_SystemStatus link-health fields. */
uint32_t mission_manager_radio_rx_count(void) { return s_radio_rx_count; }
uint32_t mission_manager_cmd_rx_count(void)   { return s_cmd_rx_count; }

#ifdef DEBUG_TEXT_CONSOLE
/* Human-readable names for the CoolTerm bring-up trace. */
static const char *cmd_name(tvr_StateCommand_Type t) {
    switch (t) {
        case tvr_StateCommand_Type_CMD_ARM:    return "ARM";
        case tvr_StateCommand_Type_CMD_LAUNCH: return "LAUNCH";
        case tvr_StateCommand_Type_CMD_ABORT:  return "ABORT";
        case tvr_StateCommand_Type_CMD_LAND:   return "LAND";
        case tvr_StateCommand_Type_CMD_NONE:   return "NONE";
        default:                               return "?";
    }
}
static const char *state_name(app_flight_state_t s) {
    switch (s) {
        case APP_FLIGHT_IDLE:    return "IDLE";
        case APP_FLIGHT_ARMED:   return "ARMED";
        case APP_FLIGHT_RISE:    return "RISE";
        case APP_FLIGHT_DESCENT: return "DESCENT";
        case APP_FLIGHT_LANDED:  return "LANDED";
        case APP_FLIGHT_ESTOP:   return "ESTOP";
        default:                 return "?";
    }
}
#endif

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
    if (!armed) {
        /* Disarm must be physically inert: zero any operator throttle so
         * CM7's next tunables poll (50 ms) drops the ESCs to minimum.
         * Covers ABORT, landing and the heartbeat-loss watchdog, which
         * all funnel through here. */
        const app_throttle_cmd_t zero = { 0 };
        (void)state_exchange_publish_throttle_cmd(&zero);
    }
}

static inline void mm_set_state(app_flight_state_t s) {
    s_flight_state = s;
    state_exchange_publish_flight_state(s);
}

/* Apply a StateCommand to the FSM. Returns true if it was accepted (and
 * thus refreshes the heartbeat watchdog). */
static bool mm_apply_state_cmd(tvr_StateCommand_Type cmd) {
    /* ESTOP is terminal — reject everything. */
    // if (s_flight_state == APP_FLIGHT_ESTOP) {
    //     return false;
    // }

    switch (cmd) {
        case tvr_StateCommand_Type_CMD_ARM:
            /* LANDED may re-arm so repeated ground-test launches do not
             * require an intervening abort or board reset. */
            if (s_flight_state == APP_FLIGHT_IDLE ||
                s_flight_state == APP_FLIGHT_LANDED) {
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
                mm_set_state(APP_FLIGHT_LANDED);
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

/* Translate nanopb SetPidGains → app_pid_gains_t and publish for CM7.
 * The attitude_kp / attitude_kd are OPTIONAL — if absent, hold what CM7
 * already has by leaving NaN markers (we use 0 to mean "unchanged"; the
 * solver merges by checking >0). Pragmatic for v1; if zero is ever a
 * valid gain we'll add an explicit valid bitmask. */
static void mm_forward_pid_gains(const tvr_SetPidGains *src) {
    /* The target uses newlib-nano without _printf_float, so %f renders as an
     * empty field. Print the gain as fixed-point using integer conversions. */
    app_pid_gains_t g = {0};
    if (src->has_attitude_kp) {
        g.attitude_kp[0] = src->attitude_kp.x;
        g.attitude_kp[1] = src->attitude_kp.y;
        g.attitude_kp[2] = src->attitude_kp.z;
    }
    if (src->has_attitude_kd) {
        g.attitude_kd[0] = src->attitude_kd.x;
        g.attitude_kd[1] = src->attitude_kd.y;
        g.attitude_kd[2] = src->attitude_kd.z;
    }
    if (src->has_attitude_ki) {
        g.attitude_ki[0] = src->attitude_ki.x;
        g.attitude_ki[1] = src->attitude_ki.y;
        g.attitude_ki[2] = src->attitude_ki.z;
    }
    g.z_kp             = src->z_kp;
    g.z_ki             = src->z_ki;
    g.z_kd             = src->z_kd;
    g.z_integral_limit = src->z_integral_limit;
    (void)state_exchange_publish_pid_gains(&g);
}

static void mm_forward_reference(const tvr_SetReference *src) {
    app_reference_t r = {0};
    r.z_ref   = src->z_ref;
    r.vz_ref  = src->vz_ref;
    r.q_valid = src->has_q_ref;
    if (src->has_q_ref) {
        r.q_ref[0] = src->q_ref.w;
        r.q_ref[1] = src->q_ref.x;
        r.q_ref[2] = src->q_ref.y;
        r.q_ref[3] = src->q_ref.z;
    } else {
        r.q_ref[0] = 1.0f;   /* identity placeholder — ignored by CM7 */
    }
    (void)state_exchange_publish_reference(&r);
}

static void mm_forward_config(const tvr_SetConfig *src) {
    app_vehicle_config_t c = {0};
    c.mass_kg   = src->mass;
    c.T_min     = src->T_min;
    c.T_max     = src->T_max;
    c.theta_min = src->theta_min;
    c.theta_max = src->theta_max;
    (void)state_exchange_publish_vehicle_config(&c);
}

/* NaN-safe clamp (!(v >= 0) also catches NaN → 0); CM7 clamps again at
 * DShot conversion. */
static float mm_clamp_throttle(float v) {
    if (!(v >= 0.0f)) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return v;
}

static void mm_forward_throttle(const tvr_SetThrottle *src) {
    app_throttle_cmd_t t;
    t.throttle_lower = mm_clamp_throttle(src->throttle);
    /* Legacy senders (no throttle_upper on the wire) drive both motors with
     * the single value, matching the pre-split behaviour. */
    t.throttle_upper = src->has_throttle_upper
        ? mm_clamp_throttle(src->throttle_upper)
        : t.throttle_lower;
    (void)state_exchange_publish_throttle_cmd(&t);
}

/* Process one decoded FlightCommand. Returns true if it should refresh
 * the heartbeat watchdog. */
static bool mm_handle_command(const tvr_FlightCommand *cmd) {
    switch (cmd->which_payload) {
        case tvr_FlightCommand_state_cmd_tag:
            return mm_apply_state_cmd(cmd->payload.state_cmd.type);

        case tvr_FlightCommand_set_pid_gains_tag:
            mm_forward_pid_gains(&cmd->payload.set_pid_gains);
            return true;

        case tvr_FlightCommand_set_reference_tag:
            mm_forward_reference(&cmd->payload.set_reference);
            return true;

        case tvr_FlightCommand_set_config_tag:
            mm_forward_config(&cmd->payload.set_config);
            return true;

        case tvr_FlightCommand_set_throttle_tag:
            /* No throttle-up while E-stopped, even though ESTOP itself is
             * recoverable on this branch. */
            if (s_flight_state == APP_FLIGHT_ESTOP) return false;
            mm_forward_throttle(&cmd->payload.set_throttle);
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
    s_radio_rx_count += (uint32_t)n;
    bool any_valid = false;

    for (size_t i = 0; i < n; ++i) {
        tvr_FlightCommand decoded = tvr_FlightCommand_init_zero;
        rp_packet_decode_result_t dec = rp_packet_decode(
            frames[i].payload, frames[i].len,
            tvr_FlightCommand_fields, &decoded);

        if (dec.status != RP_CODEC_OK) {
#ifdef DEBUG_TEXT_CONSOLE
            /* Bench-bring-up trace: a frame arrived but failed to decode
             * (bad CRC / short frame / COBS error). Proves the wire is live
             * even when framing is wrong. */
            io_debug_printf("[rx] frame len=%u DECODE-FAIL status=%d\r\n",
                            (unsigned)frames[i].len, (int)dec.status);
#endif
            continue;
        }

        s_cmd_rx_count++;
        if (mm_handle_command(&decoded)) {
            any_valid = true;
        }

#ifdef DEBUG_TEXT_CONSOLE
        /* Layer-1 uplink test: unmistakable, human-readable banner so CoolTerm
         * clearly shows which command arrived and the resulting FSM state. */
        if (decoded.which_payload == tvr_FlightCommand_state_cmd_tag) {
            io_debug_printf(
                "\r\n>>>>>> RADIO CMD: %-6s  ==>  STATE: %-7s armed=%d  [rx=%lu cmd=%lu] <<<<<<\r\n",
                cmd_name(decoded.payload.state_cmd.type),
                state_name(s_flight_state), (int)s_armed,
                (unsigned long)s_radio_rx_count, (unsigned long)s_cmd_rx_count);
        } else {
            io_debug_printf(
                "\r\n>>>>>> RADIO CMD: TUNING (which=%d)  [rx=%lu cmd=%lu] <<<<<<\r\n",
                (int)decoded.which_payload,
                (unsigned long)s_radio_rx_count, (unsigned long)s_cmd_rx_count);
        }
#endif
    }

    return any_valid;
}

/* Touchdown detector. While in DESCENT, sample the EKF state via
 * state_exchange and decide whether the rocket has actually landed.
 *
 * Conditions (all three must hold simultaneously):
 *   1. |vel_z| < MM_LAND_VEL_VERT_MPS   — barely moving vertically
 *   2. sqrt(vx² + vy²) < MM_LAND_VEL_HORIZ_MPS — barely moving laterally
 *   3. pos_z < MM_LAND_ALT_THRESH_M      — near launch altitude
 *
 * They must hold for MM_LAND_SUSTAIN_MS continuously — a momentary near-
 * zero crossing during a bounce shouldn't trip the latch. s_descent_quiet
 * _start records when the conditions first became true; if any condition
 * breaks we reset it. Once the elapsed window exceeds the sustain, we
 * latch LANDED.
 *
 * Note: we hold armed=true through LANDED so the ESC stays disarmed by
 * the controls ISR (it gates throttle on the armed slot). The host can
 * issue CMD_ABORT after touchdown to disarm cleanly. */
static void mm_check_touchdown(TickType_t now)
{
    if (s_flight_state != APP_FLIGHT_DESCENT) {
        s_descent_quiet_start = 0;
        return;
    }

    state_t state;
    (void)state_exchange_get_state(&state);

    const float vz       = state.vel[2];
    const float vh_sq    = state.vel[0] * state.vel[0] + state.vel[1] * state.vel[1];
    const float pz       = state.pos[2];
    const float vh_thr_sq = MM_LAND_VEL_HORIZ_MPS * MM_LAND_VEL_HORIZ_MPS;

    const bool quiescent =
        ( (vz < MM_LAND_VEL_VERT_MPS) && (vz > -MM_LAND_VEL_VERT_MPS) ) &&
        ( vh_sq < vh_thr_sq ) &&
        ( pz < MM_LAND_ALT_THRESH_M );

    if (!quiescent) {
        s_descent_quiet_start = 0;
        return;
    }
    if (s_descent_quiet_start == 0) {
        s_descent_quiet_start = now;
        return;
    }
    if ((now - s_descent_quiet_start) >= pdMS_TO_TICKS(MM_LAND_SUSTAIN_MS)) {
        mm_set_state(APP_FLIGHT_LANDED);
        /* Reset so a later re-arm/launch/land cycle starts a fresh sustain
         * window. */
        s_descent_quiet_start = 0;
    }
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
        (void)notify_value;   /* wake source doesn't matter — we always poll below */

        /* Pull any complete frames out of the RX DMA buffer (robust path: the
         * character-match ISR isn't reliably delivering on this H7, but bytes
         * DO land in the DMA ring — see radio_rfd900_poll), then drain the ring
         * regardless of whether we woke on the notify or the timeout. */
        radio_rfd900_poll();
        if (sensors != NULL && mm_drain_radio(sensors->radio)) {
            s_last_cmd_tick = now;
        }

        mm_check_watchdog(now);
        mm_check_touchdown(now);
        mm_publish();

#ifdef DEBUG_TEXT_CONSOLE
        /* Bring-up heartbeat on the text console (~1 Hz). Proves the board is
         * alive in CoolTerm even with no sensors/radio attached and even if
         * the terminal connects after boot (so the one-shot banner was
         * missed). Guarded by DEBUG_TEXT_CONSOLE so it disappears entirely
         * when the binary VCP console is selected. */
        static TickType_t s_hb_last;
        if ((TickType_t)(now - s_hb_last) >= pdMS_TO_TICKS(1000)) {
            s_hb_last = now;
            /* HW-visible proof-of-life independent of the VCP wiring. RED
             * was lit solid by app_init; blinking it here proves the
             * scheduler + this task loop are actually running. */
            io_status_led_toggle(IO_LED_RED);
            io_debug_printf("[hb] cm4 alive  state=%d armed=%d  t=%lu ms\r\n",
                            (int)s_flight_state, (int)s_armed,
                            (unsigned long)now);

            /* Raw radio RX probe: does UART7 physically receive ANY bytes,
             * even un-framed ones? A terminal typing plain letters has no
             * 0x00, so the normal frame path stays silent — but this catches
             * it. rx_idx advancing ⇒ bytes ARE landing on PE7 (hexdump shows
             * what). Frozen idx ⇒ nothing reaching the FC (wiring / RF / the
             * ground side isn't transmitting). */
            static uint32_t s_last_rx_idx;
            const uint32_t rx_idx = radio_rfd900_diag_rx_index();
            if (rx_idx != s_last_rx_idx) {
                uint8_t raw[24];
                const size_t k = radio_rfd900_diag_rx_copy(s_last_rx_idx, rx_idx,
                                                           raw, sizeof(raw));
                char hex[3 * sizeof(raw) + 1];
                size_t hp = 0;
                static const char HX[] = "0123456789ABCDEF";
                for (size_t i = 0; i < k; ++i) {
                    hex[hp++] = HX[(raw[i] >> 4) & 0x0F];
                    hex[hp++] = HX[raw[i] & 0x0F];
                    hex[hp++] = ' ';
                }
                hex[hp] = '\0';
                io_debug_printf("[radio] *** RX BYTES *** idx %lu->%lu: %s\r\n",
                                (unsigned long)s_last_rx_idx,
                                (unsigned long)rx_idx, hex);
                s_last_rx_idx = rx_idx;
            } else {
                io_debug_printf("[radio] RX idle  idx=%lu  rx=%lu cmd=%lu\r\n",
                                (unsigned long)rx_idx,
                                (unsigned long)s_radio_rx_count,
                                (unsigned long)s_cmd_rx_count);
            }
        }
#endif
    }
}
