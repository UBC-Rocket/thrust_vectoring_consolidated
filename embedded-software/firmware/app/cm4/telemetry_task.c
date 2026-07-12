/**
 * @file    telemetry_task.c
 * @brief   CM4 downlink: encode EKF state + FC health as tvr_Downlink and
 *          transmit over the RFD900 radio (UART7).
 *
 * This is the transmit counterpart to mission_manager_task's receive path.
 * It sends two message kinds on the same radio:
 *   - tvr_TelemetryState  at ~10 Hz  (position/velocity/attitude/rate + control)
 *   - tvr_SystemStatus    at ~1 Hz   (sensor health + radio link counters)
 *
 * Both are wrapped in a tvr_Downlink envelope and framed by rp_packet_encode:
 *   nanopb protobuf  ->  append CRC16-CCITT (LE)  ->  COBS + trailing 0x00.
 * That is exactly the frame format the ground station's receiver splits on
 * (0x00 delimiter) and feeds to rp_packet_decode. See libs/rocket-protocol.
 *
 * The radio is owned entirely by CM4 (UART7); no cross-core / SRAM4 shared
 * memory is involved on the transmit path.
 *
 * UBC Rocket, 2026
 */

#include "app/tasks.h"
#include "app/state_exchange.h"
#include "app/sensors_init.h"

#include "state_estimation/state.h"
#include "controls/flight_controller.h"

#include "dev_radio_rfd900.h"

#include "rp/codec.h"
#include "downlink.pb.h"          /* tvr_Downlink, tvr_TelemetryState, tvr_SystemStatus */

#include "io_sys/io_timestamp.h"
#include "io_sys/io_debug.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stdint.h>

/* --------------------------------------------------------------------------
 * Tunables
 * -------------------------------------------------------------------------- */

/* Base loop period. Telemetry goes out every tick (10 Hz); status goes out
 * once every STATUS_EVERY_N_TICKS ticks (1 Hz). */
#define TELEM_PERIOD_MS        100U
#define STATUS_EVERY_N_TICKS    10U

/* --------------------------------------------------------------------------
 * State
 * -------------------------------------------------------------------------- */

static uint32_t s_tx_count;   /* frames successfully handed to the radio */

/* --------------------------------------------------------------------------
 * Helpers
 * -------------------------------------------------------------------------- */

/* Map the app-side flight-state enum (state_exchange.h) to the proto enum
 * (common.pb.h). The proto enum predates the new FSM, so it has no ARMED or
 * LANDED — those fold onto the nearest proto value. DESCENT maps to LOWER.
 * If the ground display needs the full new FSM, regenerate the proto enum to
 * mirror app_flight_state_t; nothing else here changes. */
static tvr_FlightState to_proto_flight_state(app_flight_state_t s) {
    switch (s) {
        case APP_FLIGHT_IDLE:    return tvr_FlightState_FLIGHT_STATE_IDLE;
        case APP_FLIGHT_ARMED:   return tvr_FlightState_FLIGHT_STATE_IDLE;
        case APP_FLIGHT_RISE:    return tvr_FlightState_FLIGHT_STATE_RISE;
        case APP_FLIGHT_DESCENT: return tvr_FlightState_FLIGHT_STATE_LOWER;
        case APP_FLIGHT_LANDED:  return tvr_FlightState_FLIGHT_STATE_IDLE;
        case APP_FLIGHT_ESTOP:   return tvr_FlightState_FLIGHT_STATE_ESTOP;
        default:                 return tvr_FlightState_FLIGHT_STATE_IDLE;
    }
}

/* Encode one Downlink envelope and push it to the radio. Returns true if the
 * radio accepted the bytes. */
static bool send_downlink(radio_rfd900_t *radio, const tvr_Downlink *dl) {
    uint8_t packet[RP_PACKET_MAX_SIZE];
    rp_packet_encode_result_t r =
        rp_packet_encode(packet, sizeof(packet), tvr_Downlink_fields, dl);
    if (r.status != RP_CODEC_OK) {
#ifdef DEBUG_TEXT_CONSOLE
        io_debug_printf("[tx] encode FAIL status=%d\r\n", (int)r.status);
#endif
        return false;
    }
    bool ok = radio_rfd900_send(radio, packet, r.written);
    if (ok) {
        s_tx_count++;
    }
    return ok;
}

/* 10 Hz: current EKF state + control output. */
static void send_telemetry(radio_rfd900_t *radio) {
    state_t st;
    (void)state_exchange_get_state(&st);
    app_flight_state_t fs = APP_FLIGHT_IDLE;
    (void)state_exchange_get_flight_state(&fs);
    control_output_t co = {0};
    (void)state_exchange_get_control_output(&co);

    tvr_Downlink dl = tvr_Downlink_init_zero;
    dl.which_payload = tvr_Downlink_telemetry_tag;
    tvr_TelemetryState *t = &dl.payload.telemetry;

    t->timestamp_ms = (uint32_t)(io_timestamp_us() / 1000U);

    t->has_position = true;
    t->position.x = st.pos[0]; t->position.y = st.pos[1]; t->position.z = st.pos[2];
    t->has_velocity = true;
    t->velocity.x = st.vel[0]; t->velocity.y = st.vel[1]; t->velocity.z = st.vel[2];
    t->has_attitude = true;
    t->attitude.w = st.q_bn.w; t->attitude.x = st.q_bn.x;
    t->attitude.y = st.q_bn.y; t->attitude.z = st.q_bn.z;
    t->has_angular_rate = true;
    t->angular_rate.x = st.omega_b[0];
    t->angular_rate.y = st.omega_b[1];
    t->angular_rate.z = st.omega_b[2];

    t->flight_state = to_proto_flight_state(fs);
    t->thrust_cmd = co.T_cmd;
    t->gimbal_x   = co.theta_x_cmd;
    t->gimbal_y   = co.theta_y_cmd;

    (void)send_downlink(radio, &dl);
}

/* 1 Hz: health + link counters. accel_ok/gyro_ok/... are proven by the
 * driver singleton being non-NULL (init succeeded, incl. WHO_AM_I checks). */
static void send_status(radio_rfd900_t *radio) {
    app_flight_state_t fs = APP_FLIGHT_IDLE;
    (void)state_exchange_get_flight_state(&fs);

    tvr_Downlink dl = tvr_Downlink_init_zero;
    dl.which_payload = tvr_Downlink_status_tag;
    tvr_SystemStatus *s = &dl.payload.status;

    const uint32_t now_ms = (uint32_t)(io_timestamp_us() / 1000U);
    s->timestamp_ms = now_ms;
    s->uptime_ms    = now_ms;
    s->flight_state = to_proto_flight_state(fs);

    const sensors_t *sn = sensors_handles();
    s->accel_ok      = (sn != NULL && sn->bmi088   != NULL);
    s->gyro_ok       = (sn != NULL && sn->icm40609 != NULL);
    s->baro1_ok      = (sn != NULL && sn->baro     != NULL);
    s->baro2_ok      = false;                 /* single baro on this board */

    s->radio_tx_count = s_tx_count;
    s->radio_rx_count = mission_manager_radio_rx_count();
    s->cmd_rx_count   = mission_manager_cmd_rx_count();

    (void)send_downlink(radio, &dl);
}

/* --------------------------------------------------------------------------
 * Task entry
 * -------------------------------------------------------------------------- */

void task_telemetry(void *arg) {
    (void)arg;

    const sensors_t *sn = sensors_handles();
    radio_rfd900_t *radio = (sn != NULL) ? sn->radio : NULL;

    uint32_t tick = 0;
    for (;;) {
        if (radio != NULL) {
            bool armed = false;
            (void)state_exchange_get_armed(&armed);
            if (armed) {
                send_telemetry(radio);
            }
            if ((tick % STATUS_EVERY_N_TICKS) == 0U) {
                send_status(radio);
            }
        }
        tick++;
        vTaskDelay(pdMS_TO_TICKS(TELEM_PERIOD_MS));
    }
}
