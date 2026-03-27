#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "main.h"
#include "printf/printf.h"
#include "debug/log.h"
#include "mission_manager/mission_manager.h"
#include "SD_logging/log_service.h"
#include "state_estimation/state.h"
#include "state_exchange.h"
#include "spi_drivers/gnss_radio_master.h"
#include "sensors_init.h"
#include "command.pb.h"
#include "telemetry.pb.h"
#include "downlink.pb.h"
#include "status.pb.h"
#include "common.pb.h"
#include "rp/codec.h"

static flight_state_t last_logged_flight_state = IDLE;
static bool flight_header_logged = false;
static uint32_t flight_magic = 0U;
static uint32_t flight_counter = 0U;

#define TELEMETRY_INTERVAL_MS 100   /* 10 Hz */
#define STATUS_INTERVAL_MS    1000  /* 1 Hz */
#define GNSS_STATS_INTERVAL_MS 5000 /* 5 s */

static uint32_t radio_tx_count = 0;
static uint32_t radio_rx_count = 0;
static uint32_t cmd_rx_count = 0;

static void log_flight_header_if_ready(uint32_t timestamp_us);
static void log_flight_state_if_changed(flight_state_t flight_state,
                                        uint32_t timestamp);
static void handle_state_command(const tvr_StateCommand *cmd,
                                 flight_state_t *flight_state);
static void send_telemetry(const state_t *st, const control_output_t *ctrl,
                           flight_state_t flight_state);
static void send_status(flight_state_t flight_state);

void mission_manager_task_start(void *argument) {
    flight_state_t flight_state = IDLE;
    state_exchange_publish_flight_state(flight_state);

    const TickType_t timeout = pdMS_TO_TICKS(100);

    TickType_t last_stats_tick = xTaskGetTickCount();

    DLOG_PRINT("[MM] Task started\r\n");

    for (;;) {
        uint32_t flags = 0;
        xTaskNotifyWaitIndexed(0, 0, UINT32_MAX, &flags, timeout);

        log_service_try_init();

        state_t current_state = {0};
        state_exchange_get_state(&current_state);

        log_flight_header_if_ready(current_state.u_s);

        /* ── Radio RX: decode FlightCommand ── */
        if (flags & GNSS_RADIO_MSG_READY_FLAG) {
            uint8_t spi_msg[GNSS_RADIO_MESSAGE_MAX_LEN];

            while (gnss_radio_dequeue(spi_msg)) {
                tvr_FlightCommand decoded = tvr_FlightCommand_init_zero;

                rp_packet_decode_result_t dec = rp_packet_decode(
                    spi_msg,
                    GNSS_RADIO_MESSAGE_MAX_LEN,
                    tvr_FlightCommand_fields,
                    &decoded
                );

                if (dec.status == RP_CODEC_OK) {
                    radio_rx_count++;

                    DLOG_PRINT("[RADIO] Decoded OK, which_payload=%d\r\n",
                    (int)decoded.which_payload);

                    if (decoded.which_payload == tvr_FlightCommand_state_cmd_tag) {
                        DLOG_PRINT("[RADIO] StateCommand, type=%d\r\n",
                        (int)decoded.payload.state_cmd.type);
                    }

                    switch (decoded.which_payload) {
                        case tvr_FlightCommand_state_cmd_tag:
                            cmd_rx_count++;
                            handle_state_command(
                                &decoded.payload.state_cmd, &flight_state);
                            break;

                        case tvr_FlightCommand_set_pid_gains_tag:
                            cmd_rx_count++;
                            /* TODO: apply PID gains to controller */
                            break;

                        case tvr_FlightCommand_set_reference_tag:
                            cmd_rx_count++;
                            /* TODO: apply reference setpoints */
                            break;

                        case tvr_FlightCommand_set_config_tag:
                            cmd_rx_count++;
                            /* TODO: apply vehicle config */
                            break;

                        default:
                            break;
                    }
                }
            }
        }

        /* ── Periodic downlink ── */
        TickType_t now = xTaskGetTickCount();

        /* 10 Hz telemetry */
        static TickType_t last_telem_tick = 0;
        if ((now - last_telem_tick) >= pdMS_TO_TICKS(TELEMETRY_INTERVAL_MS)) {
            last_telem_tick = now;
            control_output_t ctrl = {0};
            state_exchange_get_control_output(&ctrl);
            send_telemetry(&current_state, &ctrl, flight_state);
        }

        /* 1 Hz system status */
        static TickType_t last_status_tick = 0;
        if ((now - last_status_tick) >= pdMS_TO_TICKS(STATUS_INTERVAL_MS)) {
            last_status_tick = now;
            send_status(flight_state);
        }

        /* ── Periodic GNSS stats (debug) ── */
        if ((now - last_stats_tick) >= pdMS_TO_TICKS(GNSS_STATS_INTERVAL_MS)) {
            last_stats_tick = now;
            uint32_t gps_cnt, radio_cnt, err_cnt;
            gnss_radio_get_stats(&gps_cnt, &radio_cnt, &err_cnt);
            DLOG_PRINT("[GNSS] g=%lu r=%lu e=%lu\r\n",
                       (unsigned long)gps_cnt,
                       (unsigned long)radio_cnt,
                       (unsigned long)err_cnt);
        }

        log_service_periodic_flush();
        log_flight_state_if_changed(flight_state, current_state.u_s);
        state_exchange_publish_flight_state(flight_state);
    }
}

static uint32_t next_flight_magic(void) {
    uint32_t seed = HAL_GetTick() ^ 0x5A5AA5A5U;
    if (seed == 0U) {
        seed = 0xA5A5A5A5U;
    }
    return seed;
}

static void log_flight_header_if_ready(uint32_t timestamp_us) {
    if (flight_header_logged || !log_service_ready()) {
        return;
    }

    if (flight_magic == 0U) {
        flight_magic = next_flight_magic();
        flight_counter = HAL_GetTick();
    }

    log_service_log_flight_header(timestamp_us, flight_magic, flight_counter);
    flight_header_logged = true;
}

static void log_flight_state_if_changed(flight_state_t flight_state,
                                        uint32_t timestamp) {
    if (flight_state == last_logged_flight_state) {
        return;
    }

    log_service_log_event(LOG_EVENT_CODE_FLIGHT_STATE, (uint16_t)flight_state,
                          timestamp);
    last_logged_flight_state = flight_state;
}

static void handle_state_command(const tvr_StateCommand *cmd,
                                 flight_state_t *flight_state) {
    switch (cmd->type) {
        case tvr_StateCommand_Type_CMD_ARM:
        {
            if (*flight_state == IDLE) {
                /* Disarm and invalidate the previous test so the controls task
                 * re-runs the full startup sequence before going live. */
                state_exchange_publish_armed(false);
                state_exchange_publish_startup_test_complete(false);
                state_exchange_publish_rearm_request(true);
                DLOG_PRINT("[MM] ARM: rearm sequence requested\r\n");
            } else {
                DLOG_PRINT("[MM] ARM rejected: flight_state=%d\r\n",
                           (int)*flight_state);
            }
            break;
        }

        case tvr_StateCommand_Type_CMD_LAUNCH:
            if (*flight_state == IDLE) {
                *flight_state = RISE;
            }
            break;

        case tvr_StateCommand_Type_CMD_ABORT:
            *flight_state = IDLE;
            DLOG_PRINT("[MM] Abort: ESC off, flight_state -> IDLE\r\n");
            break;

        case tvr_StateCommand_Type_CMD_LAND:
            if (*flight_state == HOVER) {
                *flight_state = LOWER;
            }
            break;

        case tvr_StateCommand_Type_CMD_NONE:
        default:
            break;
    }
}

static void send_telemetry(const state_t *st, const control_output_t *ctrl,
                           flight_state_t flight_state) {
    tvr_Downlink dl = tvr_Downlink_init_zero;
    dl.which_payload = tvr_Downlink_telemetry_tag;

    tvr_TelemetryState *telem = &dl.payload.telemetry;
    telem->timestamp_ms = HAL_GetTick();

    telem->has_position = true;
    telem->position.x = st->pos[0];
    telem->position.y = st->pos[1];
    telem->position.z = st->pos[2];

    telem->has_velocity = true;
    telem->velocity.x = st->vel[0];
    telem->velocity.y = st->vel[1];
    telem->velocity.z = st->vel[2];

    telem->has_attitude = true;
    telem->attitude.w = st->q_bn.w;
    telem->attitude.x = st->q_bn.x;
    telem->attitude.y = st->q_bn.y;
    telem->attitude.z = st->q_bn.z;

    telem->has_angular_rate = true;
    telem->angular_rate.x = st->omega_b[0];
    telem->angular_rate.y = st->omega_b[1];
    telem->angular_rate.z = st->omega_b[2];

    telem->flight_state = (tvr_FlightState)flight_state;
    telem->thrust_cmd   = ctrl->T_cmd;
    telem->gimbal_x     = ctrl->theta_x_cmd;
    telem->gimbal_y     = ctrl->theta_y_cmd;

    uint8_t pkt[RP_PACKET_MAX_SIZE];
    rp_packet_encode_result_t enc = rp_packet_encode(
        pkt, sizeof(pkt), tvr_Downlink_fields, &dl);

    if (enc.status == RP_CODEC_OK) {
        if (gnss_radio_send(pkt, (uint16_t)enc.written)) {
            radio_tx_count++;
        }
    }
}

static void send_status(flight_state_t flight_state) {
    const sensors_init_status_t *sensors = sensors_get_init_status();

    tvr_Downlink dl = tvr_Downlink_init_zero;
    dl.which_payload = tvr_Downlink_status_tag;

    tvr_SystemStatus *s = &dl.payload.status;
    s->timestamp_ms   = HAL_GetTick();
    s->uptime_ms      = HAL_GetTick();
    s->flight_state   = (tvr_FlightState)flight_state;
    s->accel_ok       = sensors->accel_ok;
    s->gyro_ok        = sensors->gyro_ok;
    s->baro1_ok       = sensors->baro_ok;
    s->baro2_ok       = sensors->baro2_ok;
    s->gps_connected  = !gnss_gps_queue_empty();
    s->radio_tx_count = radio_tx_count;
    s->radio_rx_count = radio_rx_count;
    s->cmd_rx_count   = cmd_rx_count;

    uint8_t pkt[RP_PACKET_MAX_SIZE];
    rp_packet_encode_result_t enc = rp_packet_encode(
        pkt, sizeof(pkt), tvr_Downlink_fields, &dl);

    if (enc.status == RP_CODEC_OK) {
        if (gnss_radio_send(pkt, (uint16_t)enc.written)) {
            radio_tx_count++;
        }
    }
}
