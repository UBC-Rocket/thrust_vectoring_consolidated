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
#include "timestamp.h"

#define TELEMETRY_INTERVAL_MS 100   /* 10 Hz */
#define STATUS_INTERVAL_MS    1000  /* 1 Hz */
#define GNSS_STATS_INTERVAL_MS 5000 /* 5 s */

static uint32_t radio_tx_count = 0;
static uint32_t radio_rx_count = 0;
static uint32_t cmd_rx_count = 0;

static void handle_state_command(const tvr_StateCommand *cmd,
                                 flight_state_t *flight_state);
static void handle_pid_gains(const tvr_SetPidGains *pid);
static void handle_reference(const tvr_SetReference *reference);
static void handle_configuration(const tvr_SetConfig *configuration);
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

        state_t current_state = {0};
        state_exchange_get_state(&current_state);

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
                            /* TODO: apply pid gains */
                            handle_pid_gains(
                                &decoded.payload.set_pid_gains);
                            break;

                        case tvr_FlightCommand_set_reference_tag:
                            cmd_rx_count++;
                            /* TODO: apply reference setpoints */
                            handle_reference(
                                    &decoded.payload.set_reference);
                            break;

                        case tvr_FlightCommand_set_config_tag:
                            cmd_rx_count++;
                            /* TODO: apply vehicle config */
                            handle_configuration(
                                &decoded.payload.set_config);
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

        state_exchange_publish_flight_state(flight_state);
    }
}

static void handle_pid_gains(const tvr_SetPidGains *pid) {
    if (pid == NULL) {
        return;
    }

    log_service_log_pid_gains(&(log_record_pid_gains_t){
        .timestamp_us = timestamp_us(),
        .has_attitude_kp = pid->has_attitude_kp,
        .attitude_kp_x = pid->has_attitude_kp ? pid->attitude_kp.x : 0.0f,
        .attitude_kp_y = pid->has_attitude_kp ? pid->attitude_kp.y : 0.0f,
        .attitude_kp_z = pid->has_attitude_kp ? pid->attitude_kp.z : 0.0f,
        .has_attitude_kd = pid->has_attitude_kd,
        .attitude_kd_x = pid->has_attitude_kd ? pid->attitude_kd.x : 0.0f,
        .attitude_kd_y = pid->has_attitude_kd ? pid->attitude_kd.y : 0.0f,
        .attitude_kd_z = pid->has_attitude_kd ? pid->attitude_kd.z : 0.0f,
        .z_kp = pid->z_kp,
        .z_ki = pid->z_ki,
        .z_kd = pid->z_kd,
        .z_integral_limit = pid->z_integral_limit,
    });
}

static void handle_reference(const tvr_SetReference *reference) {
    if (reference == NULL) {
        return;
    }

    log_service_log_reference(&(log_record_reference_t){
        .timestamp_us = timestamp_us(),
        .z_ref = reference->z_ref,
        .vz_ref = reference->vz_ref,
        .has_q_ref = reference->has_q_ref,
        .q_ref_w = reference->has_q_ref ? reference->q_ref.w : 0.0f,
        .q_ref_x = reference->has_q_ref ? reference->q_ref.x : 0.0f,
        .q_ref_y = reference->has_q_ref ? reference->q_ref.y : 0.0f,
        .q_ref_z = reference->has_q_ref ? reference->q_ref.z : 0.0f,
    });
}

static void handle_configuration(const tvr_SetConfig *configuration) {
    if (configuration == NULL) {
        return;
    }

    log_service_log_configuration(&(log_record_configuration_t){
        .timestamp_us = timestamp_us(),
        .mass = configuration->mass,
        .T_min = configuration->T_min,
        .T_max = configuration->T_max,
        .theta_min = configuration->theta_min,
        .theta_max = configuration->theta_max,
    });
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
        {
            if (*flight_state == IDLE) {
                *flight_state = RISE;
            }
            break;
        }

        case tvr_StateCommand_Type_CMD_ABORT:
        {
            *flight_state = IDLE;
            DLOG_PRINT("[MM] Abort: ESC off, flight_state -> IDLE\r\n");
            break;
        }

        case tvr_StateCommand_Type_CMD_LAND:
        {
            if (*flight_state == HOVER) {
                *flight_state = LOWER;
            }
            break;
        }

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
    const uint32_t tx_timestamp_us = telem->timestamp_ms * 1000U;

    uint8_t pkt[RP_PACKET_MAX_SIZE];
    rp_packet_encode_result_t enc = rp_packet_encode(
        pkt, sizeof(pkt), tvr_Downlink_fields, &dl);

    if (enc.status == RP_CODEC_OK) {
        if (gnss_radio_send(pkt, (uint16_t)enc.written)) {
            log_service_log_radio_telemetry(&(log_record_radio_telemetry_t){
                .timestamp_us   = tx_timestamp_us,
                .timestamp_ms   = telem->timestamp_ms,
                .position_x     = telem->position.x,
                .position_y     = telem->position.y,
                .position_z     = telem->position.z,
                .velocity_x     = telem->velocity.x,
                .velocity_y     = telem->velocity.y,
                .velocity_z     = telem->velocity.z,
                .attitude_w     = telem->attitude.w,
                .attitude_x     = telem->attitude.x,
                .attitude_y     = telem->attitude.y,
                .attitude_z     = telem->attitude.z,
                .angular_rate_x = telem->angular_rate.x,
                .angular_rate_y = telem->angular_rate.y,
                .angular_rate_z = telem->angular_rate.z,
                .thrust_cmd     = telem->thrust_cmd,
                .gimbal_x       = telem->gimbal_x,
                .gimbal_y       = telem->gimbal_y,
                .flight_state   = (uint8_t)telem->flight_state,
            });
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
    const uint32_t tx_timestamp_us = s->timestamp_ms * 1000U;

    uint8_t pkt[RP_PACKET_MAX_SIZE];
    rp_packet_encode_result_t enc = rp_packet_encode(
        pkt, sizeof(pkt), tvr_Downlink_fields, &dl);

    if (enc.status == RP_CODEC_OK) {
        if (gnss_radio_send(pkt, (uint16_t)enc.written)) {
            log_service_log_radio_status(&(log_record_radio_status_t){
                .timestamp_us   = tx_timestamp_us,
                .timestamp_ms   = s->timestamp_ms,
                .uptime_ms      = s->uptime_ms,
                .radio_tx_count = s->radio_tx_count,
                .radio_rx_count = s->radio_rx_count,
                .cmd_rx_count   = s->cmd_rx_count,
                .flight_state   = (uint8_t)s->flight_state,
                .accel_ok       = s->accel_ok,
                .gyro_ok        = s->gyro_ok,
                .baro1_ok       = s->baro1_ok,
                .baro2_ok       = s->baro2_ok,
                .gps_connected  = s->gps_connected,
            });
            radio_tx_count++;
        }
    }
}
