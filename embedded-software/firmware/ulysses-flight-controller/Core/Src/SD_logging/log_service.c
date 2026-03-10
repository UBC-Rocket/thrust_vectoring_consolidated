#include "SD_logging/log_service.h"

#include "SD_logging/log_writer.h"
#include "log_records/log_records.h"
#include "mission_manager/mission_manager.h"
#include "state_estimation/state.h"
#include "main.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define LOG_FLUSH_INTERVAL 100U

extern bool g_sd_card_initialized;

static bool s_logger_initialised = false;
static uint32_t s_flush_counter = 0U;

void log_service_try_init(void)
{
    if (s_logger_initialised || !g_sd_card_initialized) {
        return;
    }

    if (log_writer_init()) {
        s_logger_initialised = true;
    }
}

bool log_service_ready(void)
{
    return s_logger_initialised && log_writer_ready();
}

void log_service_log_state(const state_t *state, flight_state_t flight_state)
{
    if (!log_service_ready() || state == NULL) {
        return;
    }

    log_record_state_snapshot_t snapshot = {
        .timestamp_us = state->u_s,
        .q_w = state->q_bn.w,
        .q_x = state->q_bn.x,
        .q_y = state->q_bn.y,
        .q_z = state->q_bn.z,
        .altitude_m = state->pos[2],
        .vel_n_mps = state->vel[0],
        .vel_e_mps = state->vel[1],
        .vel_d_mps = state->vel[2],
        .flight_state = (uint8_t)flight_state,
        .estop_active = (uint8_t)(flight_state == E_STOP),
        .reserved = 0U
    };

    log_writer_append_record(LOG_RECORD_TYPE_state_snapshot,
                             &snapshot,
                             sizeof(snapshot));
}

void log_service_log_flight_header(uint32_t timestamp_us,
                                   uint32_t flight_magic,
                                   uint32_t flight_counter)
{
    if (!log_service_ready()) {
        return;
    }

    log_record_flight_header_t header = {
        .timestamp_us = timestamp_us,
        .flight_magic = flight_magic,
        .flight_counter = flight_counter
    };

    log_writer_append_record(LOG_RECORD_TYPE_flight_header,
                             &header,
                             sizeof(header));
}

void log_service_log_accel_sample(uint32_t timestamp_us,
                                  float ax_mps2, float ay_mps2, float az_mps2)
{
    if (!log_service_ready()) {
        return;
    }

    log_record_accel_sample_t record = {
        .timestamp_us = timestamp_us,
        .ax_mps2 = ax_mps2,
        .ay_mps2 = ay_mps2,
        .az_mps2 = az_mps2,
    };

    log_writer_append_record(LOG_RECORD_TYPE_accel_sample,
                             &record,
                             sizeof(record));
}

void log_service_log_gyro_sample(uint32_t timestamp_us,
                                 float gx_rad_s, float gy_rad_s, float gz_rad_s)
{
    if (!log_service_ready()) {
        return;
    }

    log_record_gyro_sample_t record = {
        .timestamp_us = timestamp_us,
        .gx_rad_s = gx_rad_s,
        .gy_rad_s = gy_rad_s,
        .gz_rad_s = gz_rad_s,
    };

    log_writer_append_record(LOG_RECORD_TYPE_gyro_sample,
                             &record,
                             sizeof(record));
}

void log_service_log_baro_sample(uint32_t timestamp_us,
                                 int32_t temp_centi,
                                 int32_t pressure_centi,
                                 uint32_t seq)
{
    if (!log_service_ready()) {
        return;
    }

    log_record_baro_sample_t record = {
        .timestamp_us = timestamp_us,
        .temp_centi = temp_centi,
        .pressure_centi = pressure_centi,
        .seq = seq
    };

    log_writer_append_record(LOG_RECORD_TYPE_baro_sample,
                             &record,
                             sizeof(record));
}

void log_service_log_baro2_sample(uint32_t timestamp_us,
                                  int32_t temp_centi,
                                  int32_t pressure_centi,
                                  uint32_t seq)
{
    if (!log_service_ready()) {
        return;
    }

    log_record_baro2_sample_t record = {
        .timestamp_us = timestamp_us,
        .temp_centi = temp_centi,
        .pressure_centi = pressure_centi,
        .seq = seq
    };

    log_writer_append_record(LOG_RECORD_TYPE_baro2_sample,
                             &record,
                             sizeof(record));
}

void log_service_log_event(uint16_t event_code, uint16_t data, uint32_t timestamp_us)
{
    if (!log_service_ready()) {
        return;
    }

    log_record_event_t event = {
        .timestamp_us = timestamp_us,
        .event_code = event_code,
        .data_u16 = data
    };

    log_writer_append_record(LOG_RECORD_TYPE_event,
                             &event,
                             sizeof(event));
}

void log_service_periodic_flush(void)
{
    if (!log_service_ready()) {
        return;
    }

    s_flush_counter++;
    if (s_flush_counter >= LOG_FLUSH_INTERVAL) {
        log_writer_flush();
        s_flush_counter = 0U;
    }
}
