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
        .timestamp_us    = state->u_s,
        .q_w             = state->q_bn.w,
        .q_x             = state->q_bn.x,
        .q_y             = state->q_bn.y,
        .q_z             = state->q_bn.z,
        .altitude_m      = state->pos[2],
        .pos_n_m         = state->pos[0],
        .pos_e_m         = state->pos[1],
        .vel_n_mps       = state->vel[0],
        .vel_e_mps       = state->vel[1],
        .vel_d_mps       = state->vel[2],
        .omega_bx_rad_s  = state->omega_b[0],
        .omega_by_rad_s  = state->omega_b[1],
        .omega_bz_rad_s  = state->omega_b[2],
        .flight_state    = (uint8_t)flight_state,
        .estop_active    = (uint8_t)(flight_state == E_STOP),
        .reserved        = 0U
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

void log_service_log_calibration(uint32_t timestamp_us,
                                 float accel_bias_x,
                                 float accel_bias_y,
                                 float accel_bias_z,
                                 float gyro_bias_x,
                                 float gyro_bias_y,
                                 float gyro_bias_z,
                                 uint16_t calibration_samples)
{
    if (!log_service_ready()) {
        return;
    }

    log_record_calibration_t record = {
        .timestamp_us        = timestamp_us,
        .accel_bias_x        = accel_bias_x,
        .accel_bias_y        = accel_bias_y,
        .accel_bias_z        = accel_bias_z,
        .gyro_bias_x         = gyro_bias_x,
        .gyro_bias_y         = gyro_bias_y,
        .gyro_bias_z         = gyro_bias_z,
        .calibration_samples = calibration_samples
    };

    log_writer_append_record(LOG_RECORD_TYPE_calibration,
                             &record,
                             sizeof(record));
}

void log_service_log_control_output(uint32_t timestamp_us,
                                    float T_cmd,
                                    float theta_x_cmd,
                                    float theta_y_cmd,
                                    float tau_gim_x,
                                    float tau_gim_y,
                                    float tau_gim_z,
                                    float tau_thrust,
                                    float phi_x,
                                    float phi_y,
                                    float phi_z,
                                    float z_pid_integral,
                                    float z_ref,
                                    float vz_ref)
{
    if (!log_service_ready()) {
        return;
    }

    log_record_control_output_t record = {
        .timestamp_us   = timestamp_us,
        .T_cmd          = T_cmd,
        .theta_x_cmd    = theta_x_cmd,
        .theta_y_cmd    = theta_y_cmd,
        .tau_gim_x      = tau_gim_x,
        .tau_gim_y      = tau_gim_y,
        .tau_gim_z      = tau_gim_z,
        .tau_thrust     = tau_thrust,
        .phi_x          = phi_x,
        .phi_y          = phi_y,
        .phi_z          = phi_z,
        .z_pid_integral = z_pid_integral,
        .z_ref          = z_ref,
        .vz_ref         = vz_ref
    };

    log_writer_append_record(LOG_RECORD_TYPE_control_output,
                             &record,
                             sizeof(record));
}

void log_service_log_gps_fix(uint32_t timestamp_us,
                              double latitude,
                              double longitude,
                              float altitude_msl,
                              float ground_speed,
                              float course,
                              float hdop,
                              uint32_t time_of_week_ms,
                              uint8_t fix_quality,
                              uint8_t num_satellites)
{
    if (!log_service_ready()) {
        return;
    }

    log_record_gps_fix_t record = {
        .timestamp_us    = timestamp_us,
        .latitude        = latitude,
        .longitude       = longitude,
        .altitude_msl    = altitude_msl,
        .ground_speed    = ground_speed,
        .course          = course,
        .hdop            = hdop,
        .time_of_week_ms = time_of_week_ms,
        .fix_quality     = fix_quality,
        .num_satellites  = num_satellites,
        .reserved        = 0U
    };

    log_writer_append_record(LOG_RECORD_TYPE_gps_fix,
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

void log_service_pid_gains( uint32_t timestamp_us, 
                    bool has_attitude_kp,
                    float attitude_kp_x,
                    float attitude_kp_y,
                    float attitude_kp_z,
                    bool has_attitude_kd,
                    float attitude_kd_x,
                    float attitude_kd_y,
                    float attitude_kd_z,
                    float z_kp,
                    float z_ki,
                    float z_kd,
                    float z_integral_limit) {

    if (!log_service_ready()) {
        return;
    }

    log_record_pid_gains_t pid = {
        .timestamp_us = timestamp_us,
        .has_attitude_kp = has_attitude_kp,
        .attitude_kp_x = attitude_kp_x,
        .attitude_kp_y = attitude_kp_y,
        .attitude_kp_z = attitude_kp_z,
        .has_attitude_kd = has_attitude_kd,
        .attitude_kd_x = attitude_kd_x,
        .attitude_kd_y = attitude_kd_y,
        .attitude_kd_z = attitude_kd_z,
        .z_kp = z_kp,
        .z_ki = z_ki,
        .z_kd = z_kd,
        .z_integral_limit = z_integral_limit
    };

    log_writer_append_record(LOG_RECORD_TYPE_pid_gains,
                                &pid,
                                sizeof(pid));

}

void log_service_reference( uint32_t timestamp_us,
                            float z_ref,
                            float vz_ref,
                            bool has_q_ref,
                            float q_ref_w,
                            float q_ref_x,
                            float q_ref_y,
                            float q_ref_z) {

    if (!log_service_ready()) {
        return;
    }

    log_record_reference_t reference = {
        .timestamp_us = timestamp_us,
        .z_ref = z_ref,
        .vz_ref = vz_ref,
        .has_q_ref = has_q_ref,
        .q_ref_w = q_ref_w,
        .q_ref_x = q_ref_x,
        .q_ref_y = q_ref_y,
        .q_ref_z = q_ref_z
    };

    log_writer_append_record(
            LOG_RECORD_TYPE_reference,
            &reference,
            sizeof(reference));

}

void log_service_configuration( uint32_t timestamp_us,
                                float mass,
                                float T_min,
                                float T_max,
                                float theta_min,
                                float theta_max) {

    if (!log_service_ready()) {
        return;
    }

    log_record_configuration_t configuration = {
        .timestamp_us = timestamp_us,
        .mass = mass,
        .T_min = T_min,
        .T_max = T_max,
        .theta_min = theta_min,
        .theta_max = theta_max
    };

    log_writer_append_record(
            LOG_RECORD_TYPE_configuration,
            &configuration,
            sizeof(configuration));
}
