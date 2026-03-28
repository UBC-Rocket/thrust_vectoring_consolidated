#ifndef LOG_SERVICE_H
#define LOG_SERVICE_H

#include <stdbool.h>
#include <stdint.h>
#include "mission_manager/mission_manager.h"
#include "state_estimation/state.h"

/**
 * @brief Shared event identifiers for log_service_log_event.
 */
#define LOG_EVENT_CODE_ESTOP 0x0001U
#define LOG_EVENT_CODE_FLIGHT_STATE 0x0002U
#define LOG_EVENT_CODE_ARM_STATE 0x0004U  /**< data_u16: 1=armed, 0=disarmed */

/**
 * @brief Perform one-time initialisation of the SD log writer if a card is present.
 */
void log_service_try_init(void);

/**
 * @brief Returns true if the logging backend is ready to accept records.
 */
bool log_service_ready(void);

/**
 * @brief Append the latest navigation state snapshot to the log.
 *
 * Safe to call from the mission manager loop; function is a no-op if logging is disabled.
 */
void log_service_log_state(const state_t *state, flight_state_t flight_state);

void log_service_log_flight_header(uint32_t timestamp_us,
                                   uint32_t flight_magic,
                                   uint32_t flight_counter);

void log_service_log_accel_sample(uint32_t timestamp_us,
                                  float ax_mps2, float ay_mps2, float az_mps2);

void log_service_log_gyro_sample(uint32_t timestamp_us,
                                 float gx_rad_s, float gy_rad_s, float gz_rad_s);

void log_service_log_baro_sample(uint32_t timestamp_us,
                                 int32_t temp_centi,
                                 int32_t pressure_centi,
                                 uint32_t seq);

void log_service_log_baro2_sample(uint32_t timestamp_us,
                                  int32_t temp_centi,
                                  int32_t pressure_centi,
                                  uint32_t seq);

/**
 * @brief Append an event record to the log (e.g., estop, state changes).
 */
void log_service_log_event(uint16_t event_code, uint16_t data, uint32_t timestamp_us);

/**
 * @brief Append a calibration record to the log.
 *
 * Call exactly once, on the first EKF step after startup calibration completes
 * (detected by calibration_samples == STARTUP_CALIBRATION_SAMPLES in se_run_imu_step).
 * accel_bias units: [g] (normalized). gyro_bias units: [rad/s].
 */
void log_service_log_calibration(uint32_t timestamp_us,
                                 float accel_bias_x,
                                 float accel_bias_y,
                                 float accel_bias_z,
                                 float gyro_bias_x,
                                 float gyro_bias_y,
                                 float gyro_bias_z,
                                 uint16_t calibration_samples);

/**
 * @brief Append a control output record to the log.
 *
 * Call at decimated rate from the Controls task (e.g. every 8th 800-Hz cycle
 * = 100 Hz). Logs all control_output_t fields including intermediate torques
 * so attitude controller faults can be separated from allocator faults.
 */
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
                                    float vz_ref);

/**
 * @brief Append a GPS fix record to the log.
 *
 * Call once per valid fix received from the GNSS board. lat/lon are kept
 * as double to preserve full GPS precision. Safe to call from the
 * state_estimation task.
 */
void log_service_log_gps_fix(uint32_t timestamp_us,
                              double latitude,
                              double longitude,
                              float altitude_msl,
                              float ground_speed,
                              float course,
                              float hdop,
                              uint32_t time_of_week_ms,
                              uint8_t fix_quality,
                              uint8_t num_satellites);


/**
 * @brief Append a SetPIDGains record to the log.
 *
 * Call once per valid PID set command recieved from GNSS board.
 */
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
                    float z_integral_limit);


void log_service_reference( uint32_t timestamp_us,
                            float z_ref,
                            float vz_ref,
                            bool has_q_ref,
                            float q_ref_w,
                            float q_ref_x,
                            float q_ref_y,
                            float q_ref_z);

void log_service_configuration( uint32_t timestamp_us,
                                float mass,
                                float T_min,
                                float T_max,
                                float theta_min,
                                float theta_max);

/**
 * @brief Periodic flush helper to limit data loss on power failure.
 */
void log_service_periodic_flush(void);

#endif /* LOG_SERVICE_H */
