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
#define LOG_EVENT_CODE_RADIO_RX 0x0003U

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

#endif /* LOG_SERVICE_H */
