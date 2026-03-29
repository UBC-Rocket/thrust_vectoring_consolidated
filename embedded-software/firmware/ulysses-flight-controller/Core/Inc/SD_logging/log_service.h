#ifndef LOG_SERVICE_H
#define LOG_SERVICE_H

#include <stdbool.h>
#include <stdint.h>
#include "log_records/log_records.h"
#include "mission_manager/mission_manager.h"
#include "state_estimation/state.h"

/* ── Event codes ── */
#define LOG_EVENT_CODE_ESTOP 0x0001U
#define LOG_EVENT_CODE_FLIGHT_STATE 0x0002U
#define LOG_EVENT_CODE_ARM_STATE 0x0004U  /**< data_u16: 1=armed, 0=disarmed */

bool log_service_ready(void);
void log_service_mark_ready(void);

/* ── Auto-generated: one log function per record type ──
 * Each takes a const pointer to the corresponding packed struct.
 * Functions for records with enable==0 compile to no-ops.
 */
#define DECLARE_LOG_FN_(id, name, fields, enable) \
    void log_service_log_##name(const log_record_##name##_t *record);
LOG_RECORD_LIST(DECLARE_LOG_FN_)
#undef DECLARE_LOG_FN_

/**
 * @brief Convenience wrapper that maps state_t + flight_state_t to a
 *        log_record_state_snapshot_t and calls log_service_log_state_snapshot().
 */
void log_service_log_state(const state_t *state, flight_state_t flight_state);

#endif /* LOG_SERVICE_H */
