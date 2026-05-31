#include "SD_logging/log_service.h"
#include "SD_logging/log_writer.h"

static bool s_logger_ready = false;

bool log_service_ready(void)
{
    return s_logger_ready && log_writer_ready();
}

void log_service_mark_ready(void)
{
    s_logger_ready = true;
}

/* ── Auto-generated: one log function per record type ──
 * When `enable` is 0 (compile-time constant), the compiler eliminates the
 * entire function body. With -O2, call sites are dead-code eliminated too. */
#define DEFINE_LOG_FN_(id, name, fields, enable) \
    void log_service_log_##name(const log_record_##name##_t *record) { \
        if (!(enable) || !log_service_ready()) return; \
        log_writer_append_record(LOG_RECORD_TYPE_##name, \
                                 record, sizeof(*record)); \
    }
LOG_RECORD_LIST(DEFINE_LOG_FN_)
#undef DEFINE_LOG_FN_

/* ── Manual: maps state_t to state_snapshot record ── */
void log_service_log_state(const state_t *state, flight_state_t flight_state)
{
    if (state == NULL) return;

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

    log_service_log_state_snapshot(&snapshot);
}
