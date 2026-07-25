/**
 * @file    state_exchange.c
 * @brief   Cross-core exchange of state / flight state / control output via
 *          intercore seqlock slots in shared SRAM4.
 *
 * UBC Rocket, 2026
 */

#include "app/state_exchange.h"
#include "app/shared_memory.h"
#include "io_intercore_slot/io_intercore_slot.h"
#include "io_sys/io_intercore.h"
#include "io_sys/io_test_hooks.h"

#include <stdatomic.h>
#include <string.h>

/* Per-core descriptors that point into shared memory. */
static io_intercore_slot_t s_slot_state;
static io_intercore_slot_t s_slot_control;
static io_intercore_slot_t s_slot_flight;
static io_intercore_slot_t s_slot_armed;
static io_intercore_slot_t s_slot_pid_gains;
static io_intercore_slot_t s_slot_reference;
static io_intercore_slot_t s_slot_vehicle_cfg;
static io_intercore_slot_t s_slot_readiness;
static io_intercore_slot_t s_slot_throttle;
static io_intercore_slot_t s_slot_motor_rpm;

/* Sequence numbers returned to callers.
 *
 * state/control/flight/armed: bumped locally on each publish. These are
 * per-core statics, so the value is only meaningful on the publishing core;
 * none of their consumers look at the seq (they read unconditionally).
 *
 * pid_gains/reference/vehicle_config: CM7's poll_tunables() DOES compare the
 * returned seq against a remembered value to detect fresh publishes from CM4,
 * so a per-core counter is useless there — on the reading core's image it
 * would sit at 0 forever and the compare would never fire. For these slots
 * (and readiness, which CM7 is also meant to read) the statics instead cache
 * the intercore seqlock header's own counter (which lives in shared SRAM4 and
 * therefore carries across cores) as of the last publish or last magic-valid
 * read performed on this core. */
static uint32_t s_state_seq;
static uint32_t s_control_seq;
static uint32_t s_flight_seq;
static uint32_t s_armed_seq;
static uint32_t s_pid_gains_seq;
static uint32_t s_reference_seq;
static uint32_t s_vehicle_cfg_seq;
static uint32_t s_readiness_seq;
static uint32_t s_throttle_seq;
static uint32_t s_motor_rpm_seq;

static bool s_initialised;

/* Wire format of the tunables slots. The .shared region is NOLOAD and never
 * zeroed, so before the first publish a reader can find power-on garbage
 * behind an even ("stable") seqlock counter. The magic word lets the getters
 * reject such garbage outright instead of relying on boot ordering — same
 * trick as the log-handoff ring, SD cursor and debug-console ring headers. */
#define TUNABLES_MAGIC 0x54554E42U /* "TUNB" */

typedef struct { uint32_t magic; app_pid_gains_t      v; } tunables_pid_gains_wire_t;
typedef struct { uint32_t magic; app_reference_t      v; } tunables_reference_wire_t;
typedef struct { uint32_t magic; app_vehicle_config_t v; } tunables_vehicle_cfg_wire_t;
typedef struct { uint32_t magic; vehicle_readiness_t  v; } readiness_wire_t;
typedef struct { uint32_t magic; app_throttle_cmd_t   v; } throttle_cmd_wire_t;
typedef struct { uint32_t magic; app_motor_rpm_t      v; } motor_rpm_wire_t;

_Static_assert(sizeof(tunables_pid_gains_wire_t)   <= APP_SLOT_PID_GAINS_PAYLOAD,
               "pid_gains wire struct exceeds its shared-memory slot");
_Static_assert(sizeof(tunables_reference_wire_t)   <= APP_SLOT_REFERENCE_PAYLOAD,
               "reference wire struct exceeds its shared-memory slot");
_Static_assert(sizeof(tunables_vehicle_cfg_wire_t) <= APP_SLOT_VEHICLE_CONFIG_PAYLOAD,
               "vehicle_config wire struct exceeds its shared-memory slot");
_Static_assert(sizeof(readiness_wire_t)            <= APP_SLOT_READINESS_PAYLOAD,
               "readiness wire struct exceeds its shared-memory slot");
_Static_assert(sizeof(throttle_cmd_wire_t)         <= APP_SLOT_THROTTLE_CMD_PAYLOAD,
               "throttle wire struct exceeds its shared-memory slot");
_Static_assert(sizeof(motor_rpm_wire_t)            <= APP_SLOT_MOTOR_RPM_PAYLOAD,
               "motor_rpm wire struct exceeds its shared-memory slot");

/* Test-only access to module-private state. No-op in production builds. */
IO_TEST_HOOK_RW(s_state_seq,       uint32_t, state_exchange_state_seq)
IO_TEST_HOOK_RW(s_control_seq,     uint32_t, state_exchange_control_seq)
IO_TEST_HOOK_RW(s_flight_seq,      uint32_t, state_exchange_flight_seq)
IO_TEST_HOOK_RW(s_armed_seq,       uint32_t, state_exchange_armed_seq)
IO_TEST_HOOK_RW(s_pid_gains_seq,   uint32_t, state_exchange_pid_gains_seq)
IO_TEST_HOOK_RW(s_reference_seq,   uint32_t, state_exchange_reference_seq)
IO_TEST_HOOK_RW(s_vehicle_cfg_seq, uint32_t, state_exchange_vehicle_cfg_seq)
IO_TEST_HOOK_RW(s_readiness_seq,   uint32_t, state_exchange_readiness_seq)
IO_TEST_HOOK_RW(s_throttle_seq,    uint32_t, state_exchange_throttle_seq)
IO_TEST_HOOK_RW(s_motor_rpm_seq,   uint32_t, state_exchange_motor_rpm_seq)
IO_TEST_HOOK_RW(s_initialised,     bool,     state_exchange_initialised)

void state_exchange_init(void) {
    if (s_initialised) return;
    io_intercore_slot_init(&s_slot_state,
                        app_shared_slot(APP_SLOT_STATE_OFFSET),
                        sizeof(state_t));
    io_intercore_slot_init(&s_slot_control,
                        app_shared_slot(APP_SLOT_CONTROL_OUTPUT_OFFSET),
                        sizeof(control_output_t));
    io_intercore_slot_init(&s_slot_flight,
                        app_shared_slot(APP_SLOT_FLIGHT_STATE_OFFSET),
                        sizeof(app_flight_state_t));
    io_intercore_slot_init(&s_slot_armed,
                        app_shared_slot(APP_SLOT_ARMED_OFFSET),
                        sizeof(bool));
    io_intercore_slot_init(&s_slot_pid_gains,
                        app_shared_slot(APP_SLOT_PID_GAINS_OFFSET),
                        sizeof(tunables_pid_gains_wire_t));
    io_intercore_slot_init(&s_slot_reference,
                        app_shared_slot(APP_SLOT_REFERENCE_OFFSET),
                        sizeof(tunables_reference_wire_t));
    io_intercore_slot_init(&s_slot_vehicle_cfg,
                        app_shared_slot(APP_SLOT_VEHICLE_CONFIG_OFFSET),
                        sizeof(tunables_vehicle_cfg_wire_t));
    io_intercore_slot_init(&s_slot_readiness,
                        app_shared_slot(APP_SLOT_READINESS_OFFSET),
                        sizeof(readiness_wire_t));
    io_intercore_slot_init(&s_slot_throttle,
                        app_shared_slot(APP_SLOT_THROTTLE_CMD_OFFSET),
                        sizeof(throttle_cmd_wire_t));
    io_intercore_slot_init(&s_slot_motor_rpm,
                        app_shared_slot(APP_SLOT_MOTOR_RPM_OFFSET),
                        sizeof(motor_rpm_wire_t));
    s_initialised = true;
}

uint32_t state_exchange_publish_state(const state_t *state) {
    if (!state) return s_state_seq;
    io_intercore_slot_publish(&s_slot_state, state);
    s_state_seq++;
    io_intercore_signal(IO_IC_STATE_UPDATED);
    return s_state_seq;
}

uint32_t state_exchange_get_state(state_t *state_out) {
    state_t tmp;
    if (io_intercore_slot_read(&s_slot_state, &tmp, NULL)) {
        if (state_out) *state_out = tmp;
    }
    return s_state_seq;
}

uint32_t state_exchange_publish_control_output(const control_output_t *out) {
    if (!out) return s_control_seq;
    io_intercore_slot_publish(&s_slot_control, out);
    s_control_seq++;
    io_intercore_signal(IO_IC_CONTROL_OUTPUT_READY);
    return s_control_seq;
}

uint32_t state_exchange_get_control_output(control_output_t *out) {
    control_output_t tmp;
    if (io_intercore_slot_read(&s_slot_control, &tmp, NULL)) {
        if (out) *out = tmp;
    }
    return s_control_seq;
}

uint32_t state_exchange_publish_flight_state(app_flight_state_t s) {
    io_intercore_slot_publish(&s_slot_flight, &s);
    s_flight_seq++;
    io_intercore_signal(IO_IC_FLIGHT_STATE_UPDATED);
    return s_flight_seq;
}

uint32_t state_exchange_get_flight_state(app_flight_state_t *out) {
    app_flight_state_t tmp;
    if (io_intercore_slot_read(&s_slot_flight, &tmp, NULL)) {
        if (out) *out = tmp;
    }
    return s_flight_seq;
}

uint32_t state_exchange_publish_armed(bool armed) {
    io_intercore_slot_publish(&s_slot_armed, &armed);
    s_armed_seq++;
    return s_armed_seq;
}

uint32_t state_exchange_get_armed(bool *armed_out) {
    bool tmp = false;
    if (io_intercore_slot_read(&s_slot_armed, &tmp, NULL)) {
        if (armed_out) *armed_out = tmp;
    }
    return s_armed_seq;
}

uint32_t state_exchange_publish_pid_gains(const app_pid_gains_t *gains) {
    if (!gains) return s_pid_gains_seq;
    tunables_pid_gains_wire_t w = { .magic = TUNABLES_MAGIC, .v = *gains };
    io_intercore_slot_publish(&s_slot_pid_gains, &w);
    /* At-rest (even) header counter after our own publish. Each slot has a
     * single writer, so this read cannot race another write. */
    s_pid_gains_seq = s_slot_pid_gains.hdr->seq;
    return s_pid_gains_seq;
}

uint32_t state_exchange_get_pid_gains(app_pid_gains_t *out) {
    tunables_pid_gains_wire_t w;
    uint32_t seq;
    if (io_intercore_slot_read(&s_slot_pid_gains, &w, &seq) &&
        w.magic == TUNABLES_MAGIC) {
        if (out) *out = w.v;
        s_pid_gains_seq = seq;
    }
    return s_pid_gains_seq;
}

uint32_t state_exchange_publish_reference(const app_reference_t *ref) {
    if (!ref) return s_reference_seq;
    tunables_reference_wire_t w = { .magic = TUNABLES_MAGIC, .v = *ref };
    io_intercore_slot_publish(&s_slot_reference, &w);
    s_reference_seq = s_slot_reference.hdr->seq;
    return s_reference_seq;
}

uint32_t state_exchange_get_reference(app_reference_t *out) {
    tunables_reference_wire_t w;
    uint32_t seq;
    if (io_intercore_slot_read(&s_slot_reference, &w, &seq) &&
        w.magic == TUNABLES_MAGIC) {
        if (out) *out = w.v;
        s_reference_seq = seq;
    }
    return s_reference_seq;
}

uint32_t state_exchange_publish_vehicle_config(const app_vehicle_config_t *cfg) {
    if (!cfg) return s_vehicle_cfg_seq;
    tunables_vehicle_cfg_wire_t w = { .magic = TUNABLES_MAGIC, .v = *cfg };
    io_intercore_slot_publish(&s_slot_vehicle_cfg, &w);
    s_vehicle_cfg_seq = s_slot_vehicle_cfg.hdr->seq;
    return s_vehicle_cfg_seq;
}

uint32_t state_exchange_get_vehicle_config(app_vehicle_config_t *out) {
    tunables_vehicle_cfg_wire_t w;
    uint32_t seq;
    if (io_intercore_slot_read(&s_slot_vehicle_cfg, &w, &seq) &&
        w.magic == TUNABLES_MAGIC) {
        if (out) *out = w.v;
        s_vehicle_cfg_seq = seq;
    }
    return s_vehicle_cfg_seq;
}

uint32_t state_exchange_publish_readiness(const vehicle_readiness_t *r) {
    if (!r) return s_readiness_seq;
    readiness_wire_t w = { .magic = TUNABLES_MAGIC, .v = *r };
    io_intercore_slot_publish(&s_slot_readiness, &w);
    s_readiness_seq = s_slot_readiness.hdr->seq;
    return s_readiness_seq;
}

uint32_t state_exchange_get_readiness(vehicle_readiness_t *out) {
    readiness_wire_t w;
    uint32_t seq;
    if (io_intercore_slot_read(&s_slot_readiness, &w, &seq) &&
        w.magic == TUNABLES_MAGIC) {
        if (out) *out = w.v;
        s_readiness_seq = seq;
    }
    return s_readiness_seq;
}

uint32_t state_exchange_publish_throttle_cmd(const app_throttle_cmd_t *cmd) {
    if (!cmd) return s_throttle_seq;
    throttle_cmd_wire_t w = { .magic = TUNABLES_MAGIC, .v = *cmd };
    io_intercore_slot_publish(&s_slot_throttle, &w);
    s_throttle_seq = s_slot_throttle.hdr->seq;
    return s_throttle_seq;
}

uint32_t state_exchange_get_throttle_cmd(app_throttle_cmd_t *out) {
    throttle_cmd_wire_t w;
    uint32_t seq;
    if (io_intercore_slot_read(&s_slot_throttle, &w, &seq) &&
        w.magic == TUNABLES_MAGIC) {
        if (out) *out = w.v;
        s_throttle_seq = seq;
    }
    return s_throttle_seq;
}

uint32_t state_exchange_publish_motor_rpm(const app_motor_rpm_t *rpm) {
    if (!rpm) return s_motor_rpm_seq;
    motor_rpm_wire_t w = { .magic = TUNABLES_MAGIC, .v = *rpm };
    io_intercore_slot_publish(&s_slot_motor_rpm, &w);
    s_motor_rpm_seq = s_slot_motor_rpm.hdr->seq;
    return s_motor_rpm_seq;
}

uint32_t state_exchange_get_motor_rpm(app_motor_rpm_t *out) {
    motor_rpm_wire_t w;
    uint32_t seq;
    if (io_intercore_slot_read(&s_slot_motor_rpm, &w, &seq) &&
        w.magic == TUNABLES_MAGIC) {
        if (out) *out = w.v;
        s_motor_rpm_seq = seq;
    }
    return s_motor_rpm_seq;
}
