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
static io_intercore_slot_t s_slot_throttle;
static io_intercore_slot_t s_slot_motor_rpm;

/* Monotonic sequence numbers; one value per core, updated by each publisher.
 * Not shared across cores because each slot is only written by one core; the
 * reading core uses the intercore seqlock's own counter for concurrency and
 * the exchanged payload carries its own domain-specific seq.
 *
 * throttle/motor_rpm are the exception: their consumers (CM7 controls poll,
 * CM4 telemetry) DO compare the returned seq across cores to detect fresh
 * publishes, so for those two slots the statics instead cache the intercore
 * seqlock header's own counter (which lives in shared SRAM4 and therefore
 * carries across cores) as of the last publish or last magic-valid read
 * performed on this core. */
static uint32_t s_state_seq;
static uint32_t s_control_seq;
static uint32_t s_flight_seq;
static uint32_t s_armed_seq;
static uint32_t s_pid_gains_seq;
static uint32_t s_reference_seq;
static uint32_t s_vehicle_cfg_seq;
static uint32_t s_throttle_seq;
static uint32_t s_motor_rpm_seq;

static bool s_initialised;

/* Wire format of the throttle / motor-RPM slots. The .shared region is NOLOAD
 * and never zeroed, so before the first publish a reader can find power-on
 * garbage behind an even ("stable") seqlock counter. The magic word lets the
 * getters reject such garbage outright instead of relying on boot ordering —
 * same trick as the log-handoff ring, SD cursor and debug-console ring
 * headers. A garbage float must never become a throttle command. */
#define TUNABLES_MAGIC 0x54554E42U /* "TUNB" */

typedef struct { uint32_t magic; app_throttle_cmd_t v; } throttle_cmd_wire_t;
typedef struct { uint32_t magic; app_motor_rpm_t    v; } motor_rpm_wire_t;

_Static_assert(sizeof(throttle_cmd_wire_t) <= APP_SLOT_THROTTLE_CMD_PAYLOAD,
               "throttle wire struct exceeds its shared-memory slot");
_Static_assert(sizeof(motor_rpm_wire_t)    <= APP_SLOT_MOTOR_RPM_PAYLOAD,
               "motor_rpm wire struct exceeds its shared-memory slot");

/* Test-only access to module-private state. No-op in production builds. */
IO_TEST_HOOK_RW(s_state_seq,     uint32_t, state_exchange_state_seq)
IO_TEST_HOOK_RW(s_control_seq,   uint32_t, state_exchange_control_seq)
IO_TEST_HOOK_RW(s_flight_seq,    uint32_t, state_exchange_flight_seq)
IO_TEST_HOOK_RW(s_armed_seq,     uint32_t, state_exchange_armed_seq)
IO_TEST_HOOK_RW(s_throttle_seq,  uint32_t, state_exchange_throttle_seq)
IO_TEST_HOOK_RW(s_motor_rpm_seq, uint32_t, state_exchange_motor_rpm_seq)
IO_TEST_HOOK_RW(s_initialised,   bool,     state_exchange_initialised)

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
                        sizeof(app_pid_gains_t));
    io_intercore_slot_init(&s_slot_reference,
                        app_shared_slot(APP_SLOT_REFERENCE_OFFSET),
                        sizeof(app_reference_t));
    io_intercore_slot_init(&s_slot_vehicle_cfg,
                        app_shared_slot(APP_SLOT_VEHICLE_CONFIG_OFFSET),
                        sizeof(app_vehicle_config_t));
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
    io_intercore_slot_publish(&s_slot_pid_gains, gains);
    s_pid_gains_seq++;
    return s_pid_gains_seq;
}

uint32_t state_exchange_get_pid_gains(app_pid_gains_t *out) {
    app_pid_gains_t tmp;
    if (io_intercore_slot_read(&s_slot_pid_gains, &tmp, NULL)) {
        if (out) *out = tmp;
    }
    return s_pid_gains_seq;
}

uint32_t state_exchange_publish_reference(const app_reference_t *ref) {
    if (!ref) return s_reference_seq;
    io_intercore_slot_publish(&s_slot_reference, ref);
    s_reference_seq++;
    return s_reference_seq;
}

uint32_t state_exchange_get_reference(app_reference_t *out) {
    app_reference_t tmp;
    if (io_intercore_slot_read(&s_slot_reference, &tmp, NULL)) {
        if (out) *out = tmp;
    }
    return s_reference_seq;
}

uint32_t state_exchange_publish_vehicle_config(const app_vehicle_config_t *cfg) {
    if (!cfg) return s_vehicle_cfg_seq;
    io_intercore_slot_publish(&s_slot_vehicle_cfg, cfg);
    s_vehicle_cfg_seq++;
    return s_vehicle_cfg_seq;
}

uint32_t state_exchange_get_vehicle_config(app_vehicle_config_t *out) {
    app_vehicle_config_t tmp;
    if (io_intercore_slot_read(&s_slot_vehicle_cfg, &tmp, NULL)) {
        if (out) *out = tmp;
    }
    return s_vehicle_cfg_seq;
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
