/**
 * @file    state_exchange.h
 * @brief   Cross-core exchange of state, flight state and control output.
 *
 * Backed by the intercore seqlock slots living in shared SRAM4. A writer on
 * one core calls *publish; a reader on either core calls *get. Each field has
 * a monotonic sequence number that can be used to detect staleness.
 *
 * UBC Rocket, 2026
 */
#ifndef APP_STATE_EXCHANGE_H
#define APP_STATE_EXCHANGE_H

#include <stdbool.h>
#include <stdint.h>

#include "state_estimation/state.h"
#include "controls/flight_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Flight-state enum was previously in mission_manager.h. Duplicated here as a
 * small independent enum so both cores can see it without pulling the whole
 * mission manager into CM7. */
typedef enum {
    APP_FLIGHT_IDLE = 0,
    APP_FLIGHT_ARMED,
    APP_FLIGHT_RISE,
    APP_FLIGHT_DESCENT,
    APP_FLIGHT_LANDED,
    APP_FLIGHT_ESTOP,
} app_flight_state_t;

/* ---------------------------------------------------------------------------
 * Tunable-from-radio payloads — CM4 (mission_manager) writes, CM7 (controls)
 * reads. Plain C structs (not the nanopb-generated tvr_* ones) so CM7
 * doesn't need to link the rocket-protocol nanopb tree. mission_manager
 * converts inbound tvr_FlightCommand variants into these before publishing.
 *
 * Layout mirrors the proto fields one-for-one (see tvr/command.proto).
 * --------------------------------------------------------------------------- */

typedef struct {
    /* Attitude gains — diagonals of 3x3 Kp/Kd in controls_attitude_config. */
    float attitude_kp[3];
    float attitude_kd[3];

    /* Z-axis altitude PID. */
    float z_kp;
    float z_ki;
    float z_kd;
    float z_integral_limit;
} app_pid_gains_t;

typedef struct {
    float q_ref[4];   /* w, x, y, z — desired attitude (body-to-nav)        */
    float z_ref;      /* desired height [m]                                 */
    float vz_ref;     /* desired z velocity [m/s]                           */
    bool  q_valid;    /* false → ISR holds its previous attitude reference  */
} app_reference_t;

typedef struct {
    float mass_kg;    /* vehicle mass — feeds thrust PID's `m`              */
    float T_min;      /* min thrust [N]                                     */
    float T_max;      /* max thrust [N]                                     */
    float theta_min;  /* min gimbal angle [rad]                             */
    float theta_max;  /* max gimbal angle [rad]                             */
} app_vehicle_config_t;

/**
 * @brief Attach to the shared-memory slots. Must be called by every core
 *        before any publish/get call. Safe to call multiple times.
 */
void state_exchange_init(void);

/* ---------- state (CM4 writes, CM7 reads) ---------- */
uint32_t state_exchange_publish_state(const state_t *state);
uint32_t state_exchange_get_state(state_t *state_out);

/* ---------- control output (CM7 writes, CM4 reads) ---------- */
uint32_t state_exchange_publish_control_output(const control_output_t *out);
uint32_t state_exchange_get_control_output(control_output_t *out);

/* ---------- flight state (CM4 writes, CM7 reads) ---------- */
uint32_t state_exchange_publish_flight_state(app_flight_state_t s);
uint32_t state_exchange_get_flight_state(app_flight_state_t *out);

uint32_t state_exchange_publish_armed(bool armed);
uint32_t state_exchange_get_armed(bool *armed_out);

/* ---------- tuning + reference (CM4 writes, CM7 reads) ----------
 * Each publish bumps a seq number; CM7's controls task polls *_get_*
 * periodically and merges fresh values into its live config/reference
 * snapshot. The ISR reads from that snapshot, not from these slots. */
uint32_t state_exchange_publish_pid_gains(const app_pid_gains_t *gains);
uint32_t state_exchange_get_pid_gains(app_pid_gains_t *out);

uint32_t state_exchange_publish_reference(const app_reference_t *ref);
uint32_t state_exchange_get_reference(app_reference_t *out);

uint32_t state_exchange_publish_vehicle_config(const app_vehicle_config_t *cfg);
uint32_t state_exchange_get_vehicle_config(app_vehicle_config_t *out);

#ifdef __cplusplus
}
#endif

#endif /* APP_STATE_EXCHANGE_H */
