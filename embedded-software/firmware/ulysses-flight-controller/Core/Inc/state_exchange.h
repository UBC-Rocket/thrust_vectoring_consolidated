/**
 * @file    state_exchange.h
 * @brief   Publish/get for state, flight_state, and control_output (sequence numbers).
 */
#ifndef STATE_EXCHANGE_H
#define STATE_EXCHANGE_H

#include <stdbool.h>
#include <stdint.h>
#include "state_estimation/state.h"
#include "mission_manager/mission_manager.h"
#include "controls/flight_controller.h"

/**
 * @brief Initialise synchronization primitives for state sharing.
 *        Safe to call multiple times.
 */
void state_exchange_init(void);

/**
 * @brief Publish the latest fused state.
 * @param state Pointer to state structure.
 * @return Monotonic sequence number after publish.
 */
uint32_t state_exchange_publish_state(const state_t *state);

/**
 * @brief Copy the most recently published state.
 * @param state_out Destination pointer (optional).
 * @return Sequence number associated with the returned state.
 */
uint32_t state_exchange_get_state(state_t *state_out);

/**
 * @brief Publish current flight state.
 * @param flight_state New flight state.
 * @return Monotonic sequence number after publish.
 */
uint32_t state_exchange_publish_flight_state(flight_state_t flight_state);

/**
 * @brief Copy the latest flight state.
 * @param flight_state_out Destination pointer (optional).
 * @return Sequence number associated with the returned flight state.
 */
uint32_t state_exchange_get_flight_state(flight_state_t *flight_state_out);

/**
 * @brief Publish control output (tau_gim, tau_thrust, T_cmd, theta_x/y).
 * @param out Pointer to control output from flight controller.
 * @return Monotonic sequence number after publish.
 */
uint32_t state_exchange_publish_control_output(const control_output_t *out);

/**
 * @brief Copy the latest control output.
 * @param out Destination pointer (optional).
 * @return Sequence number associated with the returned output.
 */
uint32_t state_exchange_get_control_output(control_output_t *out);

/**
 * @brief Publish armed state.  Set by mission manager on CMD_ARM.
 * @param armed true = controls enabled, false = outputs zeroed.
 * @return Monotonic sequence number after publish.
 */
uint32_t state_exchange_publish_armed(bool armed);

/**
 * @brief Copy the latest armed state.
 * @param armed_out Destination pointer (optional).
 * @return Sequence number associated with the returned value.
 */
uint32_t state_exchange_get_armed(bool *armed_out);

/**
 * @brief Publish startup actuator test completion.
 * @param complete true once the startup test sequence finishes.
 * @return Monotonic sequence number after publish.
 */
uint32_t state_exchange_publish_startup_test_complete(bool complete);

/**
 * @brief Copy the startup test completion flag.
 * @param complete_out Destination pointer (optional).
 * @return Sequence number associated with the returned value.
 */
uint32_t state_exchange_get_startup_test_complete(bool *complete_out);

/**
 * @brief Publish a rearm request.  Set by mission manager on CMD_ARM; cleared
 *        by the controls task once the startup sequence begins.
 * @param requested true = run startup sequence and re-arm.
 * @return Monotonic sequence number after publish.
 */
uint32_t state_exchange_publish_rearm_request(bool requested);

/**
 * @brief Copy the rearm request flag.
 * @param requested_out Destination pointer (optional).
 * @return Sequence number associated with the returned value.
 */
uint32_t state_exchange_get_rearm_request(bool *requested_out);

#endif /* STATE_EXCHANGE_H */
