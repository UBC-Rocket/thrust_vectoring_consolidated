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

#ifdef __cplusplus
}
#endif

#endif /* APP_STATE_EXCHANGE_H */
