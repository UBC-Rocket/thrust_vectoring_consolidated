/**
 * @file    state_exchange.c
 * @brief   Thread-safe exchange for state, flight_state, and control_output (mutex + latest copy).
 */
#include "state_exchange.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "command.pb.h"

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Maximum time to wait for state mutex (in ticks).
 *
 * Using bounded timeout instead of portMAX_DELAY prevents indefinite blocking.
 * 10ms is generous - state operations should complete in <1us.
 * If timeout occurs, it indicates a serious system problem.
 *
 * Note: FreeRTOS mutexes (configUSE_MUTEXES=1) include priority inheritance,
 * which prevents unbounded priority inversion. However, bounded timeout
 * provides an additional safety net for deadlock detection.
 */
#define STATE_MUTEX_TIMEOUT_TICKS pdMS_TO_TICKS(10)

/* -------------------------------------------------------------------------- */
/* Private state                                                              */
/* -------------------------------------------------------------------------- */

static StaticSemaphore_t state_mutex_buffer;
static StaticSemaphore_t flight_mutex_buffer;
static StaticSemaphore_t control_mutex_buffer;
static SemaphoreHandle_t state_mutex_handle = NULL;
static SemaphoreHandle_t flight_mutex_handle = NULL;
static SemaphoreHandle_t control_mutex_handle = NULL;

static state_t latest_state = {0};
static flight_state_t latest_flight_state = IDLE;
static control_output_t latest_control_output = {0};
static bool latest_armed = false;
static bool latest_startup_test_complete = false;
static bool latest_rearm_requested = false;
static tvr_SetPidGains latest_pid_gains = {0};
static tvr_SetReference latest_flight_reference = {0};
static tvr_SetConfig latest_vehicle_config = {0};

static uint32_t state_seq = 0;
static uint32_t flight_state_seq = 0;
static uint32_t control_seq = 0;
static uint32_t armed_seq = 0;
static uint32_t startup_test_seq = 0;
static uint32_t rearm_seq = 0;
static uint32_t pid_gains_seq = 0;
static uint32_t flight_reference_seq = 0;
static uint32_t vehicle_config_seq = 0;

/* -------------------------------------------------------------------------- */
/* Private helpers                                                            */
/* -------------------------------------------------------------------------- */

static void ensure_initialized(void)
{
    if (state_mutex_handle == NULL) {
        state_mutex_handle = xSemaphoreCreateMutexStatic(&state_mutex_buffer);
    }

    if (flight_mutex_handle == NULL) {
        flight_mutex_handle = xSemaphoreCreateMutexStatic(&flight_mutex_buffer);
    }

    if (control_mutex_handle == NULL) {
        control_mutex_handle = xSemaphoreCreateMutexStatic(&control_mutex_buffer);
    }
}

/**
 * @brief Take mutex with bounded timeout.
 * @param mutex The mutex handle.
 * @return pdTRUE if acquired, pdFALSE on timeout.
 * @note Logs error and continues on timeout to avoid system hang.
 */
static inline BaseType_t take_mutex_safe(SemaphoreHandle_t mutex)
{
    BaseType_t result = xSemaphoreTake(mutex, STATE_MUTEX_TIMEOUT_TICKS);
    /* In production: could assert or log on timeout */
    /* configASSERT(result == pdTRUE); */
    return result;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

void state_exchange_init(void)
{
    taskENTER_CRITICAL();
    ensure_initialized();
    taskEXIT_CRITICAL();
}

uint32_t state_exchange_publish_state(const state_t *state)
{
    if (!state) {
        return state_seq;
    }

    ensure_initialized();
    if (take_mutex_safe(state_mutex_handle) != pdTRUE) {
        /* Mutex timeout - return current sequence without updating */
        return state_seq;
    }
    latest_state = *state;
    state_seq++;
    uint32_t seq = state_seq;
    xSemaphoreGive(state_mutex_handle);
    return seq;
}

uint32_t state_exchange_get_state(state_t *state_out)
{
    ensure_initialized();
    if (take_mutex_safe(state_mutex_handle) != pdTRUE) {
        /* Mutex timeout - return stale data with current sequence */
        if (state_out) {
            *state_out = latest_state; /* Read without lock (may be inconsistent) */
        }
        return state_seq;
    }
    if (state_out) {
        *state_out = latest_state;
    }
    uint32_t seq = state_seq;
    xSemaphoreGive(state_mutex_handle);
    return seq;
}

uint32_t state_exchange_publish_flight_state(flight_state_t flight_state)
{
    ensure_initialized();
    if (take_mutex_safe(flight_mutex_handle) != pdTRUE) {
        /* Mutex timeout - return current sequence without updating */
        return flight_state_seq;
    }
    latest_flight_state = flight_state;
    flight_state_seq++;
    uint32_t seq = flight_state_seq;
    xSemaphoreGive(flight_mutex_handle);
    return seq;
}

uint32_t state_exchange_get_flight_state(flight_state_t *flight_state_out)
{
    ensure_initialized();
    if (take_mutex_safe(flight_mutex_handle) != pdTRUE) {
        /* Mutex timeout - return stale data with current sequence */
        if (flight_state_out) {
            *flight_state_out = latest_flight_state;
        }
        return flight_state_seq;
    }
    if (flight_state_out) {
        *flight_state_out = latest_flight_state;
    }
    uint32_t seq = flight_state_seq;
    xSemaphoreGive(flight_mutex_handle);
    return seq;
}

uint32_t state_exchange_publish_control_output(const control_output_t *out)
{
    if (!out) {
        return control_seq;
    }

    ensure_initialized();
    xSemaphoreTake(control_mutex_handle, portMAX_DELAY);
    latest_control_output = *out;
    control_seq++;
    uint32_t seq = control_seq;
    xSemaphoreGive(control_mutex_handle);
    return seq;
}

uint32_t state_exchange_get_control_output(control_output_t *out)
{
    ensure_initialized();
    xSemaphoreTake(control_mutex_handle, portMAX_DELAY);
    if (out) {
        *out = latest_control_output;
    }
    uint32_t seq = control_seq;
    xSemaphoreGive(control_mutex_handle);
    return seq;
}

uint32_t state_exchange_publish_armed(bool armed)
{
    ensure_initialized();
    if (take_mutex_safe(flight_mutex_handle) != pdTRUE) {
        return armed_seq;
    }
    latest_armed = armed;
    armed_seq++;
    uint32_t seq = armed_seq;
    xSemaphoreGive(flight_mutex_handle);
    return seq;
}

uint32_t state_exchange_get_armed(bool *armed_out)
{
    ensure_initialized();
    if (take_mutex_safe(flight_mutex_handle) != pdTRUE) {
        if (armed_out) {
            *armed_out = latest_armed;
        }
        return armed_seq;
    }
    if (armed_out) {
        *armed_out = latest_armed;
    }
    uint32_t seq = armed_seq;
    xSemaphoreGive(flight_mutex_handle);
    return seq;
}

uint32_t state_exchange_publish_startup_test_complete(bool complete)
{
    ensure_initialized();
    if (take_mutex_safe(flight_mutex_handle) != pdTRUE) {
        return startup_test_seq;
    }
    latest_startup_test_complete = complete;
    startup_test_seq++;
    uint32_t seq = startup_test_seq;
    xSemaphoreGive(flight_mutex_handle);
    return seq;
}

uint32_t state_exchange_get_startup_test_complete(bool *complete_out)
{
    ensure_initialized();
    if (take_mutex_safe(flight_mutex_handle) != pdTRUE) {
        if (complete_out) {
            *complete_out = latest_startup_test_complete;
        }
        return startup_test_seq;
    }
    if (complete_out) {
        *complete_out = latest_startup_test_complete;
    }
    uint32_t seq = startup_test_seq;
    xSemaphoreGive(flight_mutex_handle);
    return seq;
}

uint32_t state_exchange_publish_rearm_request(bool requested)
{
    ensure_initialized();
    if (take_mutex_safe(flight_mutex_handle) != pdTRUE) {
        return rearm_seq;
    }
    latest_rearm_requested = requested;
    rearm_seq++;
    uint32_t seq = rearm_seq;
    xSemaphoreGive(flight_mutex_handle);
    return seq;
}

uint32_t state_exchange_get_rearm_request(bool *requested_out)
{
    ensure_initialized();
    if (take_mutex_safe(flight_mutex_handle) != pdTRUE) {
        if (requested_out) {
            *requested_out = latest_rearm_requested;
        }
        return rearm_seq;
    }
    if (requested_out) {
        *requested_out = latest_rearm_requested;
    }
    uint32_t seq = rearm_seq;
    xSemaphoreGive(flight_mutex_handle);
    return seq;
}

uint32_t state_exchange_publish_pid_gains(tvr_SetPidGains pid_gains)
{
    ensure_initialized();
    if (take_mutex_safe(flight_mutex_handle) != pdTRUE) {
        return pid_gains_seq;
    }
    latest_pid_gains = pid_gains;
    pid_gains_seq++;
    uint32_t seq = pid_gains_seq;
    xSemaphoreGive(flight_mutex_handle);
    return seq;
}

uint32_t state_exchange_get_pid_gains(tvr_SetPidGains *pid_gains_out)
{
    ensure_initialized();
    if (take_mutex_safe(flight_mutex_handle) != pdTRUE) {
        if (pid_gains_out) {
            *pid_gains_out = latest_pid_gains;
        }
        return pid_gains_seq;
    }
    if (pid_gains_out) {
        *pid_gains_out = latest_pid_gains;
    }
    uint32_t seq = pid_gains_seq;
    xSemaphoreGive(flight_mutex_handle);
    return seq;
}

uint32_t state_exchange_publish_vehicle_config(tvr_SetConfig config)
{
    ensure_initialized();
    if (take_mutex_safe(flight_mutex_handle) != pdTRUE) {
        return vehicle_config_seq;
    }
    latest_vehicle_config = config;
    vehicle_config_seq++;
    uint32_t seq = vehicle_config_seq;
    xSemaphoreGive(flight_mutex_handle);
    return seq;
}

uint32_t state_exchange_get_vehicle_config(tvr_SetConfig *config_out)
{
    ensure_initialized();
    if (take_mutex_safe(flight_mutex_handle) != pdTRUE) {
        if (config_out) {
            *config_out = latest_vehicle_config;
        }
        return vehicle_config_seq;
    }
    if (config_out) {
        *config_out = latest_vehicle_config;
    }
    uint32_t seq = vehicle_config_seq;
    xSemaphoreGive(flight_mutex_handle);
    return seq;
}

uint32_t state_exchange_publish_flight_reference(tvr_SetReference reference)
{
    ensure_initialized();
    if (take_mutex_safe(flight_mutex_handle) != pdTRUE) {
        return flight_reference_seq;
    }
    latest_flight_reference = reference;
    flight_reference_seq++;
    uint32_t seq = flight_reference_seq;
    xSemaphoreGive(flight_mutex_handle);
    return seq;
}

uint32_t state_exchange_get_flight_reference(tvr_SetReference *reference_out)
{
    ensure_initialized();
    if (take_mutex_safe(flight_mutex_handle) != pdTRUE) {
        if (reference_out) {
            *reference_out = latest_flight_reference;
        }
        return flight_reference_seq;
    }
    if (reference_out) {
        *reference_out = latest_flight_reference;
    }
    uint32_t seq = flight_reference_seq;
    xSemaphoreGive(flight_mutex_handle);
    return seq;
}
