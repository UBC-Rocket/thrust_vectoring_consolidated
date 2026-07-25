/**
 * @file    app_init.h
 * @brief   Per-core init phases + scheduler hand-off.
 *
 * Boot order on each core:
 *   1. Reset / clocks / SystemInit (Cube boot files)
 *   2. main() → io_init_cm{4,7}()    bring up HW transports
 *               dev_init_cm{4,7}()   instantiate device drivers
 *               app_init_cm{4,7}()   create FreeRTOS tasks, wire notifies
 *               app_start_kernel()   hand off to the scheduler (never returns)
 *
 * Splitting these phases keeps the dependency order one-way (HW → IO → DEV
 * → APP) and lets a future SIL build supply its own io_init / app_start_kernel
 * without touching the dev / app layers.
 *
 * UBC Rocket, 2026
 */
#ifndef APP_INIT_H
#define APP_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------- DEV-layer init ------------------------- */
/** @brief Instantiate every CM4 device driver. Call after io_init_cm4(). */
void dev_init_cm4(void);

/** @brief Instantiate every CM7 actuator driver. Call after io_init_cm7(). */
void dev_init_cm7(void);

/* ------------------------- APP-layer init ------------------------- */
/** @brief Create CM4 FreeRTOS tasks; wire notify targets. Call after dev_init_cm4(). */
void app_init_cm4(void);

/** @brief Create CM7 FreeRTOS tasks. Call after dev_init_cm7(). */
void app_init_cm7(void);

/* ------------------------- Scheduler hand-off ------------------------- */
/**
 * @brief Hand control to the task scheduler. Never returns on H747
 *        (calls @c vTaskStartScheduler). A SIL build would replace this with
 *        a pthread join over the simulated thread pool.
 */
void app_start_kernel(void) __attribute__((noreturn));

#ifdef __cplusplus
}
#endif

#endif /* APP_INIT_H */
