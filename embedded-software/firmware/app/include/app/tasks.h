/**
 * @file    tasks.h
 * @brief   Entry points for per-core FreeRTOS tasks.
 *
 * Each task is declared here and defined in @c app/cm{4,7}/ — the per-core
 * CMakeLists link the appropriate set. The functions have the signature
 * expected by osThreadNew / xTaskCreate.
 *
 * UBC Rocket, 2026
 */
#ifndef APP_TASKS_H
#define APP_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---- CM4 tasks ---- */
void task_state_estimation(void *arg);
void task_mission_manager(void *arg);
void task_sd_log(void *arg);

/* ---- CM7 tasks ---- */
void task_controls(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASKS_H */
