#ifndef TRACE_HOOKS_H
#define TRACE_HOOKS_H

#ifdef DEBUG

/**
 * @brief Trace hook called by FreeRTOS when a task is switched in.
 *
 * Must be safe to call from the scheduler critical section.
 */
void trace_hook_task_switched_in(void);

/**
 * @brief Trace hook called by FreeRTOS when a task is switched out.
 */
void trace_hook_task_switched_out(void);

/**
 * @brief Trace hook called by FreeRTOS when a task is created.
 *
 * @param pxNewTCB  Pointer to the new task's TCB (passed by FreeRTOS macro).
 */
void trace_hook_task_create(void *pxNewTCB);

#endif /* DEBUG */
#endif /* TRACE_HOOKS_H */
