#ifndef CRASH_DUMP_H
#define CRASH_DUMP_H

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

#define CRASH_DUMP_SD_BLOCK_START   (131072U - 16U) /* Last 16 blocks of initial erase */
#define CRASH_DUMP_SD_BLOCK_COUNT   16U             /* 8 KB total */
#define CRASH_DUMP_MAGIC            0xDEADFA17U
#define CRASH_DUMP_VERSION          1U
#define CRASH_MAX_TASKS             8U
#define CRASH_STACK_DUMP_WORDS      32U             /* 128 bytes per task stack */

#define CRASH_FAULT_HARD            0U
#define CRASH_FAULT_MEMMANAGE       1U
#define CRASH_FAULT_BUS             2U
#define CRASH_FAULT_USAGE           3U

typedef struct __attribute__((packed)) {
    /* Header */
    uint32_t magic;
    uint32_t version;
    uint32_t timestamp_us;
    uint32_t total_size;

    /* Fault registers */
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t bfar;
    uint32_t mmfar;
    uint32_t shcsr;
    uint32_t exc_return;
    uint32_t fault_msp;
    uint32_t fault_psp;

    /* Exception stack frame */
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr_from_frame;
    uint32_t pc;
    uint32_t xpsr;

    /* Active task at time of fault */
    char     active_task_name[16];
    uint32_t active_task_number;

    /* Task list */
    uint8_t  task_count;
    uint8_t  fault_type;
    uint8_t  reserved[2];
} crash_dump_header_t;

typedef struct __attribute__((packed)) {
    char     name[16];
    uint32_t priority;
    uint32_t stack_pointer;
    uint32_t stack_base;
    uint32_t task_number;
    uint8_t  state;
    uint8_t  reserved[3];
    uint32_t stack_words[CRASH_STACK_DUMP_WORDS];
} crash_dump_task_entry_t;

/**
 * @brief Register a task handle for inclusion in crash dumps.
 *
 * Call after each task creation. Safe to call before the scheduler starts.
 */
void crash_dump_register_task(TaskHandle_t handle);

/**
 * @brief Main crash dump handler — called from fault handler assembly stubs.
 *
 * Collects system state and writes a crash dump to the SD card using
 * blocking (polling) I/O. Does NOT return.
 *
 * @param stack_frame  Pointer to the exception stack frame (R0-R3, R12, LR, PC, xPSR).
 * @param exc_return   The EXC_RETURN value from LR at fault entry.
 * @param fault_type   One of CRASH_FAULT_HARD / _MEMMANAGE / _BUS / _USAGE.
 */
void crash_dump_handler(uint32_t *stack_frame, uint32_t exc_return, uint32_t fault_type);

/**
 * @brief Check if a crash dump was written before the last reset.
 *
 * @return true if the .crash_noinit flag indicates a previous crash.
 */
bool crash_dump_check_previous(void);

#endif /* CRASH_DUMP_H */
