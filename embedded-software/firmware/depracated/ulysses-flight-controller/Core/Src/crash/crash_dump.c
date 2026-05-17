#include "crash/crash_dump.h"
#include "timestamp.h"
#include "main.h"
#include <string.h>
#include <stdbool.h>

extern SD_HandleTypeDef hsd1;

/* SRAM address range for pointer validation */
#define SRAM_START  0x20000000UL
#define SRAM_END    (0x20000000UL + 640U * 1024U)

/* FreeRTOS internal: pxCurrentTCB has external linkage (used by port layer) */
extern void * volatile pxCurrentTCB;

/* ---- Task registration ---- */

static TaskHandle_t s_registered_tasks[CRASH_MAX_TASKS];
static volatile uint8_t s_registered_count = 0;

void crash_dump_register_task(TaskHandle_t handle)
{
    if (s_registered_count < CRASH_MAX_TASKS) {
        s_registered_tasks[s_registered_count++] = handle;
    }
}

/* ---- .crash_noinit flag ---- */

__attribute__((section(".crash_noinit")))
static volatile uint32_t s_crash_written_flag;

bool crash_dump_check_previous(void)
{
    if (s_crash_written_flag == CRASH_DUMP_MAGIC) {
        s_crash_written_flag = 0;
        return true;
    }
    return false;
}

/* ---- Helpers ---- */

static bool is_valid_sram_ptr(const void *p)
{
    uint32_t addr = (uint32_t)(uintptr_t)p;
    return (addr >= SRAM_START) && (addr < SRAM_END);
}

static void copy_task_name(char dst[16], const char *src)
{
    if (!is_valid_sram_ptr(src)) {
        memset(dst, 0, 16);
        return;
    }
    for (int i = 0; i < 15; i++) {
        dst[i] = src[i];
        if (src[i] == '\0') {
            memset(&dst[i + 1], 0, 15 - i);
            return;
        }
    }
    dst[15] = '\0';
}

static void crash_sd_abort_and_reinit(void)
{
    HAL_SD_Abort(&hsd1);

    volatile uint32_t timeout = 0;
    while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
        if (++timeout > 2000000U) {
            break;
        }
    }
}

/* ---- Main crash dump handler ---- */

/* Static buffer — must NOT be on stack (stack may be corrupted) */
static uint8_t s_crash_buffer[CRASH_DUMP_SD_BLOCK_COUNT * 512U]
    __attribute__((aligned(4)));

void crash_dump_handler(uint32_t *stack_frame, uint32_t exc_return,
                        uint32_t fault_type)
{
    memset(s_crash_buffer, 0xFF, sizeof(s_crash_buffer));

    crash_dump_header_t *hdr = (crash_dump_header_t *)s_crash_buffer;

    /* Header */
    hdr->magic           = CRASH_DUMP_MAGIC;
    hdr->version         = CRASH_DUMP_VERSION;
    hdr->timestamp_us    = timestamp_us();
    hdr->fault_type      = (uint8_t)fault_type;

    /* Fault registers */
    hdr->cfsr       = SCB->CFSR;
    hdr->hfsr       = SCB->HFSR;
    hdr->bfar       = SCB->BFAR;
    hdr->mmfar      = SCB->MMFAR;
    hdr->shcsr      = SCB->SHCSR;
    hdr->exc_return = exc_return;
    hdr->fault_msp  = __get_MSP();
    hdr->fault_psp  = __get_PSP();

    /* Exception stack frame */
    if (is_valid_sram_ptr(stack_frame)) {
        hdr->r0            = stack_frame[0];
        hdr->r1            = stack_frame[1];
        hdr->r2            = stack_frame[2];
        hdr->r3            = stack_frame[3];
        hdr->r12           = stack_frame[4];
        hdr->lr_from_frame = stack_frame[5];
        hdr->pc            = stack_frame[6];
        hdr->xpsr          = stack_frame[7];
    }

    /* Active task info */
    TaskHandle_t current = (TaskHandle_t)pxCurrentTCB;
    if (is_valid_sram_ptr(current)) {
        copy_task_name(hdr->active_task_name, pcTaskGetName(current));
        hdr->active_task_number = (uint32_t)uxTaskGetTaskNumber(current);
    }

    /* Per-task entries */
    crash_dump_task_entry_t *entries =
        (crash_dump_task_entry_t *)(s_crash_buffer + sizeof(crash_dump_header_t));

    uint8_t count = 0;
    for (uint8_t i = 0; i < s_registered_count && count < CRASH_MAX_TASKS; i++) {
        TaskHandle_t h = s_registered_tasks[i];
        if (!is_valid_sram_ptr(h)) {
            continue;
        }

        crash_dump_task_entry_t *e = &entries[count];
        memset(e, 0, sizeof(*e));

        copy_task_name(e->name, pcTaskGetName(h));
        e->task_number   = (uint32_t)uxTaskGetTaskNumber(h);
        e->priority      = (uint32_t)uxTaskPriorityGet(h);
        e->state         = (uint8_t)eTaskGetState(h);

        /* Read pxTopOfStack — first word of TCB */
        uint32_t *tcb_ptr = (uint32_t *)(uintptr_t)h;
        if (is_valid_sram_ptr(tcb_ptr)) {
            e->stack_pointer = tcb_ptr[0]; /* pxTopOfStack */
        }

        /* Dump top N words of the task's stack */
        uint32_t *sp = (uint32_t *)(uintptr_t)e->stack_pointer;
        for (uint32_t w = 0; w < CRASH_STACK_DUMP_WORDS; w++) {
            if (is_valid_sram_ptr(&sp[w])) {
                e->stack_words[w] = sp[w];
            } else {
                break;
            }
        }

        count++;
    }

    hdr->task_count = count;
    hdr->total_size = (uint32_t)(sizeof(crash_dump_header_t)
                      + count * sizeof(crash_dump_task_entry_t));

    /* Write to SD card */
    crash_sd_abort_and_reinit();

    uint32_t blocks_needed = (hdr->total_size + 511U) / 512U;
    if (blocks_needed > CRASH_DUMP_SD_BLOCK_COUNT) {
        blocks_needed = CRASH_DUMP_SD_BLOCK_COUNT;
    }

    HAL_SD_WriteBlocks(&hsd1, s_crash_buffer,
                       CRASH_DUMP_SD_BLOCK_START, blocks_needed, 5000U);

    /* Mark that a crash dump was written */
    s_crash_written_flag = CRASH_DUMP_MAGIC;

    __BKPT(0);
    while (1) { /* Never return */ }
}
