/**
 * @file sync.h
 * @brief Portable synchronization primitives for single-core and dual-core MCUs.
 *
 * This header abstracts synchronization mechanisms to enable future migration
 * to dual-core processors. On single-core, these map to standard CMSIS/FreeRTOS
 * primitives. On dual-core, implementations would use hardware semaphores (HSEM).
 *
 * @note For dual-core STM32H7/H5 variants, replace implementations with HSEM-based
 *       spinlocks and add inter-processor memory barriers.
 *
 * UBC Rocket, Jan 2026
 */

#ifndef SYNC_H
#define SYNC_H

#include "stm32h5xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Define ULYSSES_DUAL_CORE to enable dual-core synchronization.
 *        When undefined, single-core optimized primitives are used.
 */
/* #define ULYSSES_DUAL_CORE */

/* -------------------------------------------------------------------------- */
/* Memory Barriers                                                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Data Memory Barrier - ensures all explicit memory accesses before
 *        this instruction complete before any explicit memory accesses after.
 */
#define SYNC_DMB() __DMB()

/**
 * @brief Data Synchronization Barrier - ensures all explicit memory accesses
 *        and cache/TLB maintenance complete before continuing.
 */
#define SYNC_DSB() __DSB()

/**
 * @brief Instruction Synchronization Barrier - flushes pipeline and ensures
 *        subsequent instructions are fetched from cache/memory.
 */
#define SYNC_ISB() __ISB()

/**
 * @brief Compiler barrier - prevents compiler from reordering across this point.
 *        Does NOT prevent CPU reordering (use SYNC_DMB for that).
 */
#define SYNC_COMPILER_BARRIER() __asm volatile("" ::: "memory")

/* -------------------------------------------------------------------------- */
/* Critical Sections (Interrupt-based)                                        */
/* -------------------------------------------------------------------------- */

#if !defined(ULYSSES_DUAL_CORE)

/**
 * @brief Enter critical section from task context.
 *        Disables interrupts on single-core. On dual-core, would also acquire
 *        hardware spinlock.
 */
#define SYNC_ENTER_CRITICAL() \
    do { \
        taskENTER_CRITICAL(); \
    } while (0)

/**
 * @brief Exit critical section from task context.
 */
#define SYNC_EXIT_CRITICAL() \
    do { \
        taskEXIT_CRITICAL(); \
    } while (0)

/**
 * @brief Enter critical section from ISR context.
 * @return Status value to pass to SYNC_EXIT_CRITICAL_FROM_ISR.
 */
#define SYNC_ENTER_CRITICAL_FROM_ISR() taskENTER_CRITICAL_FROM_ISR()

/**
 * @brief Exit critical section from ISR context.
 * @param status Value returned by SYNC_ENTER_CRITICAL_FROM_ISR.
 */
#define SYNC_EXIT_CRITICAL_FROM_ISR(status) taskEXIT_CRITICAL_FROM_ISR(status)

/**
 * @brief Enter critical section using raw PRIMASK (no FreeRTOS).
 *        Use when FreeRTOS scheduler may not be running.
 * @param primask_var Variable to store PRIMASK state.
 */
#define SYNC_ENTER_CRITICAL_RAW(primask_var) \
    do { \
        (primask_var) = __get_PRIMASK(); \
        __disable_irq(); \
    } while (0)

/**
 * @brief Exit critical section using raw PRIMASK.
 * @param primask_var Variable containing saved PRIMASK state.
 */
#define SYNC_EXIT_CRITICAL_RAW(primask_var) \
    do { \
        __set_PRIMASK(primask_var); \
    } while (0)

#else /* ULYSSES_DUAL_CORE */

/*
 * Dual-core implementation placeholder.
 * Would use STM32 Hardware Semaphore (HSEM) peripheral:
 *
 * #define SYNC_ENTER_CRITICAL() \
 *     do { \
 *         taskENTER_CRITICAL(); \
 *         while (HAL_HSEM_FastTake(HSEM_ID_SPINLOCK) != HAL_OK) {} \
 *     } while (0)
 *
 * #define SYNC_EXIT_CRITICAL() \
 *     do { \
 *         HAL_HSEM_Release(HSEM_ID_SPINLOCK, 0); \
 *         taskEXIT_CRITICAL(); \
 *     } while (0)
 */
#error "Dual-core synchronization not yet implemented. Define HSEM-based primitives."

#endif /* ULYSSES_DUAL_CORE */

/* -------------------------------------------------------------------------- */
/* Atomic Operations                                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief Atomic load of a uint8_t with acquire semantics.
 *        Ensures subsequent reads see data written before the store that
 *        published this value.
 */
static inline uint8_t sync_load_acquire_u8(volatile uint8_t *ptr)
{
    uint8_t val = *ptr;
    SYNC_DMB();
    return val;
}

/**
 * @brief Atomic store of a uint8_t with release semantics.
 *        Ensures all prior writes are visible before this store is visible.
 */
static inline void sync_store_release_u8(volatile uint8_t *ptr, uint8_t val)
{
    SYNC_DMB();
    *ptr = val;
}

/**
 * @brief Atomic load of a uint32_t with acquire semantics.
 */
static inline uint32_t sync_load_acquire_u32(volatile uint32_t *ptr)
{
    uint32_t val = *ptr;
    SYNC_DMB();
    return val;
}

/**
 * @brief Atomic store of a uint32_t with release semantics.
 */
static inline void sync_store_release_u32(volatile uint32_t *ptr, uint32_t val)
{
    SYNC_DMB();
    *ptr = val;
}

/**
 * @brief Atomic load of a bool with acquire semantics.
 */
static inline bool sync_load_acquire_bool(volatile bool *ptr)
{
    bool val = *ptr;
    SYNC_DMB();
    return val;
}

/**
 * @brief Atomic store of a bool with release semantics.
 */
static inline void sync_store_release_bool(volatile bool *ptr, bool val)
{
    SYNC_DMB();
    *ptr = val;
}

/* -------------------------------------------------------------------------- */
/* Cache Maintenance                                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief Check if D-Cache is enabled.
 */
static inline bool sync_dcache_enabled(void)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    return (SCB->CCR & SCB_CCR_DC_Msk) != 0U;
#else
    return false;
#endif
}

/**
 * @brief Invalidate D-Cache for a memory region (after DMA RX).
 *        Call before CPU reads data written by DMA.
 * @param addr Start address (should be 32-byte aligned).
 * @param size Size in bytes (should be multiple of 32).
 */
static inline void sync_dcache_invalidate(void *addr, uint32_t size)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if (sync_dcache_enabled()) {
        SCB_InvalidateDCache_by_Addr(addr, (int32_t)size);
    }
#else
    (void)addr;
    (void)size;
#endif
}

/**
 * @brief Clean D-Cache for a memory region (before DMA TX).
 *        Call after CPU writes data, before DMA reads it.
 * @param addr Start address (should be 32-byte aligned).
 * @param size Size in bytes (should be multiple of 32).
 */
static inline void sync_dcache_clean(void *addr, uint32_t size)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if (sync_dcache_enabled()) {
        SCB_CleanDCache_by_Addr(addr, (int32_t)size);
    }
#else
    (void)addr;
    (void)size;
#endif
}

/**
 * @brief Clean and invalidate D-Cache for a memory region.
 *        Use for bidirectional DMA buffers.
 */
static inline void sync_dcache_clean_invalidate(void *addr, uint32_t size)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if (sync_dcache_enabled()) {
        SCB_CleanInvalidateDCache_by_Addr(addr, (int32_t)size);
    }
#else
    (void)addr;
    (void)size;
#endif
}

/* -------------------------------------------------------------------------- */
/* Alignment Macros                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Cache line size for STM32H5 (32 bytes).
 */
#define SYNC_CACHE_LINE_SIZE 32U

/**
 * @brief Align a variable to cache line boundary for DMA.
 * @example __ALIGNED(SYNC_CACHE_LINE_SIZE) uint8_t dma_buffer[64];
 */
#ifndef SYNC_CACHE_ALIGNED
#define SYNC_CACHE_ALIGNED __ALIGNED(SYNC_CACHE_LINE_SIZE)
#endif

/**
 * @brief Round size up to cache line multiple.
 */
#define SYNC_CACHE_ALIGN_SIZE(size) \
    (((size) + SYNC_CACHE_LINE_SIZE - 1U) & ~(SYNC_CACHE_LINE_SIZE - 1U))

#ifdef __cplusplus
}
#endif

#endif /* SYNC_H */

