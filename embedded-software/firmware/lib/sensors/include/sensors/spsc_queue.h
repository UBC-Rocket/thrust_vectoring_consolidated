/**
 * @file spsc_queue.h
 * @brief Portable memory barrier for SPSC ring buffers in sensor drivers.
 *
 * Provides SENSORS_MEMORY_BARRIER() which defaults to a compiler barrier
 * (sufficient for single-core host testing). On ARM targets, define
 * SENSORS_USE_ARM_BARRIER before including this header (or via compile
 * definition) to use the hardware __DMB() instruction.
 *
 * @ UBC Rocket, 2026
 */

#ifndef SENSORS_SPSC_QUEUE_H
#define SENSORS_SPSC_QUEUE_H

#ifndef SENSORS_MEMORY_BARRIER
  #if defined(SENSORS_USE_ARM_BARRIER)
    /* ARM Cortex-M: use hardware Data Memory Barrier.
       __DMB() is provided by CMSIS (cmsis_gcc.h / cmsis_compiler.h),
       which is included transitively by any consumer that uses HAL. */
    #define SENSORS_MEMORY_BARRIER() __DMB()
  #elif defined(__GNUC__) || defined(__clang__)
    #define SENSORS_MEMORY_BARRIER() __asm volatile("" ::: "memory")
  #else
    #define SENSORS_MEMORY_BARRIER() ((void)0)
  #endif
#endif

#endif /* SENSORS_SPSC_QUEUE_H */
