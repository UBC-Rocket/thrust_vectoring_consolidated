/**
 * @file    shared_memory.c
 * @brief   Defines the dual-core shared-memory region.
 *
 * Drivers and the intercore seqlock slots address into this region by
 * offset (see APP_SLOT_*_OFFSET in app/shared_memory.h). The array is
 * tagged with section(".shared"), which both CM4 and CM7 linker scripts
 * route to SRAM4 (0x38000000, D3 domain, uncached on both cores) so
 * the two cores see the same backing bytes at the same physical address.
 *
 * UBC Rocket, 2026
 */
#include "app/shared_memory.h"

#include <stdint.h>

uint8_t __shared_region_start__[APP_SHARED_REGION_SIZE]
    __attribute__((section(".shared"), aligned(8)));
