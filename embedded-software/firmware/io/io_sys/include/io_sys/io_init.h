/**
 * @file io_init.h
 * @brief Per-core IO-layer initialisation entry points.
 *
 * Each core's main() calls io_init_cm{4,7}() to bring up clocks, DMAs,
 * peripherals, the timestamp source and the transport queues. The driver
 * layer (dev_init_cm{4,7}, declared in app_init.h) follows. Scheduler
 * hand-off lives at the APP layer (app_start_kernel, also in app_init.h).
 *
 * UBC Rocket, 2026
 */

#ifndef IO_INIT_H
#define IO_INIT_H

#include "io_sys/io_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void io_init_cm4(void);
void io_init_cm7(void);

#ifdef __cplusplus
}
#endif

#endif /* IO_INIT_H */
