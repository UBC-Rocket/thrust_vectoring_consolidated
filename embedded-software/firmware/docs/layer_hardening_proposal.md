# Proposal: Layer Hardening + Encapsulation (IO / Dev / App)

> **SUPERSEDED** by [architecture_layers.md](architecture_layers.md). The v2
> doc rejects this proposal's "app does the BSP wiring" pattern in favour of
> self-contained device drivers. Kept here as historical context only.

**Status:** superseded
**Author:** Benedikt Howard (drafted with Claude)
**Date:** 2026-05-16

---

## Motivation

The transport-BSP / driver / app split is in place but soft. Today:

- A few **cross-TU globals** still exist (`g_servos`, `g_escs` in [app_init_cm7.c](../app/cm7/app_init_cm7.c)) where module-local statics + getters would be cleaner.
- **Driver-internal data lives in public headers** (e.g. the full `imu_bmi088_t` struct fields are visible to anyone who includes [imu_bmi088.h](../libs/drivers/imu_bmi088/include/drivers/imu_bmi088.h)).
- **The app pulls BSP transport headers** in places it shouldn't (`controls_task.c` includes `bsp/bsp_timestamp.h` — fine for system services, but the same pattern would let it reach into `bsp_spi.h`).
- **The driver knows BSP logical IDs** (`imu_bmi088_init` takes `BSP_SPI_BMI088_ACC` etc.). The driver shouldn't know which slot it's plugged into.
- **CMake link visibility is permissive** — most libraries are linked `PUBLIC`, so a top-of-tree consumer transitively gets the whole world.

These are not bugs today, but they are exactly the kinds of softness that turns into a tangle once the codebase doubles in size. Hardening them now is cheap; hardening them later is not.

This document proposes a small set of concrete moves.

---

## Goals

1. No object visible outside its translation unit unless it has to be.
2. Every layer's headers expose **what** that layer offers, not **how** it works.
3. App code can call into device drivers and a small "system services" subset of the BSP, but cannot reach transport (SPI / UART / EXTI / DShot / SD).
4. Drivers can call BSP but have no knowledge of which physical peripheral they are bound to.
5. CMake enforces (3) and (4): linking the app target should not transitively expose transport symbols.

Non-goals: no source reshuffling beyond what's required, no behavioural change.

---

## Move 1 — Replace cross-TU globals with module-local statics + getters

### Current

```c
/* app/cm7/app_init_cm7.c */
servo_feetech_t g_servos;   /* visible to anyone with `extern` */
esc_dshot_t     g_escs;
```

Anything in any TU can `extern servo_feetech_t g_servos;` and bypass init contracts.

### Proposed

Move device singletons into an `actuators_init.{h,c}` mirror of [sensors_init.h](../app/include/app/sensors_init.h). The actuator structs are `static` inside the .c; the header exposes only an `actuators_t` view-struct of pointers, returned by `actuators_handles()`.

```c
/* app/cm7/actuators_init.c */
static servo_feetech_t s_servos;
static esc_dshot_t     s_escs;
static actuators_t     s_handles;

void actuators_init(void) { /* configure both, populate s_handles */ }
const actuators_t *actuators_handles(void) { return &s_handles; }
```

```c
/* app/cm7/controls_task.c */
const actuators_t *act = actuators_handles();
servo_feetech_set_pair_degrees(act->servos, deg_x, deg_y);
esc_dshot_set_throttle(act->escs, throttle);
```

Apply the same pattern to **every** non-static module-level object. Concretely:

| File | Today | After |
|---|---|---|
| `app/cm7/app_init_cm7.c` | `g_servos`, `g_escs` global | `static`, exposed via new `actuators_init.h` |
| `flight_controller/CM4/Bsp/bsp_spi.c` | `s_q[]`, `s_q_init[]`, `s_devs[]` | already static — leave alone |
| `flight_controller/CM4/Bsp/bsp_uart.c` | `s_uart[]`, `s_map[]` | already static |
| `flight_controller/CM7/Bsp/bsp_dshot_tim.c` | `s_dma_buf`, `s_busy` | already static |
| any other `extern`s | none found | n/a |

The point of doing the audit is to make "no cross-TU globals" a rule we enforce, not an accident.

---

## Move 2 — Opaque driver handles

### Current

```c
/* libs/drivers/imu_bmi088/include/drivers/imu_bmi088.h */
typedef struct {
    bmi088_accel_t accel_dev;
    bmi088_gyro_t  gyro_dev;
    /* ... full ring buffer, notify target, etc. */
} imu_bmi088_t;
```

A caller can read or write any field. Drive-by includes pull the full type and slow incremental builds.

### Proposed

Driver headers expose only a forward-declared opaque type plus the public API. The struct definition lives in the .c. The caller no longer allocates one — it asks the driver for one.

```c
/* libs/drivers/imu_bmi088/include/drivers/imu_bmi088.h */
typedef struct imu_bmi088 imu_bmi088_t;        /* opaque */

bool                imu_bmi088_init(const imu_bmi088_cfg_t *cfg);
const imu_bmi088_t *imu_bmi088_get(void);
size_t              imu_bmi088_drain(const imu_bmi088_t *d, imu_sample_t *out, size_t max);
```

The driver owns its own storage (`static imu_bmi088_t s_singleton;` in the .c). `imu_bmi088_get()` returns `&s_singleton` after init, NULL before. Single instance per driver is the only sane model on this MCU anyway — one BMI088, one ICM-40609, etc. — so the singleton restriction is honest.

Apply uniformly to:
- `imu_bmi088`, `imu_icm40609`
- `baro_ms5611`
- `gps_nmea`, `radio_rfd900`
- `servo_feetech`, `esc_dshot`

`sensors_t` and `actuators_t` view-structs hold the opaque pointers returned by each driver's `*_get()`.

**Tradeoff:** no `xTaskCreateStatic`-style "you allocate the storage" pattern. Acceptable because the storage is bounded and small (a handful of structs total).

---

## Move 3 — Drop logical BSP IDs from driver configs

### Current

```c
imu_bmi088_init(&d, &(imu_bmi088_cfg_t){
    .acc_dev   = BSP_SPI_BMI088_ACC,
    .gyro_dev  = BSP_SPI_BMI088_GYRO,
    .acc_int   = BSP_EXTI_BMI088_ACC_INT1,
    .gyro_int  = BSP_EXTI_BMI088_GYRO_INT1,
});
```

The driver header forces the caller — and indirectly the driver source — to know which logical BSP slot the device is on. That's a leak: the BSP enum is target-specific. A SIL build would have to define the same enum values to satisfy the driver's signature.

### Proposed

Wrap the (SPI device, CS, EXTI) tuple into a BSP-provided **session handle** that the driver takes opaquely:

```c
/* libs/bsp/include/bsp/bsp_spi.h */
typedef struct bsp_spi_session bsp_spi_session_t;

/* Returned by the BSP for a logical device. The driver carries the pointer;
 * the BSP knows what's behind it. */
const bsp_spi_session_t *bsp_spi_session(bsp_spi_dev_t dev);
```

The driver `init` then accepts session handles, not enum values:

```c
typedef struct {
    const bsp_spi_session_t  *acc;
    const bsp_spi_session_t  *gyro;
    const bsp_exti_session_t *acc_int;
    const bsp_exti_session_t *gyro_int;
} imu_bmi088_cfg_t;
```

The app (in `sensors_init.c`) does the binding once:

```c
imu_bmi088_init(&(imu_bmi088_cfg_t){
    .acc      = bsp_spi_session(BSP_SPI_BMI088_ACC),
    .gyro     = bsp_spi_session(BSP_SPI_BMI088_GYRO),
    .acc_int  = bsp_exti_session(BSP_EXTI_BMI088_ACC_INT1),
    .gyro_int = bsp_exti_session(BSP_EXTI_BMI088_GYRO_INT1),
});
```

The driver source uses `bsp_spi_submit_to(session, &xfer)` etc. — no enum. SIL provides its own enum + sessions; the driver is untouched.

Cost: one extra hop (BSP enum → session) per call site. The session lookup is `O(1)` table indexing.

---

## Move 4 — Tier the BSP into "system services" vs "transport"

App code legitimately wants:
- A monotonic clock (`bsp_timestamp_us`)
- Inter-core signalling (`bsp_intercore_*` — though arguably the app should use `state_exchange` and `log_service`, not raw HSEM)
- Status LEDs

App code does **not** want:
- `bsp_spi_*`, `bsp_uart_*`, `bsp_exti_*`, `bsp_dshot_*`, `bsp_sd_*` — those are the driver layer's concern.

### Proposed split

Two interface libraries instead of one:

```
libs/
├── bsp/                    # transport — internal to driver builds
│   └── include/bsp/
│       ├── bsp_spi.h
│       ├── bsp_uart.h
│       ├── bsp_exti.h
│       ├── bsp_dshot.h
│       ├── bsp_sd.h
│       └── bsp_types.h
└── bsp_sys/                # system services — visible to app
    └── include/bsp_sys/
        ├── bsp_timestamp.h
        ├── bsp_intercore.h
        ├── bsp_status_led.h
        └── bsp_init.h        # bsp_kernel_start lives here
```

The two halves still get implemented in the same per-core `flight_controller/CM{4,7}/Bsp/` directory — this is purely a header-namespace split.

CMake then links:
- `drivers/*` PRIVATE → `bsp` + `bsp_sys`
- `app_*` PRIVATE → `bsp_sys` only

Result: a `#include "bsp/bsp_spi.h"` from `app/cm4/controls_task.c` becomes a build error (header not on include path).

---

## Move 5 — Tighten CMake link visibility

Today every `target_link_libraries(... PUBLIC)` transitively re-exports the linked lib. That makes app targets accidentally see every BSP header.

### Proposed rule

| Producer | PUBLIC linkage to | PRIVATE linkage to |
|---|---|---|
| `libs/bsp` (interface) | — (header-only) | — |
| `libs/bsp_sys` (interface) | — (header-only) | — |
| `libs/spi_queue` etc. | `stm32cubemx` | — |
| `libs/drivers/X` | `bsp_sys` | `bsp`, `libs/sensors/*`, `libs/spi_queue`, `libs/uart_dma_cm` |
| `libs/app_shared` | `bsp_sys`, `state_estimation`, etc. | — |
| `libs/app_cm4` | `app_shared`, `bsp_sys`, `drivers/*` | — |
| per-core executable | (everything via app_cmX) | `bsp` (only for the Bsp/ impl .c files), HAL, FreeRTOS |

The takeaway: the **executable** is the one place that sees both halves of the BSP, because it compiles the impl files. Nobody else has both.

---

## Move 6 — Split `bsp_init` from `drivers_init` from `app_init`

### Current

`app_init_cm4()` instantiates drivers **and** creates tasks. They're the same step.

### Proposed

Three phases, three functions, three layers:

```c
/* main.c */
SystemInit();
bsp_init_cm4();        // hardware → ready
drivers_init_cm4();    // sensors_init + log_service init
app_init_cm4();        // create FreeRTOS tasks
bsp_kernel_start();    // never returns
```

`drivers_init_cm4()` lives in `app/cm4/drivers_init_cm4.c` (or directly in `sensors_init.c`); `app_init_cm4()` only does task creation + cross-task wiring. The phases also make initialisation order explicit, which we'll appreciate the first time a driver init has to spin on something.

The reverse on shutdown is symmetric (not relevant on this MCU but is for SIL).

---

## Move 7 — Replace `bool` returns with a `bsp_status_t`

Drivers can't tell *why* a `bsp_spi_submit` failed today — queue full, bus busy, peripheral fault, all collapse to `false`. A typed status lets the driver:

- Retry on transient busy/queue-full
- Escalate fault states to mission manager
- Mark a sensor as offline after N consecutive bus errors

```c
typedef enum {
    BSP_OK = 0,
    BSP_ERR_BUSY,
    BSP_ERR_TIMEOUT,
    BSP_ERR_NACK,
    BSP_ERR_BUS,
    BSP_ERR_NO_DEVICE,
    BSP_ERR_PARAM,
} bsp_status_t;
```

Every BSP entry point returns `bsp_status_t` instead of `bool`. Existing `bool`-returning drivers either propagate or collapse.

---

## What this is *not*

- **Not a redesign**: every interface that exists today still exists; we are tightening the contracts and renaming a few.
- **Not adding RTOS abstractions**: tasks, semaphores, notifications stay FreeRTOS-typed for now. (A future SIL pass will deal with this.)
- **Not introducing C++ niceties**: no inheritance, no templates, no smart pointers.

---

## Implementation order (suggested)

1. **Move 1** (globals → statics) — pure-mechanical, no API changes. ~30 min.
2. **Move 6** (init phase split) — adds clarity, no behavioural change. ~30 min.
3. **Move 4** + **Move 5** (bsp_sys split + CMake visibility) — biggest semantic win; touches every CMakeLists.txt but is mechanical. ~1 h.
4. **Move 2** (opaque handles) — touches every driver header but is the same edit each time. ~1 h.
5. **Move 7** (`bsp_status_t`) — touch every BSP signature; drivers initially just `if (... != BSP_OK)`. ~1 h.
6. **Move 3** (sessions) — most invasive; defer until #2 lands. ~2 h.

Total: roughly half a day of careful edits, no functional risk.

---

## Definition of done

- `grep -rn "^[a-zA-Z_].* g_" app/ libs/drivers/` returns nothing.
- `app/` includes no header under `bsp/` (only `bsp_sys/`).
- Driver headers expose only a forward-declared struct + APIs; struct fields are not externally visible.
- Removing `BSP_SPI_BMI088_ACC` from `bsp_spi.h` (i.e., renaming the enum value) does not require touching any file under `libs/drivers/`.
- `cd libs/controls/tests && ./build/test_controls` still: 93 / 0 / 0.
- `cd libs/state_estimation/tests && ./build/test_state_estimation` still: 32 / 0 / 0.

---

## What still has to land outside this proposal (recap)

These are tracked separately and are not in scope here:

- main.c on each core calling `bsp_init → drivers_init → app_init → bsp_kernel_start`.
- IRQ glue in `stm32h7xx_it.c` for UART CM and HSEM.
- Linker-script `.shared` section in SRAM4, exported `__shared_region_start__`.
- MPU configuration for CM7 D-cache + SRAM4 non-cacheable region.
- Actually filling in the device init sequences (BMI088, ICM-40609, MS5611) and the actuator TIM/DMA wiring (DShot).
- Porting the task bodies from `depracated/`.

The hardening above is a prerequisite for those steps to land *clean*, not a substitute for them.
