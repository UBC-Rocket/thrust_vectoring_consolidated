# HW / IO / Dev / App Layer Architecture

**Status:** implemented (originally proposed 2026-05-16).
**Supersedes:** [layer_hardening_proposal.md](layer_hardening_proposal.md) (v1) — v1 proposed "app does the IO wiring", which we rejected in favour of self-contained device drivers.

> **Note on paths.** The proposal section below uses the as-proposed names
> for clarity. The implementation made a few small adjustments to keep IO
> helper libs consistent: `libs/spi_queue` → `io/io_spi_queue`,
> `libs/uart_dma_cm` → `io/io_uart_dma_cm`, `libs/intercore` →
> `io/io_intercore_slot`. DEV files are flat (`dev/imu_bmi088/include/dev_imu_bmi088.h`).
> Per-target IO impl filenames dropped the `_cm{4,7}` suffix
> (`io/h747/CM4/io_timestamp.c`) since the parent directory already
> conveys the core.

---

## The model

Four layers, bottom-up. The arrows are "may call"; nothing reaches sideways or downward across more than one layer.

```
┌─────────────────────────────────────────────────────────────────┐
│                              APP                                │
│         (FreeRTOS tasks, state exchange, mission FSM,           │
│          log service, controls glue)                            │
├────────────────────────────▲────────────────────────────────────┤
│                              │                                  │
│                              ▼                                  │
│                              DEV                                │
│      (per-device drivers: imu_bmi088, imu_icm40609,             │
│       baro_ms5611, gps_nmea, radio_rfd900, esc_dshot,           │
│       servo_feetech — each self-contained, knows its            │
│       own peripheral wiring)                                    │
├────────────────────────────▲────────────────────────────────────┤
│                              │                                  │
│                              ▼                                  │
│                              IO                                 │
│      (target-specific peripheral utilities:                     │
│       SPI job queue + DMA callbacks, UART DMA + char-match,     │
│       EXTI dispatch, DShot bit-bang, SDMMC async writer,        │
│       timestamp source, inter-core HSEM + shared memory)        │
├────────────────────────────▲────────────────────────────────────┤
│                              │                                  │
│                              ▼                                  │
│                              HW                                 │
│      (ST HAL, CubeMX-generated peripheral init, GPIO mux,       │
│       clock tree, TIM / DMA primitives)                         │
└─────────────────────────────────────────────────────────────────┘
```

What each layer **owns**:

| Layer | Owns | Touches downward | Does **not** touch |
|---|---|---|---|
| **APP** | Tasks, scheduling glue, mission state, state exchange, logging | DEV drivers, IO system services (timestamp, intercore) | IO transports (SPI/UART/EXTI/DShot), HW |
| **DEV** | Device protocol, calibration defaults, EXTI handler, sample ring, init sequence — **all per-device config baked in** | IO layer by symbolic resource name (`IO_SPI_BMI088_ACC`, `IO_EXTI_BMI088_INT1`, …) | HW directly |
| **IO** | DMA pipelines, peripheral callbacks, per-peripheral resource symbols, timestamps, intercore, status LEDs | HW (ST HAL) | DEV, APP |
| **HW** | Bare-metal peripheral init, register-level wiring, clock setup | — | anything above |

Why this carving:
- **Drivers are self-contained.** A driver knows it talks to "BMI088 accel on SPI4 with CS on PE3 and INT on PC13", because that's a property of the board, and the board is fixed. The app doesn't pick the wiring at boot — it doesn't need to.
- **IO is the SIL substitution boundary.** Swap `io/h747/` for `io/sil/`; the symbols a driver references (`IO_SPI_BMI088_ACC`, etc.) resolve to a simulated peripheral that returns canned register responses. The driver source is unchanged.
- **App never sees a `HAL_*` symbol, never sees `hspi4`, never sees a CS pin.** The most it sees is `io_timestamp_us()`.

---

## Resource binding: symbolic, owned by IO

Drivers don't accept config structs. They reference IO resources by name:

```c
/* dev/imu_bmi088/imu_bmi088.c */
#include "io/io_spi.h"
#include "io/io_exti.h"

extern const io_spi_dev_t   IO_SPI_BMI088_ACC;
extern const io_spi_dev_t   IO_SPI_BMI088_GYRO;
extern const io_exti_line_t IO_EXTI_BMI088_ACC_INT1;
extern const io_exti_line_t IO_EXTI_BMI088_GYRO_INT1;

static struct {
    /* per-driver state: queue, sample ring, notify target, dev regs */
} s_self;

static void on_acc_data_ready(void *user) { /* submit SPI read */ }

bool imu_bmi088_init(void) {
    io_exti_register(&IO_EXTI_BMI088_ACC_INT1, on_acc_data_ready, &s_self);
    /* … config sequence via io_spi_xfer_blocking(&IO_SPI_BMI088_ACC, …) */
    return true;
}
```

The resource symbols are defined by the per-target IO layer:

```c
/* io/h747/cm4/io_spi_resources.c */
const io_spi_dev_t IO_SPI_BMI088_ACC = {
    .bus     = &g_spi4_bus,          /* file-private spi_queue instance */
    .cs_port = GPIOE,
    .cs_pin  = GPIO_PIN_3,
};
const io_spi_dev_t IO_SPI_BMI088_GYRO = {
    .bus     = &g_spi4_bus,
    .cs_port = GPIOE,
    .cs_pin  = GPIO_PIN_4,
};
/* … */
```

Swap target:
```c
/* io/sil/io_spi_resources.c */
const io_spi_dev_t IO_SPI_BMI088_ACC = {
    .sim = &g_sim_bmi088_acc,        /* simulator handle */
};
```

The driver doesn't notice. **What changes between targets is the IO layer's symbol table.** The driver source is target-independent code that happens to reference target-defined symbols.

Calibration values that *do* vary per assembly (servo trim, IMU bias, baro reference) come from a non-volatile config service (flash sector / SD card) at runtime, not from compile-time config structs. The driver carries the defaults; the config service may override.

---

## What this means for the current tree

Renames:

| Today | After |
|---|---|
| `libs/bsp/` (interface headers) | `libs/io/` |
| `libs/bsp/include/bsp/bsp_*.h` | `libs/io/include/io/io_*.h` |
| `libs/drivers/` | `libs/dev/` |
| `libs/drivers/<x>/include/drivers/<x>.h` | `libs/dev/<x>/include/dev/<x>.h` |
| `firmware/flight_controller/CM{4,7}/Bsp/` | `firmware/flight_controller/CM{4,7}/Io/` |
| `bsp_init_cm{4,7}()` | `io_init_cm{4,7}()` |
| `bsp_kernel_start()` | stays at top level, e.g. `app_start_kernel()` (it's a scheduler hand-off, not an IO concern) |

The `libs/sensors/` parser helpers stay where they are — they sit at the same level as a vendor driver would (pure C, no I/O), and the DEV layer pulls them in.

`libs/spi_queue/`, `libs/uart_dma_cm/`, `libs/intercore/` are implementation details of the IO layer; they remain separate libs so they can be unit-tested, but they only ever link into IO impl files. They are not in the public IO API.

---

## Moves

### Move 1 — Privatise every global; expose getters only

Anything currently at module scope without `static` becomes `static`. If something outside the TU needs access, it gets a getter.

Concrete targets:
- `app/cm7/app_init_cm7.c` — `g_servos`, `g_escs` become file-private statics. Access via `actuators_handles()` from a new `app/cm7/actuators_init.{c,h}`, mirror of [sensors_init.h](../app/include/app/sensors_init.h).
- Audit and fix any other `extern`-able definition. (Grep target: `^[A-Za-z_].* [a-z][a-zA-Z_0-9]+ *=`.)
- DEV drivers themselves: since each driver becomes singleton-by-design (Move 2), the only "global" is the singleton struct, which is `static` inside the .c.

### Move 2 — DEV drivers are opaque singletons; no caller-allocated structs

Driver public header exposes only the API. The state struct is forward-declared and defined in the .c.

```c
/* libs/dev/imu_bmi088/include/dev/imu_bmi088.h */
typedef struct imu_bmi088 imu_bmi088_t;     /* opaque */

bool                 imu_bmi088_init(void);
size_t               imu_bmi088_drain(imu_sample_t *out, size_t max);
void                 imu_bmi088_set_notify(io_task_handle_t task, uint32_t bit);
const imu_bmi088_t  *imu_bmi088_get(void);   /* NULL before init */
```

```c
/* libs/dev/imu_bmi088/src/imu_bmi088.c */
struct imu_bmi088 { /* private fields */ };
static struct imu_bmi088 s_self;
```

App code references the driver only by these calls; it never allocates an `imu_bmi088_t`.

`sensors_t` / `actuators_t` view-structs in the APP layer become read-only pointer bundles populated from `imu_bmi088_get()` etc., for ergonomics in task code.

### Move 3 — DEV drivers carry their own peripheral binding

No more `imu_bmi088_cfg_t` / `servo_feetech_cfg_t` from the caller. The driver `.c` file is the single source of truth for "which peripheral this device is on". The driver references IO resource symbols directly (extern-declared from `io/<target>/`).

```c
/* libs/dev/servo_feetech/src/servo_feetech.c */
#include "io/io_uart.h"
extern const io_uart_id_t IO_UART_SERVO_BUS;

static const uint8_t SERVO_ID_X = 1;
static const uint8_t SERVO_ID_Y = 2;
static const uint16_t POS_MIN   = 1024;
static const uint16_t POS_MID   = 2048;
static const uint16_t POS_MAX   = 3072;
static const float    DEG_RANGE = 90.0f;
```

This is the user-requested correction to v1: **config lives in the driver, not in the caller**.

Tradeoff accepted: porting a driver to a new board requires editing the driver. That's fine — different board, different driver. The protocol layer (`libs/sensors/bmi088_*.c`) is the part that's board-independent and stays reusable.

### Move 4 — Rename `bsp` → `io`, `drivers` → `dev`

Mechanical rename. Headers move from `bsp/` → `io/`, `drivers/` → `dev/`. Update include paths everywhere. Update CMake target names (`bsp` → `io`, `drivers_imu_bmi088` → `dev_imu_bmi088`, etc.). The HW layer keeps living inside `flight_controller/CM{4,7}/Core/` (CubeMX-generated, untouched).

### Move 5 — Split IO surfaces by who's allowed to see them

Two interface namespaces inside the IO layer:

```
libs/io/include/
├── io/             # transports — DEV only
│   ├── io_spi.h
│   ├── io_uart.h
│   ├── io_exti.h
│   ├── io_dshot.h
│   ├── io_sd.h
│   └── io_types.h
└── io_sys/         # system services — APP and DEV
    ├── io_timestamp.h
    ├── io_intercore.h
    ├── io_status_led.h
    └── io_init.h
```

CMake link rules (Move 6) prevent APP from `#include`-ing transport headers.

### Move 6 — Tighten CMake link visibility

| Target | PUBLIC link | PRIVATE link |
|---|---|---|
| `io` (interface) | — | — |
| `io_sys` (interface) | — | — |
| `spi_queue`, `uart_dma_cm`, `intercore` | `stm32cubemx` | — |
| `dev_<name>` (each device driver) | `io_sys` | `io`, `sensors`, `spi_queue`/`uart_dma_cm` as needed |
| `app_shared` | `io_sys`, `state_estimation`, `log_records`, … | — |
| `app_cm{4,7}` | `app_shared`, `io_sys`, `dev_*` | — |
| per-core executable | — | `io` (only because it compiles the IO impl .c files), HAL, FreeRTOS |

Effect:
- `#include "io/io_spi.h"` from `app/cm4/state_estimation_task.c` → build error (header not on the include path of `app_cm4`'s transitive deps).
- `#include "io/io_spi.h"` from `dev/imu_bmi088/src/imu_bmi088.c` → OK (`io` is PRIVATE-linked).
- The executable transitively gets all of IO via its app + dev deps, plus directly via its own Bsp/Io impl .c sources. Nobody else sees the whole pile.

### Move 7 — Four explicit init phases, owned bottom-up

```c
int main(void) {
    SystemInit();              /* HW boot, clock, MPU */
    io_init_cm4();             /* IO layer: bring up spi_queue/uart/exti/etc. */
    dev_init_cm4();            /* DEV layer: each driver self-inits */
    app_init_cm4();            /* APP layer: create tasks, wire notifies */
    app_start_kernel();        /* never returns */
}
```

`dev_init_cm4()` is just:
```c
void dev_init_cm4(void) {
    imu_bmi088_init();
    imu_icm40609_init();
    baro_ms5611_init();
    gps_nmea_init();
    radio_rfd900_init();
}
```
…and the CM7 equivalent inits its actuators. No app code knows about init ordering between drivers.

`app_start_kernel()` is the scheduler entry; on H747 it's `vTaskStartScheduler()`; on SIL it's `pthread_join` over the simulated thread pool.

### Move 8 — `io_status_t` instead of `bool` returns

Every IO entry point returns a typed status:
```c
typedef enum {
    IO_OK = 0,
    IO_ERR_BUSY,
    IO_ERR_TIMEOUT,
    IO_ERR_NACK,
    IO_ERR_BUS,
    IO_ERR_NO_DEVICE,
    IO_ERR_PARAM,
} io_status_t;
```

DEV drivers can react meaningfully (retry on `BUSY`, mark sensor offline after N `BUS`, etc.). Initial conversion is mechanical (`bool` → `IO_OK`).

---

## Implementation order

| # | Move | Risk | Effort |
|---|---|---|---|
| 1 | Privatise globals (Move 1) | very low | 30 min |
| 2 | Rename `bsp` → `io`, `drivers` → `dev` (Move 4) | very low — pure rename | 1 h |
| 3 | Four-phase init (Move 7) | low | 30 min |
| 4 | Split IO surfaces (Move 5) + CMake visibility (Move 6) | low — mechanical | 1 h |
| 5 | Drop driver config structs; symbolic IO binding (Move 3) | medium — touches every driver | 1.5 h |
| 6 | Opaque DEV singletons (Move 2) | medium — touches every driver header | 1 h |
| 7 | `io_status_t` typed returns (Move 8) | low — touches every IO call site, but mechanical | 1 h |

Total: ~6 hours, fully reversible at each step.

---

## Definition of done

- `grep -rn "^[A-Za-z_].* [a-zA-Z_][a-zA-Z_0-9]* *= " app/ libs/dev/ libs/io/` returns only `static` definitions and `const`-table initialisers.
- `app/` translation units include nothing under `io/` — only `io_sys/`, `app/`, `dev/`, `controls/`, `state_estimation/`, `log_records/`. Enforced by CMake (Move 6) and verified by a grep at PR time.
- DEV driver headers contain only forward-declared opaque types + APIs. No struct field is externally visible.
- Renaming or remapping `IO_SPI_BMI088_ACC` in `io/h747/cm4/io_spi_resources.c` does not require any edit under `libs/dev/`.
- An empty `io/sil/` stub directory exists with placeholder symbol definitions; an HD build (host-defined `IO_TARGET_SIL`) of `dev/` + `app/` links successfully against the stub. This is the lowest-cost SIL-readiness gate.
- `cd libs/controls/tests && ./build/test_controls` → 93 / 0 / 0.
- `cd libs/state_estimation/tests && ./build/test_state_estimation` → 32 / 0 / 0.

---

## Explicit non-goals

- No introduction of C++.
- No RTOS abstraction layer above FreeRTOS for now (will reconsider when SIL lands).
- No code generation, no DSL, no per-board config files driven from JSON/YAML.
- No change to `state_exchange` / `log_service` interfaces.
- Not in scope: actually filling in BMI088 / ICM-40609 / MS5611 init bodies, DShot TIM+DMA wiring, linker `.shared` section, MPU config. Those land independently after the hardening is in place.

---

## Why this is better than v1

- **Simpler model**: no app-side "binding" step. The board is a fixed thing; the driver knows it.
- **Easier to read**: looking at `dev/imu_bmi088/src/imu_bmi088.c` tells you everything about how the BMI088 is wired, with no jumping through app code.
- **SIL story sharper**: the substitution boundary is exactly the IO layer's resource-symbol table. Drivers don't need to know SIL exists.
- **Honest about reality**: drivers were always going to know their wiring (you can't write a sensible BMI088 register sequence without knowing which CS to assert). v1 tried to pretend otherwise via abstraction. v2 names the truth.
