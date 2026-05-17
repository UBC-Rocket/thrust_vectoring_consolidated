# Firmware

All embedded firmware and shared libraries for the TVR project.

## Layer model

```
APP   app/                 hardware-agnostic tasks, state exchange, log service
DEV   dev/                 per-device drivers (opaque singletons)
IO    io/                  transport layer (SPI/UART/EXTI/DShot) +
                           system services (timestamp/intercore/LED/SD/init)
HW    flight_controller/   CubeMX project — STM32H747 dual-core (CM4 + CM7)
                           includes HAL, FreeRTOS, peripheral init
```

Plus:

| Top-level | Purpose |
|---|---|
| `lib/`              | Pure-C utility libs (controls, state_estimation, log_records, …) |
| `sil/`              | (placeholder) future POSIX SIL backend for the IO layer |
| `docs/`             | Architecture proposals, peripheral notes, driver notes |
| `depracated/`       | Old H5 flight controller + G0 GNSS daughter board, retained for reference |
| `tools/`            | Host-side Python tools (SD log decoding, serial debug) |

## Building (on-target)

The CubeMX project at `flight_controller/` orchestrates two `ExternalProject`
sub-builds (one per core):

```sh
cd flight_controller
cmake --preset debug && cmake --build --preset debug
```

Each per-core build pulls in `io/`, `dev/`, `app/`, and `lib/` via
`add_subdirectory` — they don't need to be configured separately.

## Testing (host)

```sh
cd lib/controls/tests         && cmake -B build && cmake --build build && ./build/test_controls
cd lib/state_estimation/tests && cmake -B build && cmake --build build && ./build/test_state_estimation
```

## Further reading

- [`docs/architecture_layers.md`](docs/architecture_layers.md) — the HW / IO / DEV / APP layer proposal
- [`docs/spi_dma_overview.md`](docs/spi_dma_overview.md) — SPI job queue + DMA
- [`docs/sd_logging_overview.md`](docs/sd_logging_overview.md) — binary log design
