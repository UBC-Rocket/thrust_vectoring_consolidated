# IO layer

Transport between hardware peripherals and device drivers. Drivers and the
app code program against the headers in [`io/include/io/`](io/include/io/)
(transport — SPI, UART, EXTI, DShot) and [`io_sys/include/io_sys/`](io_sys/include/io_sys/)
(system services — timestamp, intercore, status LED, SD, init, shared types).

## Contents

| Path | What |
|---|---|
| `io/`                  | `io` interface lib — transport headers (DEV-only) |
| `io_sys/`              | `io_sys` interface lib — system-service headers (APP + DEV) |
| `io_spi_queue/`        | Generic SPI job-queue helper used by per-target SPI impls |
| `io_uart_dma_cm/`      | DMA-circular + character-match UART helper |
| `io_intercore_slot/`   | Seqlock shared-memory primitive for the H747 dual-core |
| `h747/CM4/`            | H747 CM4 IO impl (defines IO_SPI_*, IO_UART_*, IO_EXTI_* symbols) |
| `h747/CM7/`            | H747 CM7 IO impl |

## SIL substitution

The IO layer is the only place a SIL backend needs to replace. Add a peer
target dir (e.g. `sil/`) defining the same `IO_*` const symbols and the same
`io_*` functions; everything above (DEV, APP) links unchanged.
