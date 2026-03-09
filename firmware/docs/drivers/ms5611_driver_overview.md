# MS5611 Barometer Driver Overview

The MS5611 is a high-resolution barometric pressure sensor used to estimate altitude. Unlike the BMI088, it provides pressure and temperature through two ADC conversions (D1 and D2). This document tracks how the firmware initializes the baro, schedules conversions, reads results over SPI + DMA, and feeds samples into the fusion pipeline.

## Initialization sequence

`Core/Src/spi_drivers/ms5611_device.c` contains `ms5611_init`, which is called from `ms5611_poller_init`. The key steps are:

1. **Reset** – `ms5611_build_reset` constructs the `0x1E` command, which is sent to the sensor with chip select toggled manually.
2. **PROM read** – The MS5611 stores calibration coefficients `C1..C6` (and CRC) in its PROM. The driver loops over all eight addresses via `ms5611_build_prom_read`, reading each 16-bit word and parsing it with `ms5611_parse_prom_word`. The results populate `ms5611_t::C[]`.
3. **CRC check** – `ms5611_check_crc` validates the PROM contents. If the CRC fails, initialization returns error code `2`.
4. **Ring buffer reset** – `ms5611_sample_ring` head and tail pointers are zeroed so consumers start from a clean queue.
5. **Polling state** – `ms5611_poller_init` stores the SPI handle, CS pin, desired oversampling ratio (OSR), and output data rate (ODR). It calls `ms5611_poller_set_rate` to convert OSR into the required conversion delay (microseconds) and the interval between conversions.

If initialization succeeds, the poller is ready to orchestrate conversions. Failures (PROM parse or CRC) keep the poller inactive and allow future retries or error reporting.

## Poller and timing

The barometer does not raise interrupts; instead, the driver runs a deterministic conversion sequence driven by a 1 kHz tick:

* `ms5611_poller_tick_1khz(ms5611_poller_t *p)` is called from `state_estimation_task_start` once per millisecond.
* The poller maintains a simple state machine:
  1. Issue a D1 (pressure) conversion command (`ms5611_submit_convert_d1`).
  2. Wait the required conversion time (`p->conv_us` converted to ticks).
  3. Read D1 via `ms5611_submit_read_d1` (uses the SPI job queue and DMA).
  4. Immediately issue a D2 (temperature) conversion (`ms5611_submit_convert_d2`).
  5. Wait, then read D2 (`ms5611_submit_read_d2`), compute compensated values, and enqueue a sample.
* The cycle repeats at the requested ODR (e.g., 50 Hz).

This polling avoids busy-waiting by spacing out conversions and using the SPI queue to perform each transaction asynchronously.

## SPI job flow

Each `*_submit_*` helper builds a `spi_job_t`:

* `cs_port` / `cs_pin` target `BARO1_CS`.
* `len` is determined by the command builder (`ms5611_build_convert_*` or `ms5611_build_adc_read`).
* `type` is `SPI_XFER_TX` for convert commands (only command bytes sent) and `SPI_XFER_TXRX` for ADC reads (command + dummy read).
* `done` is set for read jobs:
  * `ms5611_cb_read_d1` parses the ADC result from `rx[1]` (first byte is dummy), stores it in `p->dev.D1_raw`, and notes the timestamp.
  * `ms5611_cb_read_d2` parses the D2 result, calls `ms5611_compute` to derive temperature and pressure in centi-units, and creates an `ms5611_sample_t`.

Jobs are submitted to `jobq_spi_2` just like the BMI088 drivers, so the barometer shares the same DMA path and arbitration logic.

## Sample formatting and queueing

When both D1 and D2 have been collected, `ms5611_cb_read_d2` executes:

1. `ms5611_parse_adc_result` extracts a 24-bit raw value.
2. `ms5611_compute` runs the datasheet compensation formulas using stored calibration constants, producing:
   * `temp_centi` – temperature in 0.01 °C.
   * `pressure_centi` – pressure in 0.01 mbar (also equals 0.01 hPa).
3. A `ms5611_sample_t` is filled with timestamps (`t_us`), raw ADC readings, compensated values, and a monotonically increasing sequence number.
4. The sample is enqueued via `ms5611_sample_queue(&ms5611_sample_ring, &sample)` for consumers to read later.

## Consumption by higher-level tasks

* The `state_estimation_task_start` loop dequeues up to 32 barometer samples per iteration and logs/uses them the same way as accelerometer/gyro samples.
* Downstream code can call `ms5611_fetch_latest` to obtain the most recent `ms5611_t` structure plus sequence number without draining the sample ring.
* The fused state published through `state_exchange_publish_state` incorporates barometric altitude as one of the navigation observables.

## Key properties

* **Deterministic cadence:** The conversion state machine ensures pressure and temperature are always paired, even though each requires a separate ADC cycle.
* **DMA-backed transfers:** Every SPI transaction, including the short convert commands, runs through the shared job queue, simplifying bus arbitration with other sensors.
* **Calibration-awareness:** PROM coefficients are read once at startup and cached, so subsequent computations are quick and performed entirely in firmware.
* **Thread-safe buffers:** The ring buffer decouples the high-rate poller from slower consumers, preventing timing jitter in the EKF loop.

With this driver, the MS5611 delivers high-resolution pressure data synchronized with the IMU streams, supporting accurate altitude estimation and flight-state decisions.
