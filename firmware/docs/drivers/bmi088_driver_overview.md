# BMI088 IMU Driver Overview

This note documents how the Ulysses firmware configures and samples the Bosch BMI088 inertial measurement unit (IMU), which integrates a three-axis accelerometer and a three-axis gyroscope. It covers the initialization sequence, data-ready flow, SPI job wiring, and how raw samples are converted into physical units.

## Accelerometer path

### Initialization sequence

`Core/Src/spi_drivers/bmi088_accel_device.c` performs a deterministic configuration ordered as follows (matching the Bosch datasheet requirements):

1. **Soft reset** – `bmi088_accel_build_softreset` writes `0xB6` into `ACC_SOFTRESET`, guaranteeing the device starts from a known state.
2. **Chip ID validation** – read from `ACC_CHIP_ID` and compare against `0x1E`. If the ID differs, the driver aborts with error code `1`.
3. **Power-up** – two writes of `bmi088_accel_build_power_conf` ensure `Pwr_Conf` exits suspend and `Pwr_Ctrl` enables the accelerometer core (expecting `Pwr_Ctrl = 0x04`). Failure returns error codes `2` or `3`.
4. **Range** – `bmi088_accel_build_range(BMI088_ACC_RANGE_3G, …)` selects ±3 g and sets the internal scale factor (mg/LSB) used later during parsing.
5. **Bandwidth / ODR** – `bmi088_accel_build_odr_bw(BMI088_ACC_ODR_800_HZ, BMI088_ACC_BWP_NORMAL, …)` requests an 800 Hz data rate with normal bandwidth, balancing latency and noise.
6. **INT1 routing** – `bmi088_accel_build_int_pin_conf` configures INT1 as push-pull, active-high, non-latching; `bmi088_accel_build_int_event_map` maps the DATA_READY event onto that pin.
7. **Sanity checks** – multiple reads of `STATUS`, `ACC_CONF`, `ACC_RANGE` confirm the configuration stuck, validating that bit 7 of STATUS (data ready) toggles.

Each SPI transaction toggles the accelerometer’s chip-select (`BMI_ACC_CS`) manually with small inter-transfer delays (`delay_us`) to meet datasheet timing. Initialization returns `0` on success, or a small non-zero code indicating where validation failed.

### Data-ready and job submission

* The BMI088 accelerometer asserts INT1 every time a new sample is available. This pin is wired to `BMI_ACC_INT_1_Pin`, which triggers the EXTI path handled in `Core/Src/interrupts.c`.
* In `HAL_GPIO_EXTI_Rising_Callback`, the code checks `bmi088_accel_ready` and calls `bmi088_accel_interrupt()`.
* `bmi088_accel_interrupt()` constructs a `spi_job_t` with:
  - `cs_port = BMI_ACC_Chip_Select_GPIO_Port`, `cs_pin = BMI_ACC_Chip_Select_Pin`.
  - `len = bmi088_accel_build_read(BMI088_ACC_READ_DATA_XYZ, job.tx)` to queue a six-byte XYZ read plus command overhead.
  - `t_sample = timestamp_us()` so the sample retains timing metadata.
  - `type = SPI_XFER_TXRX` because SPI needs dummy clocks to receive data.
  - `done = bmi088_accel_done`, which parses received bytes and enqueues the decoded vector.
  - `sensor = SENSOR_ID_ACCEL` for bookkeeping.
* The job is submitted to the shared queue `jobq_spi_2` via `spi_submit_job`. From here the generic SPI + DMA machinery described in the previous document executes the transfer, and the completion callback runs inside the DMA ISR.

### Sample parsing and buffering

`bmi088_accel_done` receives the RX buffer (starting at byte 2 because the first two bytes are command/dummy). It uses `bmi088_accel_parse_data_xyz` to translate raw 16-bit counts into m/s² values by applying the scale factor saved during initialization. The callback then sets `sample.t_us = job->t_sample` and pushes the struct into `bmi088_acc_sample_ring`, a lock-free ring buffer defined in `Core/Inc/spi_drivers/BMI088_accel.h`.

The ring buffer feeds `state_estimation_task_start`, which drains up to 32 samples per loop iteration for fusion.

## Gyroscope path

The gyro follows the same pattern but with gyro-specific registers and conversion factors.

### Initialization highlights

Located in `Core/Src/spi_drivers/bmi088_gyro_device.c`:

1. **Soft reset & chip ID** – ensures the device responds with `BMI088_GYRO_CHIP_ID_VALUE`.
2. **Power enable** – sets `GYRO_PWR` registers to normal operating mode.
3. **Output data rate / bandwidth** – configured via `bmi088_gyro_build_range_and_bw` (e.g., 2000 °/s, 2000 Hz bandwidth, depending on mission requirements).
4. **Interrupt routing** – INT3/INT4 pins are configured similarly to the accelerometer so data-ready events produce GPIO interrupts.

Each step mirrors the accelerometer flow so both sensors are ready before data collection begins.

### Interrupt + job submission

* The gyro’s data-ready pin maps to `BMI_GYRO_INT_1_Pin`. When `HAL_GPIO_EXTI_Rising_Callback` sees this pin and `bmi088_gyro_ready` is true, it calls `bmi088_gyro_interrupt()`.
* That function builds a `spi_job_t` containing a FIFO read or direct XYZ read depending on configuration. The sample timestamp and completion callback (`bmi088_gyro_done`) are populated in the same manner as the accelerometer path.
* Jobs are submitted to the same `jobq_spi_2` queue, meaning accelerometer and gyro requests are arbitrated in arrival order.

### Parsing and buffering

`bmi088_gyro_done` calls `bmi088_gyro_parse_data_xyz` (or FIFO parser) to convert raw counts into rad/s values using the range-dependent LSB scale. Parsed samples are pushed into `bmi088_gyro_sample_ring` for later use. The state-estimation task consumes these alongside the accelerometer and barometer samples to build the fused attitude solution.

## Failure handling and readiness flags

Both drivers expose `bmi088_accel_ready` / `bmi088_gyro_ready` flags that gate the EXTI callback. Initialization sets these flags true only if setup succeeded. If initialization fails (chip ID mismatch, power state incorrect, etc.), the flags remain false, preventing noisy interrupts from scheduling SPI jobs that would inevitably fail. This guard keeps the SPI queue from filling with invalid work and leaves room for future retries or fault reporting.

## How this ties into mission logic

* The accelerometer and gyro sample rings feed `state_exchange` and the `state_estimation` task, which compute vehicle attitude and rates.
* Mission manager logic (e.g., the orientation-based e-stop) relies on this fused state to decide whether to abort.
* The debug UART now prints a subset of recent accelerometer/gyro samples, leveraging the same ring buffers, making it easier to validate driver behaviour.

With these drivers, the BMI088 IMU delivers high-rate inertial data with precise timestamps, consistent configuration, and minimal CPU overhead thanks to the shared SPI DMA job queue.
