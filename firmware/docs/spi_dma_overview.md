# SPI + DMA Sensor Bus Overview

This note explains how the flight computer shares a single SPI bus among multiple sensors by combining a DMA-powered transfer engine with a small job queue. It is written for a general audience, so every term is defined as it appears.

## Why this architecture exists

The BMI088 accelerometer, BMI088 gyro, and MS5611 barometer all speak **SPI** (Serial Peripheral Interface), a protocol where a master toggles a **chip-select (CS)** line to talk to one slave at a time. Only one transfer can be “on the wire” at any given instant. Polling each device inside a loop would waste CPU cycles and risks missing samples, so Ulysses uses:

1. **Data-ready interrupts** (BMI088 INT1, gyro INT, baro timing) so sensors tell us exactly when a sample is available (`Core/Src/interrupts.c`).
2. A **job queue** that records “who wants the bus next” in FIFO order (`Core/Src/spi_drivers/SPI_queue.c`).
3. **DMA (Direct Memory Access)** so the SPI peripheral moves bytes to/from memory without CPU involvement, freeing the Cortex‑M33 to run navigation tasks.

The combination guarantees lossless acquisition even when multiple sensors assert their data-ready pins at nearly the same moment.

## Glossary

| Term | Definition |
| --- | --- |
| **SPI transfer** | A burst of clocked bits where the MCU simultaneously transmits (`MOSI`) and/or receives (`MISO`) data. All transfers specify a CS pin so the correct sensor listens. |
| **DMA** | Hardware that copies memory for a peripheral without the CPU touching every byte. The STM32H5 SPI block can request DMA for transmit, receive, or both. |
| **Job** | One queued SPI transaction described by `spi_job_t` (CS port/pin, transmit buffer, length, transfer type, completion callback, timestamp, and sensor identifier). |
| **Job queue** | A circular buffer (`spi_job_queue_t`) that stores pending jobs plus bookkeeping (head/tail pointers, “bus busy” flag, staging buffer for RX data). |
| **Data-ready interrupt** | A GPIO interrupt that fires when a sensor asserts “new data available.” In `HAL_GPIO_EXTI_Rising_Callback` these interrupts dispatch to `bmi088_accel_interrupt()` or `bmi088_gyro_interrupt()` to enqueue SPI work. |

## End-to-end flow

1. **Interrupt fires**  
   - The BMI088 accelerometer pulses `BMI_ACC_INT_1_Pin`. HAL routes this to `HAL_GPIO_EXTI_Rising_Callback` in `Core/Src/interrupts.c`.  
   - We check `bmi088_accel_ready` and call `bmi088_accel_interrupt()`.

2. **Job creation** (`Core/Src/spi_drivers/bmi088_accel_device.c`)  
   - The interrupt handler populates a `spi_job_t` with:
     * CS port/pin for the accelerometer.
     * A transmit buffer built by `bmi088_accel_build_read(...)`, which contains the SPI read command and dummy bytes.
     * The sample timestamp (`timestamp_us()`), so we can reconstruct when the sensor captured the data.
     * The completion callback `bmi088_accel_done`, which parses the returned bytes into physical acceleration and pushes it into `bmi088_acc_sample_ring`.
   - The job is submitted via `spi_submit_job(job, &jobq_spi_2)` (job queue bound to SPI2).

3. **Queue arbitration** (`Core/Src/spi_drivers/SPI_queue.c`)  
   - `spi_submit_job` briefly disables interrupts, checks whether the bus is idle, and either:
     * Starts the job immediately (`start_job`), or
     * Copies it into the circular queue if the bus is already busy.
   - This guarantees only one DMA transfer is active at a time, preventing chip-select collisions.

4. **DMA transfer**  
   - `start_job` asserts the sensor’s CS line, unlocks the HAL SPI handle (because HAL leaves it locked inside IRQ context), and starts the appropriate DMA mode (`HAL_SPI_Transmit_DMA`, `_Receive_DMA`, or `_TransmitReceive_DMA`).  
   - DMA pushes bytes directly between memory and the SPI peripheral while the CPU continues running FreeRTOS tasks.

5. **Completion ISR**  
   - When DMA finishes, HAL calls `HAL_SPI_*CpltCallback`, which funnels into `spi_dma_complete_common`.  
   - We deassert CS, invoke the job’s `done` callback (still inside ISR context), and parse the sensor payload.  
   - The job queue is checked for pending work; if another job exists, `start_job` is called immediately to keep the bus busy with virtually no gap.

6. **Data ready again**  
   - The sensor queues the next transfer the next time it asserts the data-ready pin, and the process repeats.

## Why use DMA + job queue?

* **Deterministic timing:** Data-ready interrupts mean the MCU responds right when the sensor has new data, reducing latency compared to polling.
* **Bus safety:** The queue serializes access so two devices never drive the SPI lines simultaneously, which could otherwise corrupt readings.
* **CPU efficiency:** DMA offloads the byte shoveling. The CPU only handles small control steps (queueing jobs, parsing results) which is critical during the high-rate 1 kHz sensor fusion loop.
* **ISR-friendly completion:** Each job records a callback that is safe to run inside the DMA completion ISR. That callback performs lightweight parsing and drops samples into lock-free rings (`bmi088_acc_sample_ring`, `bmi088_gyro_sample_ring`, `ms5611_sample_ring`). Higher-level tasks (e.g., `state_estimation_task_start`) pull from those rings without blocking the interrupt path.

## Interplay with the rest of the system

* `Core/Src/tasks/state_estimation.c` pulls fused samples from the sensor rings, producing a navigation state vector consumed by mission logic.
* `Core/Src/interrupts.c` ties the physical GPIO interrupts into the SPI job submission functions.
* `Core/Src/spi_drivers/*` encapsulate device-specific details (how to build read commands, how to parse payloads), keeping the queue generic.

Together these pieces form a responsive, low-overhead sensor acquisition path suitable for rockets, UAVs, or any embedded application where multiple SPI sensors must coexist without missing data.
