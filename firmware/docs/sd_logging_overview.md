# SD Card Logging

> **Status:** In development. The logger streams records to the SD card, but recovery tooling and mission-critical validation are still ongoing.

## Goals

Simplify post-flight analysis by capturing structured telemetry directly to an SD card without needing an external PC. Records follow the shared schema in `Core/Inc/SD_logging/log_records.h`, so firmware and host tools stay in sync.

## Architecture

1. **Detection & init** (`Core/Src/main.c`):
   * `MX_SDMMC1_SD_Init()` only runs when the card-detect GPIO reports a card present.
   * Init configures the SDMMC for 4-bit transfers but intentionally divides the bus clock (`ClockDiv = 4`, ~12 MHz) to keep CRC margins healthy on our board routing; once the card responds, the controller’s internal DMA (IDMA) and NVIC path are enabled so block writes run without CPU copies.
   * `g_sd_card_initialized` reflects whether the card is ready for logging.

2. **Log writer** (`Core/Src/SD_logging/log_writer.c`):
   * Double-buffered 512-byte blocks; each buffer fills with framed records and is flushed via `HAL_SD_WriteBlocks_DMA`.
   * Frames consist of a small header (`magic`, `type`, `payload length`, `CRC16`) followed by the payload, matching the structs in `log_records.h`.
   * A FreeRTOS mutex guards buffer access so any task can enqueue records safely.
   * DMA completion ISR releases a semaphore, allowing the next buffer to be written without blocking the mission manager; errors propagate through `HAL_SD_ErrorCallback`, so the mission tasks immediately stop logging if the card reports CRC/timeouts.
   * Before logging starts, the first ~4 MB of the card are erased to guarantee stale data from earlier flights is cleared.

3. **Log service use in tasks**:
   * Mission manager calls `log_service_try_init`, `log_service_log_event` (E-STOP + flight-state transitions), and `log_service_periodic_flush`. Radio logging hooks will land here as the parser matures.
   * As soon as the SD backend is ready, mission manager emits a `flight_header` record containing a per-flight magic value; decoders can lock onto the newest header to ignore historical data.
   * The state-estimation task logs accelerometer, gyro, and barometer samples after dequeuing them, and also logs each fused state snapshot with the latest flight state. All of this happens in task context.
   * `log_service` (`Core/Src/SD_logging/log_service.c`) hides the framing, CRC, and DMA flush cadence; tasks simply pass physical units (m/s², rad/s, centi-units).

## Host tooling

* `tools/logging/generate_log_schema.py` parses `log_records.h` and produces `tools/logging/log_schema.py`.
* `tools/SD/decode_log.py` walks a raw sector dump, validates CRCs, and emits newline-delimited JSON; by default it expects `tools/SD/bin/log_dump.bin` and writes `tools/SD/logs/flight.jsonl`, so `python tools/SD/decode_log.py` is enough after copying your dump there.
  * Override paths via `python tools/SD/decode_log.py /tmp/dump.bin --output /tmp/flight.jsonl`.
  * Pass `--latest-flight` to automatically focus on the newest flight header, or `--flight-magic 0xDEADBEEF` to pin to a specific run.
* Ground scripts can import the shared schema (see above) to build richer tooling if the stock decoder needs extension.

## Extracting a Raw SD Dump

**macOS / Linux**

1. Insert the SD card and identify the device node (e.g., `diskutil list` on macOS or `lsblk` on Linux).
2. Unmount/open the device so nothing else is writing: `diskutil unmountDisk /dev/disk4` (macOS) or `sudo umount /dev/sdX1` (Linux).
3. Use `dd` against the *raw* device to copy either the first few megabytes or the full card. Example (first 4 MB, matching the firmware’s preflight erase window):
   ```
   sudo dd if=/dev/rdisk4s1 of=$HOME/ubc_rocket/ulysses/tools/SD/logs/log_dump.bin bs=1m count=4
   ```
   Increase `count=` or drop it entirely if you want the full contents. Always point `of=` somewhere inside your user account so permissions stay friendly.

**Windows**

1. Open Disk Management (`diskmgmt.msc`) and note the SD card’s physical drive number (e.g., `PhysicalDrive2`). Make sure the volume is dismounted.
2. From an elevated PowerShell or Command Prompt, run a Windows build of `dd` or use Win32DiskImager. Example with `dd` (from Cygwin/MSYS/GnuWin32):
   ```
   dd if=\\.\PhysicalDrive2 of=C:\logs\log_dump.bin bs=1M --progress
   ```
   Win32DiskImager users can click **Read** to save the entire card into `log_dump.bin`.
3. Move `log_dump.bin` into `tools/SD/logs/` and run `python tools/SD/decode_log.py --latest-flight` to get `flight.jsonl`.

## Small-print / Known limitations

* **WIP:** No filesystem. Data is appended as raw sectors, so you’ll need a custom decoder script to interpret the log.
* **Power loss:** The logger is append-only; unflushed data in the active buffer is lost if power drops. Flush frequency can be tuned in mission manager.
* **Card compatibility:** Initialization has been tested with standard SDHC and some UHS-I cards, but high-speed tuning, wear-level behaviour, and long-duration stress tests remain.
* **Tooling gap:** Scripts to parse the binary log into CSV/plots are planned but not included yet.

Despite the “WIP” status, the framework is designed so that once tooling is ready, the firmware can log arbitrarily complex records without further schema drift.
