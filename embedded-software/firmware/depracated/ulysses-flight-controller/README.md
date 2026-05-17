# Ulysses

**Open-source flight controller firmware for UBC Rocket's experimental VTOL platform**

Ulysses powers UBC Rocket's Coaxial Counter-Rotating Vertical Take-off and Landing Drone — a testbed designed for developing and validating advanced rocket flight control algorithms in real-world conditions.

Built on the **STM32H563** (Cortex-M33, 250 MHz) running **FreeRTOS**, the firmware fuses IMU, barometer, and GPS data through an Extended Kalman Filter, drives a PID-based flight controller at 800 Hz, and communicates over a protobuf-based radio link.

## Table of Contents

- [Project Roadmap](#project-roadmap)
- [RTOS Architecture](#rtos-architecture)
- [Prerequisites](#prerequisites)
- [Building](#building)
- [Flashing and Debugging](#flashing-and-debugging)
- [Project Structure](#project-structure)
- [Contributing](#contributing)

## Project Roadmap

| Phase | Deliverables | Planned Completion |
|---|---|---|
| Phase 1 | Prototype capable of moving up, hovering, and landing | Jan 2026 |
| Phase 2 | Prototype capable of moving along a horizontal plane | May 2026 |
| Phase 3 | Swapping from PID to MPC control; FPGA integration; general polishing | Late 2026 |

## RTOS Architecture

The firmware runs four FreeRTOS tasks with the following priority hierarchy:

```
Priority High ──────────── Controls (800 Hz)
                           Debug Logging (optional)
Priority AboveNormal ───── State Estimation (event-driven)
Priority Normal ────────── Mission Manager (10 Hz telemetry)
```

### Controls Task

| | |
|---|---|
| **Priority** | High |
| **Rate** | 800 Hz (TIM4 interrupt-driven) |
| **Stack** | 2,048 bytes |

Runs the flight controller loop. Reads the latest fused state from the state exchange, computes attitude corrections (PD controller) and thrust commands (PID controller), and outputs PWM signals to servos (gimbal) and ESCs (motors). Includes a staleness check — outputs zero control if state data is too old.

### State Estimation Task

| | |
|---|---|
| **Priority** | AboveNormal |
| **Rate** | Event-driven (ISR notifications) |
| **Stack** | 2,048 bytes |

Fuses sensor data through a dual Extended Kalman Filter:
- **Orientation EKF** — quaternion state, driven by accelerometer and gyroscope
- **Body EKF** — position and velocity, driven by barometer and GPS

On startup, performs a calibration phase (20 samples) to compute gyro/accel biases. Publishes fused state (position, velocity, attitude, angular rate) to the shared state exchange. Logs all sensor samples and fused state to the SD card.

Responds to ISR flags: `BMI088_ACCEL_SAMPLE_FLAG`, `BMI088_GYRO_SAMPLE_FLAG`, `MS5611_BARO_SAMPLE_FLAG`, `MS5607_BARO2_SAMPLE_FLAG`, `GNSS_GPS_FIX_READY_FLAG`.

### Mission Manager Task

| | |
|---|---|
| **Priority** | Normal |
| **Rate** | 100 ms timeout / radio event |
| **Stack** | 2,048 bytes |

Manages the flight state machine and radio communications:

- **Flight states:** `IDLE` → `RISE` → `HOVER` → `LOWER` (or `E_STOP` at any time)
- **Radio RX:** Decodes protobuf `FlightCommand` messages (ARM, LAUNCH, ABORT, LAND, PID updates)
- **Radio TX:** Sends telemetry at 10 Hz and system status at 1 Hz
- **SD Logging:** Logs flight state transitions and periodically flushes the log buffer

### Debug Logging Task (optional)

| | |
|---|---|
| **Priority** | High |
| **Stack** | 512 bytes |
| **Enabled by** | `ULYSSES_ENABLE_DEBUG_LOGGING` (ON by default) |

Drains the debug log buffer and outputs formatted messages over USART1. Disable via the CMake option for production builds.

### Sensor Acquisition

Sensor data acquisition is handled asynchronously via DMA — no dedicated polling task is needed. SPI transfers for the BMI088 IMU, MS5611/MS5607 barometers, and GNSS radio run in the background and fire interrupts when data is ready, which notify the State Estimation task.

## Prerequisites

> **Windows is not supported.** The STM32CubeMX code generation uses forward slashes in paths, which breaks native Windows builds. If you're on Windows, use [WSL2](https://learn.microsoft.com/en-us/windows/wsl/install) with the Linux instructions below.

### Required Tools

| Tool | Version | Purpose |
|---|---|---|
| **ARM GNU Toolchain** | Latest (`arm-none-eabi-gcc`) | Cross-compiler for Cortex-M33 |
| **CMake** | >= 3.22 | Build system generator |
| **Ninja** | Latest | Build system |
| **STM32CubeIDE** | Latest | Provides ST's OpenOCD (required for STM32H563 support) |
| **Git** | Latest | Version control + submodules |

### Optional Tools

| Tool | Purpose |
|---|---|
| **STM32CubeMX** | GUI pin/peripheral configuration (edits `ulysses.ioc`) |
| **VS Code** | Recommended editor with Cortex-Debug integration |

### Install on macOS (Homebrew)

```bash
brew install --cask gcc-arm-embedded   # ARM toolchain
brew install cmake ninja git
```

STM32CubeIDE must be downloaded manually from [st.com](https://www.st.com/en/development-tools/stm32cubeide.html) (requires a free myST account).

### Install on Linux / WSL2

```bash
sudo apt update
sudo apt install gcc-arm-none-eabi cmake ninja-build git
```

Download STM32CubeIDE from [st.com](https://www.st.com/en/development-tools/stm32cubeide.html).

### Verify Installation

```bash
arm-none-eabi-gcc --version
cmake --version
ninja --version
```

## Building

### Clone the Repository

```bash
git clone https://github.com/UBC-Rocket/ulysses.git
cd ulysses
git submodule update --init --recursive
```

### Configure and Build (command line)

Using CMake presets (recommended):

```bash
# Debug build
cmake --preset debug
cmake --build --preset debug

# Release build
cmake --preset release
cmake --build --preset release
```

Or manually:

```bash
cmake -B build/debug \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug \
  -G Ninja

cmake --build build/debug
```

The output binary is `build/debug/ulysses.elf`.

### Build Options

| Option | Default | Description |
|---|---|---|
| `ULYSSES_ENABLE_DEBUG_LOGGING` | `ON` | Enable serial debug output over USART1 |
| `ULYSSES_USE_USB` | `OFF` | Enable USB support |

Pass options during configuration:

```bash
cmake --preset debug -DULYSSES_ENABLE_DEBUG_LOGGING=OFF
```

### Build with VS Code

1. Install the recommended extensions (open Command Palette → `Extensions: Show Recommended Extensions`)
2. VS Code will auto-detect `CMakePresets.json` — select `debug` or `release` when prompted
3. Click **Build** in the status bar or press `F7`

## Flashing and Debugging

Flashing and debugging require an **ST-Link** debugger connected to the board and **ST's OpenOCD** (bundled with STM32CubeIDE). Upstream OpenOCD does not yet support the STM32H563.

### Step 1: Set Up the OpenOCD Environment Variable

You need to set `STM32_OPENOCD_SCRIPTS_PATH` to point to the ST OpenOCD scripts directory inside your STM32CubeIDE installation.

**macOS:**

```bash
# Add to your ~/.zshrc, ~/.bashrc, or ~/.config/fish/config.fish
export STM32_OPENOCD_SCRIPTS_PATH="/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.debug.openocd_2.3.200.202510310951/resources/openocd/st_scripts"
```

> The exact plugin version number (e.g. `2.3.200.202510310951`) will vary based on your STM32CubeIDE version. To find it:
> ```bash
> ls /Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/ | grep openocd
> ```

**Linux / WSL2:**

```bash
# Add to your ~/.bashrc or ~/.zshrc
export STM32_OPENOCD_SCRIPTS_PATH="/opt/st/stm32cubeide_1.19.0/plugins/com.st.stm32cube.ide.mcu.debug.openocd_2.3.100.202501240831/resources/openocd/st_scripts"
```

> Find the exact path with:
> ```bash
> find /opt/st -path "*/openocd/st_scripts" -type d
> ```

**Important:** Restart your shell and VS Code after setting the variable.

### Step 2: Configure the OpenOCD Binary Path in VS Code

Create or edit `.vscode/settings.json`:

```json
{
  "cortex-debug.openocdPath": "<OPENOCD_BINARY_PATH>"
}
```

Where `<OPENOCD_BINARY_PATH>` is:

| Platform | Typical Path |
|---|---|
| **macOS** | `/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.openocd.macos64_2.4.200.202505051030/tools/bin/openocd` |
| **Linux** | `/opt/st/stm32cubeide_1.19.0/plugins/com.st.stm32cube.ide.mcu.externaltools.openocd.linux64_2.4.200.202505051030/tools/bin/openocd` |

> Find the exact path with:
> ```bash
> # macOS
> find /Applications/STM32CubeIDE.app -name "openocd" -type f
> # Linux
> find /opt/st -name "openocd" -type f
> ```

### Step 3: Flash and Debug

**From VS Code (recommended):**

1. Connect your ST-Link to the board
2. Open the **Run and Debug** panel (`Ctrl+Shift+D` / `Cmd+Shift+D`)
3. Select **Ulysses - Debug** or **Ulysses - Release**
4. Press `F5` to flash and start debugging
5. The debugger will halt at `main()` — press `F5` again to continue

The launch configurations use:
- **Interface:** `interface/stlink-dap.cfg`
- **Target:** `target/stm32h5x.cfg`
- **SVD:** `platform/svd/STM32H563.svd` (enables peripheral register inspection)

**From the command line (flash only):**

```bash
# Find the openocd binary (same path as OPENOCD_BINARY_PATH above)
OPENOCD=/path/to/openocd

$OPENOCD \
  -s "$STM32_OPENOCD_SCRIPTS_PATH" \
  -f interface/stlink-dap.cfg \
  -f target/stm32h5x.cfg \
  -c "program build/debug/ulysses.elf verify reset exit"
```

## Project Structure

```
ulysses/
├── Core/
│   ├── Inc/                        # Headers
│   │   ├── controls/               #   Flight controller, PID
│   │   ├── state_estimation/       #   EKF, quaternions, matrices
│   │   ├── motor_drivers/          #   PWM, ESC, servo drivers
│   │   ├── spi_drivers/            #   BMI088, MS5611, MS5607, GNSS radio
│   │   ├── SD_logging/             #   SD card logging
│   │   └── debug/                  #   Debug log facilities
│   └── Src/                        # Source files (mirrors Inc/ layout)
│       └── tasks/                  #   FreeRTOS task implementations
├── Drivers/                        # STM32 HAL & CMSIS (auto-generated)
├── Middlewares/                     # FreeRTOS kernel
├── rocket-protocol-lib/            # Protobuf radio protocol (submodule)
│   ├── generated/                  #   Nanopb-generated C code
│   ├── nanopb/                     #   Nanopb runtime library
│   └── src/                        #   Codec, COBS framing, CRC16
├── cmake/
│   ├── gcc-arm-none-eabi.cmake     # ARM Cortex-M33 toolchain file
│   ├── dependencies.cmake          # External deps (printf library)
│   └── stm32cubemx/                # CubeMX-generated CMake sources
├── docs/                           # Additional documentation
├── platform/svd/                   # SVD file for register debugging
├── tools/                          # SD card and serial utilities
├── CMakeLists.txt                  # Main build configuration
├── CMakePresets.json               # Build presets (debug/release)
├── ulysses.ioc                     # STM32CubeMX project configuration
├── STM32H563xx_FLASH.ld            # Linker script
└── startup_stm32h563xx.s           # MCU startup assembly
```

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) and the [Development Setup Guide](docs/development_setup.md) for detailed toolchain installation instructions.
