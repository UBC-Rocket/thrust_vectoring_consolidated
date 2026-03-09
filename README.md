# TVR - UBC Rocket

[![Flight Controller](https://github.com/UBC-Rocket/thrust_vectoring_consolidated/actions/workflows/firmware-flight-controller.yml/badge.svg)](https://github.com/UBC-Rocket/thrust_vectoring_consolidated/actions/workflows/firmware-flight-controller.yml)
[![GNSS Radio](https://github.com/UBC-Rocket/thrust_vectoring_consolidated/actions/workflows/firmware-gnss-radio.yml/badge.svg)](https://github.com/UBC-Rocket/thrust_vectoring_consolidated/actions/workflows/firmware-gnss-radio.yml)
[![Firmware Libraries Tests](https://github.com/UBC-Rocket/thrust_vectoring_consolidated/actions/workflows/firmware-libs-tests.yml/badge.svg)](https://github.com/UBC-Rocket/thrust_vectoring_consolidated/actions/workflows/firmware-libs-tests.yml)
[![Protocol Library Tests](https://github.com/UBC-Rocket/thrust_vectoring_consolidated/actions/workflows/protocol-lib-tests.yml/badge.svg)](https://github.com/UBC-Rocket/thrust_vectoring_consolidated/actions/workflows/protocol-lib-tests.yml)
[![Ground Station](https://github.com/UBC-Rocket/thrust_vectoring_consolidated/actions/workflows/ground-station.yml/badge.svg)](https://github.com/UBC-Rocket/thrust_vectoring_consolidated/actions/workflows/ground-station.yml)

Mono-repo for the TVR project, containing firmware, ground station, shared libraries, and tools.

## Repository Structure

| Directory | Description |
|-----------|-------------|
| `firmware/ulysses-flight-controller/` | STM32H563 flight controller firmware (FreeRTOS, CMake) |
| `firmware/ulysses-gnss-radio/` | STM32G0xx GNSS/radio module firmware |
| `firmware/libs/state_estimation/` | EKF, quaternion math, and body state estimation library |
| `firmware/libs/controls/` | PID controller and flight controller library |
| `firmware/libs/sensors/` | Bus-agnostic sensor protocol drivers |
| `firmware/libs/lwgps/` | Lightweight GPS NMEA parser (vendored) |
| `firmware/libs/unity/` | Unity C test framework (vendored) |
| `firmware/docs/` | Firmware documentation |
| `firmware/tools/` | SD log decoding and serial debug tools |
| `ground-station/` | Qt6/QML ground control station |
| `libs/rocket-protocol/` | Shared protobuf protocol library (nanopb, COBS, CRC) |
| `controls/prototyping/` | MATLAB/Simulink control system prototyping |
| `jetson/` | Future Jetson compute module (placeholder) |
| `electrical-hardware/` | Electrical hardware design files |
| `mechanical-hardware/` | Mechanical hardware design files |

## Getting Started

For firmware development, open the board folder directly in VS Code:

- `firmware/ulysses-flight-controller/`
- `firmware/ulysses-gnss-radio/`

Each folder is self-contained with CMake presets, debug launch configs, and recommended extensions. See [firmware/docs/development_setup.md](firmware/docs/development_setup.md) for full setup instructions.

### Prerequisites

- ARM GCC toolchain (`arm-none-eabi-gcc`) for firmware
- STM32CubeIDE (for ST's OpenOCD — see [development setup](firmware/docs/development_setup.md))
- CMake 3.22+ and Ninja
- Qt 6.6+ for ground station
- Python 3.10+ with [uv](https://docs.astral.sh/uv/) for tools

### Environment Variables

Debugging requires two environment variables set in your shell profile (`~/.bashrc`, `~/.zshrc`, or `~/.config/fish/config.fish`). Both paths come from your STM32CubeIDE installation — the plugin version in the path varies by release.

| Variable | Purpose |
|---|---|
| `STM32_OPENOCD_PATH` | Path to the OpenOCD binary bundled with STM32CubeIDE |
| `STM32_OPENOCD_SCRIPTS_PATH` | Path to the OpenOCD st_scripts directory |

<details>
<summary>macOS</summary>

```bash
export STM32_OPENOCD_PATH="/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.openocd.macos64_<version>/tools/bin/openocd"
export STM32_OPENOCD_SCRIPTS_PATH="/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.debug.openocd_<version>/resources/openocd/st_scripts"
```

</details>

<details>
<summary>Windows</summary>

```bash
export STM32_OPENOCD_PATH="D:/ST/STM32CubeIDE_<version>/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.openocd.win32_<version>/tools/bin/openocd.exe"
export STM32_OPENOCD_SCRIPTS_PATH="D:/ST/STM32CubeIDE_<version>/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.debug.openocd_<version>/resources/openocd/st_scripts"
```

</details>

<details>
<summary>Linux</summary>

```bash
export STM32_OPENOCD_PATH="/opt/st/stm32cubeide_<version>/plugins/com.st.stm32cube.ide.mcu.externaltools.openocd.linux64_<version>/tools/bin/openocd"
export STM32_OPENOCD_SCRIPTS_PATH="/opt/st/stm32cubeide_<version>/plugins/com.st.stm32cube.ide.mcu.debug.openocd_<version>/resources/openocd/st_scripts"
```

</details>

> [!IMPORTANT]
> Restart VS Code (and your shell) after setting these variables.

### Building Firmware

```bash
cd firmware/ulysses-flight-controller
cmake --preset debug
cmake --build --preset debug
```

### Running Tests

```bash
# State estimation tests
cd firmware/libs/state_estimation/tests
cmake -B build && cmake --build build
ctest --test-dir build

# Controls tests
cd firmware/libs/controls/tests
cmake -B build && cmake --build build
ctest --test-dir build
```

### Building Ground Station

```bash
cd ground-station
cmake -B build
cmake --build build
```

## Releases

Tags follow the format `<component>/v<semver>`:
- `firmware-fc/v1.0.0` — Flight controller firmware
- `firmware-gnss/v1.0.0` — GNSS radio firmware
- `ground-station/v1.0.0` — Ground control station
- `tvr/v1.0.0` — Coordinated release of all components

## Contributors

<a href="https://github.com/UBC-Rocket/thrust_vectoring_consolidated/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=UBC-Rocket/thrust_vectoring_consolidated" />
</a>
