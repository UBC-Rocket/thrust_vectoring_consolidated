# TVR - UBC Rocket

Mono-repo for the TVR project, containing firmware, ground station, shared libraries, and tools.

## Repository Structure

| Directory | Description |
|-----------|-------------|
| `firmware/ulysses-flight-controller/` | STM32H563 flight controller firmware (FreeRTOS, CMake) |
| `firmware/ulysses-gnss-radio/` | STM32G0xx GNSS/radio module firmware |
| `firmware/libs/state_estimation/` | EKF, quaternion math, and body state estimation library |
| `firmware/libs/controls/` | PID controller and flight controller library |
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

Open `TVR.code-workspace` in VS Code for a multi-root workspace with Cortex-Debug support for each firmware target.

### Prerequisites

- ARM GCC toolchain (`arm-none-eabi-gcc`) for firmware
- Qt 6.6+ for ground station
- Python 3.10+ with [uv](https://docs.astral.sh/uv/) for tools
- CMake 3.22+

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

## CI/CD

GitHub Actions workflows run on push/PR:

- **Firmware Flight Controller** - cross-compile on `firmware/ulysses-flight-controller/**` or `firmware/libs/**` changes
- **Firmware GNSS Radio** - cross-compile on `firmware/ulysses-gnss-radio/**` or `firmware/libs/lwgps/**` changes
- **Firmware Library Tests** - host tests for `state_estimation` and `controls`
- **Protocol Library Tests** - host tests for `rocket-protocol`
- **Ground Station** - Qt6 build on `ground-station/**` changes

## Releases

Tags follow the format `<component>/v<semver>`:
- `firmware-fc/v1.0.0` - Flight controller firmware
- `firmware-gnss/v1.0.0` - GNSS radio firmware
- `ground-station/v1.0.0` - Ground control station
- `tvr/v1.0.0` - Coordinated release of all components
