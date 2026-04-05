# Firmware

All embedded firmware and shared firmware libraries for the TVR project.

## Structure

| Directory | Description |
|-----------|-------------|
| `ulysses-flight-controller/` | STM32H563 main flight controller (FreeRTOS, CubeMX) |
| `ulysses-gnss-radio/` | STM32G0xx GNSS receiver and radio transceiver |
| `libs/state_estimation/` | EKF, quaternion/vector math, body state estimation |
| `libs/controls/` | PID controller and flight controller (torque, allocation, gimbal) |
| `libs/lwgps/` | Lightweight GPS NMEA parser (vendored from [lwgps](https://github.com/MaJerle/lwgps)) |
| `libs/unity/` | Unity C test framework (vendored, for host-side testing) |
| `docs/` | Firmware-specific documentation (SPI, SD logging, drivers) |
| `tools/` | Host-side Python tools (SD log decoding, serial debug) |

## Building

Each firmware target uses CMake presets for cross-compilation:

```bash
# Flight controller
cd ulysses-flight-controller
cmake --preset debug && cmake --build --preset debug

# GNSS radio
cd ulysses-gnss-radio
cmake --preset Debug && cmake --build --preset Debug

# Gimbal test stand
cd gimbal_test_stand
cmake --preset debug && cmake --build --preset debug
```

## Testing

The shared libraries (`state_estimation`, `controls`) have host-side unit tests using the Unity framework:

```bash
cd libs/state_estimation/tests
cmake -B build && cmake --build build && ctest --test-dir build

cd libs/controls/tests
cmake -B build && cmake --build build && ctest --test-dir build
```

## Debugging

Open `TVR.code-workspace` in VS Code. Each firmware project has its own `.vscode/launch.json` with Cortex-Debug configurations for ST-Link/OpenOCD.
