# Firmware Libraries

Shared C libraries used by firmware targets. These are platform-independent and can be tested on host.

| Library | Description |
|---------|-------------|
| `state_estimation/` | Extended Kalman Filter (quaternion orientation + body position/velocity), matrix operations, quaternion/vector math |
| `controls/` | PID controller, flight controller (attitude torque, control allocation, thrust PID, gimbal angles) |
| `lwgps/` | Lightweight GPS NMEA parser (vendored third-party library) |
| `unity/` | Unity C test framework (vendored, v2.6.1) |

## Testing

```bash
# State estimation (22 tests)
cd state_estimation/tests && cmake -B build && cmake --build build && ctest --test-dir build

# Controls (10 tests)
cd controls/tests && cmake -B build && cmake --build build && ctest --test-dir build
```

## Usage from firmware

Each library is consumed via `add_subdirectory()` in the firmware CMakeLists:

```cmake
set(FW_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libs)
add_subdirectory(${FW_LIBS_DIR}/state_estimation ${CMAKE_BINARY_DIR}/state_estimation)
add_subdirectory(${FW_LIBS_DIR}/controls ${CMAKE_BINARY_DIR}/controls)

target_link_libraries(${CMAKE_PROJECT_NAME} state_estimation controls)
```
