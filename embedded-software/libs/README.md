# Shared Libraries

Libraries shared across multiple components (firmware, ground station, Jetson).

| Library | Description |
|---------|-------------|
| `rocket-protocol/` | Protobuf-based communication protocol using nanopb, COBS framing, and CRC16 |

## rocket-protocol

Used by both the flight controller firmware and the ground station for telemetry and command encoding/decoding.

```cmake
set(TVR_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../libs)
add_subdirectory(${TVR_LIBS_DIR}/rocket-protocol ${CMAKE_BINARY_DIR}/rocket-protocol)

target_link_libraries(${CMAKE_PROJECT_NAME} rocket-protocol)
```
