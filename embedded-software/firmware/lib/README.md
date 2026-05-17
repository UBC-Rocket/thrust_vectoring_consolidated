# lib/ — utility libraries

Pure-C, hardware-agnostic libraries. No layer affinity — used by APP, DEV,
and/or other libs.

| Lib | Description |
|---|---|
| `state_estimation/` | EKF (quaternion orientation + position/velocity), matrix + quaternion math |
| `controls/`         | PID, flight controller (attitude torque, allocation, thrust PID, gimbal) |
| `log_records/`      | SD log record schema, frame encoding, CRC-16 CCITT |
| `sensors/`          | Bus-agnostic sensor protocol helpers (BMI088, MS5611) |
| `lwgps/`            | Lightweight NMEA parser (vendored upstream) |
| `gnss-protocol/`    | Shared `gps_fix_t` for inter-board GPS protocol |
| `collections/`      | Generic SPSC ring (`spsc_ring.h`) |
| `timestamp/`        | DWT cycle-counter µs helper (Cortex-M) |
| `unity/`            | Unity test framework (vendored) |

## Testing

```sh
cd lib/controls/tests        && cmake -B build && cmake --build build && ./build/test_controls
cd lib/state_estimation/tests && cmake -B build && cmake --build build && ./build/test_state_estimation
```
