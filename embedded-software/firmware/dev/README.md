# DEV layer

Per-device drivers. Each driver is a self-contained opaque singleton that
owns its EXTI handler, init sequence, sample ring, and IO bindings.

| Driver dir | Lib target | Header | Device | IO bindings |
|---|---|---|---|---|
| `imu_bmi088/`    | `dev_imu_bmi088`    | `dev_imu_bmi088.h`    | Bosch BMI088 IMU                | SPI4 + BMI_ACC_CS / BMI_GYRO_CS + INT1 lines |
| `imu_icm40609/`  | `dev_imu_icm40609`  | `dev_imu_icm40609.h`  | TDK InvenSense ICM-40609-D      | SPI3 (HW NSS) + ICM_INT1 |
| `baro_ms5611/`   | `dev_baro_ms5611`   | `dev_baro_ms5611.h`   | TE MS5611 barometer             | SPI2 (HW NSS) |
| `gps_nmea/`      | `dev_gps_nmea`      | `dev_gps_nmea.h`      | u-blox SAM-M10Q GPS (NMEA)      | UART5, `'\n'` framing |
| `radio_rfd900/`  | `dev_radio_rfd900`  | `dev_radio_rfd900.h`  | RFDesign RFD900x telemetry      | UART7, `0x00` framing |
| `servo_feetech/` | `dev_servo_feetech` | `dev_servo_feetech.h` | Feetech STS/SCS bus servos      | UART8, half-duplex |
| `esc_dshot/`     | `dev_esc_dshot`     | `dev_esc_dshot.h`     | DShot ESC pair                  | DShot upper/lower channels |

## API pattern

Every driver is an opaque singleton with the same five-call shape:

```c
#include "dev_imu_bmi088.h"

bool            imu_bmi088_init(void);                       /* IO bindings baked in */
imu_bmi088_t   *imu_bmi088_get (void);                       /* NULL before init */
size_t          imu_bmi088_drain(imu_bmi088_t *d, imu_sample_t *out, size_t max);
void            imu_bmi088_set_notify(imu_bmi088_t *d, io_task_handle_t t, uint32_t bit);
```

The struct is forward-declared in the public header; its definition lives
in the .c. Each driver references its IO transport symbols by name
(`IO_SPI_BMI088_ACC`, `IO_EXTI_BMI088_ACC_INT1`, …) which are declared in
`io/io_*.h` and defined per-target in `io/h747/CM{4,7}/`.

## Test hooks

Each driver's `s_self` singleton (and key scratch buffers, where applicable)
is exposed via `IO_TEST_HOOK_RW` / `IO_TEST_HOOK_ARRAY` macros from
[`io/io_sys/io_test_hooks.h`](../io/io_sys/include/io_sys/io_test_hooks.h).
The macros compile to nothing in production; tests link with
`-DIO_TEST_HOOKS` to access the underlying state.
