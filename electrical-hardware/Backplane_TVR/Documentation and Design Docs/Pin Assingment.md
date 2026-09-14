# Board to Board Connector Pin Assignment

## List of Interfaces / PWR needed between boards: (Pin Budget)

| # | Signal Group          | Count | Total Pins |
|---|-----------------------|-------|------|
| 1 | +3V3                  | 2×    | 2    |
| 2 | +5V                   | 2×    | 2    |
| 3 | PWM                   | 2×    | 2    |
| 4 | Gate Driver SPI       | 2×    | 8    |
| 5 | ESC MCU UART          | 1×    | 2    |
| 6 | Gate Driver EN        | 2×    | 2    |
| 7 | ESC Temp Sensor       | 2×    | 4    |
| 8 | Cell Voltages         | 7x     | 7    |
| 9 | Fuel Gauge I2C        | 1×    | 2    |
| 10| Cell Monitoring I2C   | 1×    | 2    |
| 11| Camera I2C            | 1×    | 2    |
| 12| Alternating GND       | 15x     | 15   |
|   | **TOTAL**             |       | **50** |

## Choosen Connector:


## Pin Assignment:

| Net          | Pin |                | Pin | Net          |
|-------------:|:---:|:--------------:|:---:|:-------------|
|              | 1   | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 2   | +3V3              |
|              | 3   | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 4   | GND             |
|              | 5   | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 6   | +5V             |
|              | 7   | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 8   | GND             |
|              | 9   | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 10  |              |
|              | 11  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 12  |              |
|              | 13  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 14  |              |
|              | 15  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 16  |              |
|              | 17  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 18  |              |
|              | 19  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 20  |              |
|              | 21  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 22  |              |
|              | 23  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 24  |              |
|              | 25  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 26  | +3V3             |
|              | 27  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 28  | GND             |
|              | 29  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 30  | +5v             |
|              | 31  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 32  | GND             |
|              | 33  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 34  |              |
|              | 35  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 36  |              |
|              | 37  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 38  |              |
|              | 39  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 40  |              |
|              | 41  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 42  |              |
|              | 43  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 44  |              |
|              | 45  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 46  |              |
|              | 47  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 48  |              |
|              | 49  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 50  |              |
|              | 51  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 52  |              |
|              | 53  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 54  | +3V3             |
|              | 55  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 56  | GND             |
|              | 57  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 58  | +5V             |
|              | 59  | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 60  | GND             |


