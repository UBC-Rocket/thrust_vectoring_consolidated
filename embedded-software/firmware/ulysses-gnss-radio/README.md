# Ulysses GNSS-Radio Slave Board

Dedicated GNSS receiver and radio transceiver slave board for the UBC Rocket Ulysses avionics system.

## Overview

The GNSS-Radio slave board interfaces with the Ulysses flight controller via SPI, providing:
- **GPS/GNSS positioning** with NMEA sentence u-blox SAM-M10Q
- **Radio communication** via RFD900x Radio Modem 
- **Dual-mode SPI protocol** (pull and push) for flexible data exchange
- **Debug injection system** for testing without physical peripherals (Debug builds only)

This board offloads GPS and radio communication from the main flight controller, reducing its processing burden and enabling deterministic real-time performance.

## Hardware

- **MCU**: STM32G0B1CEU6 (Cortex-M0+, 128KB FLASH, 144KB RAM)
- **Clock**: 16 MHz HSI (no PLL, low power configuration)
- **Peripherals**:
  - SPI1 (slave mode) - Communication with flight controller
  - USART5 (57600 baud) - Radio transceiver interface
  - USART6 (115200 baud) - GPS module interface
  - USART1 (115200 baud) - Debug console (ST-Link)
  - DMA1 (6 channels) - High-throughput peripheral data transfer
  - GPIO PB2 - IRQ output for push mode signaling

## Features

### SPI Slave Protocol
- **Pull Mode**: Master-initiated transactions with command-based protocol
  - Commands: 0x01-0x05 (radio read/write, GPS read, buffer status)
  - Transaction format: `[CMD:1][DUMMY:4][PAYLOAD:0-256]`
  - Total sizes: 6-261 bytes depending on command

- **Push Mode**: Slave-initiated data delivery via IRQ assertion
  - Slave asserts IRQ (PB2, active high) when data available
  - Transaction format: `[TYPE:1][PAYLOAD:N]`
  - Types: 0x01 (Radio), 0x05 (GPS)

- **Startup Configuration**: Dynamic mode selection on power-up
  - Master sends 8-byte configuration frame
  - Byte 0: Mode selection (0x00=Pull, 0x01=Push)
  - Bytes 1-7: GPS configuration (update rate, constellation, reserved)

### GPS Integration
- **Reception**: DMA circular + Character Match on '\n' via USART6
- **Parsing**: lwgps library (NMEA 0183 standard)
- **Supported sentences**: GPGGA, GPRMC, GPGSA, GPGSV (+ GLONASS variants)
- **Dual queue system**:
  - Raw NMEA queue: 10 samples × 87 bytes (pull mode)
  - Parsed fix queue: 10 fixes × 48 bytes (push mode)
- **Output formats**:
  - Pull mode: Raw NMEA sentences (up to 87 bytes)
  - Push mode: Parsed `gps_fix_t` struct (48 bytes, packed binary)

### Radio Integration
- **Interface**: UART5 at 57600 baud
- **Message size**: 256 bytes per message
- **Queue**: 10-message circular buffer
- **Read modes**:
  - FIFO (0x02): Oldest message first
  - LIFO (0x01): Newest message first (stack behavior)

### Debug Features (DEBUG builds only)
USART1 serves dual purpose when compiled with DEBUG macro:

**RX (Message Injection)**:
- Binary protocol for test message injection
- `0x52 ('R') + payload` → Inject radio message
- `0x47 ('G') + payload` → Inject GPS NMEA sentence
- Fully emulates real peripheral behavior via DMA + CM

**TX (System Logging)**:
- Human-readable event logging
- Log formats:
  - `[RADIO RX] <hex> (<ASCII>)` - Radio message received from UART5
  - `[GPS NMEA] <sentence>` - GPS NMEA sentence received from UART6
  - `[GPS FIX] Lat: X, Lon: Y, Alt: Z, ...` - Decoded GPS fix
  - `[SPI TX] Radio msg from master: <hex>` - Radio TX from SPI master
  - `[SPI->M] Radio: <hex> (<ASCII>)` - Radio message sent to SPI master
  - `[SPI->M] GPS: <data>` - GPS data sent to SPI master
  - `[SPI->M] BufLen: <count>` - Buffer length response sent to SPI master
- Zero code size impact in Release builds

## Build System

### Prerequisites
- CMake 3.16+
- Ninja build system
- arm-none-eabi-gcc toolchain
- Git (for lwgps submodule)

### Building

```bash
# Initialize lwgps submodule (first time only)
git submodule update --init --recursive

# Debug build (with injection and logging)
cmake --preset Debug
ninja -C build/Debug

# Release build (production, no debug features)
cmake --preset Release
ninja -C build/Release
```

### Build Configuration
- **Debug preset**: Defines `DEBUG` macro, enables debug UART features
- **Release preset**: Stripped binary, no debug overhead
- **C standard**: C11 with GNU extensions
- **Optimization**: `-Og` (Debug), `-O2` (Release)

## Project Structure

```
ulysses-gnss-radio/
├── Core/
│   ├── Inc/                        # Header files
│   │   ├── main.h
│   │   ├── protocol_config.h       # SPI protocol definitions & structs
│   │   ├── spi_slave.h             # SPI slave driver API
│   │   ├── gps.h                   # GPS driver API
│   │   ├── gps_nema_queue.h        # Raw NMEA queue
│   │   ├── gps_fix_queue.h         # Parsed fix queue
│   │   ├── radio_driver.h          # Radio transceiver API
│   │   ├── radio_queue.h           # Radio message queue
│   │   ├── debug_uart.h            # Debug injection/logging (DEBUG only)
│   │   └── uart_callbacks.h        # UART interrupt routing
│   │
│   └── Src/                        # Implementation files
│       ├── main.c                  # Initialization & main loop
│       ├── spi_slave.c             # SPI slave driver (register-level)
│       ├── gps.c                   # GPS NMEA receiver & parser
│       ├── radio_driver.c          # Radio transceiver interface
│       ├── debug_uart.c            # Debug injection/logging
│       ├── uart_callbacks.c        # UART interrupt routing
│       └── stm32g0xx_hal_msp.c     # HAL MSP init (CubeMX generated)
│
├── lib/lwgps/                      # lwgps NMEA parser (git submodule)
├── Drivers/                        # STM32 HAL drivers (CubeMX generated)
├── CMakeLists.txt                  # Build configuration
├── CMakePresets.json               # Debug/Release presets
└── ulysses-gnss-radio.ioc         # STM32CubeMX project file
```

## SPI Slave Implementation Deep Dive

This section explains the low-level implementation details of the SPI slave driver, which uses register-level programming for maximum performance and determinism.

### Architecture Overview

The SPI slave uses a **hybrid RXNE interrupt + DMA approach**:
- **First byte (command)**: Captured by RXNE interrupt for fast response
- **Remaining bytes**: Handled by DMA for zero CPU overhead
- **Transaction completion**: Detected via NSS rising edge (EXTI interrupt)

This architecture provides a 4-byte time window to decode the command and prepare the response before the master clocks out the data region.

### Peripheral Initialization Note

**IMPORTANT**: The SPI peripheral initialization uses the STM32 HAL layer (`stm32g0xx_hal_msp.c`) to configure GPIO pins, enable clocks, and reserve the SPI1 peripheral. This HAL-based initialization ensures proper peripheral reservation and avoids conflicts with other code that might try to use SPI1.

After HAL initialization completes, the SPI slave driver ([spi_slave.c](Core/Src/spi_slave.c)) takes over and uses **register-level programming** for all runtime operations. The HAL is only used during startup to:
- Configure SPI1 GPIO pins (PA4-PA7) in alternate function mode
- Enable peripheral clocks (SPI1, DMA1, DMAMUX, GPIOA, GPIOB)
- Set up NVIC interrupt priorities

This hybrid approach (HAL for init, registers for runtime) provides the best of both worlds: HAL's initialization convenience and register-level performance for time-critical operations.

### Why 4 Dummy Bytes?

The implementation uses **4 dummy bytes** (not 2) to avoid a TX FIFO prefetch race condition:

1. STM32G0 SPI peripheral has a 4-byte TX FIFO
2. When master starts clocking, slave TX FIFO immediately pulls 4 bytes from DMA
3. **Critical timing constraint**: We must patch the TX buffer with response data BEFORE those 4 bytes are consumed
4. By placing response data starting at byte 5 (after CMD + 4 DUMMY), we guarantee the RXNE ISR has time to prepare the response

**Timing breakdown** (at 1 MHz SPI clock):
- Command byte arrives: RXNE ISR fires (~1 µs)
- Decode command + patch TX buffer: ~10-20 µs (worst case)
- Dummy bytes consumed: 4 bytes × 8 µs/byte = 32 µs
- **Result**: 32 µs window is sufficient for even worst-case command handling

### Pull Mode Implementation

Pull mode uses **master-initiated transactions** with command-based protocol.

#### State Machine (Pull Mode)
```
UNCONFIGURED ──[config frame]──> IDLE ──[NSS↓]──> ACTIVE ──[NSS↑]──> IDLE
                                    ↑                                    │
                                    └────────────────────────────────────┘
```

#### Transaction Flow (Pull Mode)

**Step 1: Idle State (Waiting)**
- `SPI_STATE_IDLE`
- TX DMA armed with 261-byte buffer (zeros initially)
- RX DMA configured but **RXDMAEN disabled** (critical!)
- **RXNEIE enabled** - waiting for first byte interrupt

**Step 2: Command Byte Arrival**
- NSS falls (master starts transaction)
- First byte clocked in → SPI RXNE flag set
- `SPI1_IRQHandler()` fires (highest priority interrupt)
- **Critical section** (~10-20 µs):
  ```c
  cmd = SPI1->DR;  // Read command byte (clears RXNE)
  ctx.current_cmd = cmd;

  // Patch TX buffer based on command
  switch (cmd) {
    case CMD_RADIO_RX_FIFO:
      memcpy(&tx_buf[5], radio_msg, 256);  // Offset 5 = skip cmd+dummy
      break;
    // ... other commands
  }

  // NOW enable RX DMA (handoff to DMA for remaining bytes)
  SPI1->CR2 |= SPI_CR2_RXDMAEN;
  ```
- **Disable RXNEIE** - no more byte interrupts
- **Enable RXDMAEN** - DMA takes over for bytes 1-260

**Step 3: DMA Takes Over**
- State transitions to `SPI_STATE_ACTIVE`
- RX DMA silently captures bytes 1-260 into `rx_buf[1..260]`
- TX DMA silently sends bytes 0-260 from `tx_buf[0..260]`
- **Zero CPU overhead** from this point on

**Step 4: Transaction Complete**
- NSS rises (master done)
- EXTI4_15 interrupt fires (NSS rising edge on PA4)
- Process write commands (e.g., CMD_RADIO_TX extracts payload from rx_buf)
- Re-arm for next transaction: `spi_slave_arm()`

#### Pull Mode Register Configuration
```c
// Initial setup (waiting for command)
SPI1->CR2 = (7 << SPI_CR2_DS_Pos)    // 8-bit frames
          | SPI_CR2_FRXTH            // 1-byte RX threshold (trigger RXNE every byte)
          | SPI_CR2_RXNEIE           // RXNE interrupt enabled
          | SPI_CR2_TXDMAEN;         // TX DMA enabled (RX DMA OFF initially!)

// After command captured (in RXNE ISR)
SPI1->CR2 |= SPI_CR2_RXDMAEN;        // Enable RX DMA (handoff from interrupt)
```

### Push Mode Implementation

Push mode uses **slave-initiated transactions** via IRQ assertion.

#### State Machine (Push Mode)
```
IDLE ──[data available]──> HAVE_DATA ──[NSS↓]──> ACTIVE ──[NSS↑]──> IDLE
                              (IRQ↑)                          (IRQ↓)
```

#### Transaction Flow (Push Mode)

**Step 1: Data Becomes Available**
- GPS fix parsed or radio message received
- `spi_slave_tick()` called from main loop
- Detects pending data in queue
- Calls `spi_slave_prepare_push(PUSH_TYPE_GPS)`:
  ```c
  tx_buf[0] = PUSH_TYPE_GPS;  // Type byte
  memcpy(&tx_buf[1], &gps_fix, 48);  // GPS fix payload
  ctx.tx_dma_length = 49;  // Total bytes to send
  ```
- Arms **both TX and RX DMA** simultaneously
- Asserts **IRQ line (PB2 high)**
- State transitions to `SPI_STATE_HAVE_DATA`

**Step 2: Master Responds to IRQ**
- Master detects IRQ rising edge
- Master asserts NSS (starts SPI transaction)
- **Both DMAs active from start** (different from pull mode!)
  - TX DMA sends prepared buffer (type + payload)
  - RX DMA captures what master sends (might be zeros, might be radio TX)

**Step 3: Transaction Complete**
- NSS rises
- `spi_slave_nss_exti_handler()` fires
- **Deassert IRQ (PB2 low)**
- Check RX buffer for valid data:
  ```c
  if (rx_received >= 256 && spi_slave_rx_has_valid_data()) {
    // Master sent radio TX simultaneously! Enqueue it.
    radio_message_enqueue(256, rx_buf, radio_queue);
  }
  ```
- Pop transmitted message from queue
- Re-arm for next transaction

#### Push Mode Register Configuration
```c
// Armed for push transaction (both DMAs active)
SPI1->CR2 = (7 << SPI_CR2_DS_Pos)    // 8-bit frames
          | SPI_CR2_FRXTH            // 1-byte RX threshold
          | SPI_CR2_TXDMAEN          // TX DMA enabled
          | SPI_CR2_RXDMAEN;         // RX DMA enabled (both on from start!)
// Note: RXNEIE is OFF in push mode (no command byte to capture)
```

### Key Differences: Pull vs Push

| Aspect | Pull Mode | Push Mode |
|--------|-----------|-----------|
| **Initiator** | Master (via command) | Slave (via IRQ) |
| **RXNEIE** | Enabled (capture command) | Disabled (no command byte) |
| **RXDMAEN** | Enabled AFTER command | Enabled from start |
| **TX Buffer** | Patched in RXNE ISR | Pre-loaded before IRQ assert |
| **IRQ Line** | Not used | Asserted when data ready |
| **Transaction Length** | Variable (6-261 bytes) | Fixed per type (49 or 257 bytes) |

### DMA Configuration Details

The implementation uses **DMA1 with DMAMUX routing**:

**RX DMA (Channel 1)**:
```c
DMA1_Channel1->CCR = DMA_CCR_MINC         // Increment memory address
                   | DMA_CCR_TCIE         // Transfer complete interrupt
                   | DMA_CCR_TEIE         // Transfer error interrupt
                   | (2 << DMA_CCR_PL_Pos); // High priority (RX critical!)
```

**TX DMA (Channel 2)**:
```c
DMA1_Channel2->CCR = DMA_CCR_DIR          // Memory → Peripheral
                   | DMA_CCR_MINC         // Increment memory address
                   | (1 << DMA_CCR_PL_Pos); // Medium priority
```

**DMAMUX Routing**:
```c
DMAMUX1_Channel0->CCR = 16;  // SPI1_RX request (RM0444 Table 59)
DMAMUX1_Channel1->CCR = 17;  // SPI1_TX request
```

### NSS Edge Detection (EXTI)

Transaction completion is detected via **NSS rising edge** on PA4:

```c
// Route PA4 to EXTI line 4
EXTI->EXTICR[1] = (0x00 << 0);  // Port A = 0x00

// Enable rising edge detection
EXTI->RTSR1 |= (1 << 4);  // Rising edge on line 4

// Unmask interrupt
EXTI->IMR1 |= (1 << 4);
```

**Why EXTI instead of SPI NSS interrupt?**
- STM32G0 SPI peripheral doesn't have a dedicated NSS interrupt
- EXTI provides precise timing for NSS rising edge detection
- Allows us to know exactly when master releases the bus

### Collision Handling (Push Mode)

**Scenario**: Slave asserts IRQ, but master was already starting a radio TX transaction.

**Detection**: Check RX buffer pattern after push transaction:
```c
bool spi_slave_rx_has_valid_data(void) {
  uint16_t zeros = 0, ones = 0;
  for (int i = 0; i < 16; i++) {  // Sample first 16 bytes
    if (rx_buf[i] == 0x00) zeros++;
    if (rx_buf[i] == 0xFF) ones++;
  }
  // If >14 bytes are dummy (0x00 or 0xFF), it's not real data
  return (zeros + ones) < 14;
}
```

**Resolution**: If valid data detected, enqueue it as a radio TX message.

### Interrupt Priorities

Critical for correct operation:

| Interrupt | Priority | Rationale |
|-----------|----------|-----------|
| SPI1_IRQn (RXNE) | 0 (highest) | Must capture command within 4-byte window |
| DMA1_Channel1_IRQn | 1 (high) | Process RX data quickly |
| EXTI4_15_IRQn (NSS) | 2 (normal) | Transaction cleanup can wait |

### Buffer Alignment

Both TX and RX buffers are **32-byte aligned** for optimal DMA performance:
```c
uint8_t tx_buf[MAX_TRANSACTION_SIZE] __attribute__((aligned(32)));
uint8_t rx_buf[MAX_TRANSACTION_SIZE] __attribute__((aligned(32)));
```

**Why 32 bytes?** STM32G0 DMA works best with cache-line aligned buffers (even though G0 has no D-cache, alignment helps bus arbitration).

## SPI Protocol Reference

### Configuration Frame (Startup)
```
Master → Slave (on power-up):
  [MODE:1][GPS_RATE:1][GPS_CONSTELLATION:1][RESERVED:5]

MODE: 0x00 = Pull mode, 0x01 = Push mode
GPS_RATE: Update rate in Hz (e.g., 1, 5, 10)
GPS_CONSTELLATION: Bitmask for GNSS constellations
```

### Pull Mode Commands
| Code | Command | Direction | Payload | Total Size | Description |
|------|---------|-----------|---------|------------|-------------|
| 0x01 | RADIO_RX_LIFO | Slave→Master | 256 B | 261 B | Newest radio message |
| 0x02 | RADIO_RX_FIFO | Slave→Master | 256 B | 261 B | Oldest radio message |
| 0x03 | RADIO_RXBUF_LEN | Slave→Master | 1 B | 6 B | Message count (0-255) |
| 0x04 | RADIO_TX | Master→Slave | 256 B | 261 B | Transmit radio message |
| 0x05 | GPS_RX | Slave→Master | 87 B | 92 B | Raw NMEA sentence |

### Push Mode Data Types
| Type | Data Type | Payload | Total Size | Description |
|------|-----------|---------|------------|-------------|
| 0x01 | RADIO | 256 B | 257 B | Radio message ready |
| 0x05 | GPS | 48 B | 49 B | Parsed GPS fix (gps_fix_t) |

### Transaction Timing
- **Dummy bytes**: 4 bytes (critical for TX FIFO prefetch race avoidance)
- **IRQ assertion**: Active high on PB2
- **IRQ deassert**: Automatic on transaction complete (NSS rising edge)

## GPS Fix Structure (gps_fix_t)

```c
typedef struct {
    double   latitude;         // Degrees (-90 to +90)           [8 bytes]
    double   longitude;        // Degrees (-180 to +180)         [8 bytes]
    float    altitude_msl;     // Meters above sea level         [4 bytes]
    float    ground_speed;     // m/s                            [4 bytes]
    float    course;           // Degrees true north (0-360)     [4 bytes]
    uint8_t  fix_quality;      // 0=invalid, 1=GPS, 2=DGPS       [1 byte]
    uint8_t  num_satellites;   // Number of satellites used      [1 byte]
    uint8_t  padding1[2];      // Alignment                      [2 bytes]
    float    hdop;             // Horizontal dilution            [4 bytes]
    uint32_t time_of_week_ms;  // GPS time of week (ms)          [4 bytes]
    uint8_t  padding2[8];      // Future use                     [8 bytes]
} __attribute__((packed)) gps_fix_t;  // Total: 48 bytes
```

## Memory Usage

**Debug build** (with injection & logging):
- FLASH: 28.4 KB (22.2% of 128 KB)
- RAM: 11.9 KB (8.1% of 144 KB)

**Release build** (production):
- FLASH: ~26 KB (estimated, no debug overhead)
- RAM: ~10 KB (estimated)

## Testing

### Hardware Testing
1. Connect ST-Link programmer to SWD pins
2. Flash firmware: `cmake --preset Debug && ninja -C build/Debug && st-flash write build/Debug/ulysses-gnss-radio.bin 0x08000000`
3. Monitor UART1 output for startup message

### Debug Injection Testing
Use a serial terminal (e.g., minicom, screen, PuTTY) connected to USART1:

```bash
# Inject radio message "Hello"
echo -ne '\x52Hello\x00' > /dev/ttyUSB0

# Inject GPS NMEA sentence
echo -ne '\x47$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n' > /dev/ttyUSB0
```

Monitor log output to verify messages are enqueued and processed correctly.

### SPI Protocol Testing
Use a logic analyzer or oscilloscope to verify:
- Configuration frame reception on startup
- Pull mode command responses
- Push mode IRQ assertion timing
- DMA transfer completion

## Configuration

### Changing SPI Mode
The operational mode is selected via configuration frame on startup. The flight controller must send the config frame immediately after power-up before normal SPI communication.

### GPS Configuration
GPS update rate and constellation selection can be configured via the configuration frame (bytes 1-2). Implementation of these features depends on GPS module capabilities.

## Troubleshooting

**Problem**: "Awaiting configuration from master" message persists
**Solution**: Master must send 8-byte configuration frame on startup

**Problem**: GPS not parsing NMEA sentences
**Solution**: Check USART6 baud rate (should be 115200), verify GPS module TX connection

**Problem**: Radio messages not received
**Solution**: Verify USART5 baud rate (57600), check radio module UART connection

**Problem**: Debug injection not working
**Solution**: Ensure firmware built with Debug preset (DEBUG macro defined)

## References

- **SPI Protocol**: See `spi_timing.html` (deprecated - contains some inaccuracies)
- **Protocol Definitions**: [Core/Inc/protocol_config.h](Core/Inc/protocol_config.h)
- **GPS Parser**: [lib/lwgps/README.md](lib/lwgps/README.md)
- **STM32G0 Reference**: RM0444 (STM32G0x1 reference manual)

## Contributing

This is part of the UBC Rocket avionics system. For questions or contributions, contact the UBC Rocket Avionics team.

## License

[Add license information here]
