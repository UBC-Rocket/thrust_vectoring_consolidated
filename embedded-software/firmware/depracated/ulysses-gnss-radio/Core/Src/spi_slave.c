/**
 * @file spi_slave.c
 * @brief Low-level register-based SPI slave implementation for STM32G0B1
 *
 * Hardware Configuration:
 * - SPI1 on PA4(NSS), PA5(SCK), PA6(MISO), PA7(MOSI)
 * - IRQ output on PB1 (active high, for push mode)
 * - DMA1 Channel 1 (RX), Channel 2 (TX)
 *
 * Implements hybrid RXNE interrupt + DMA approach:
 * - First byte (command) captured by RXNE interrupt
 * - Remaining bytes handled by DMA (zero CPU overhead)
 * - Hardware NSS for automatic transaction framing
 * - EXTI on NSS rising edge (line 4) for transaction completion
 *
 * Protocol Modes:
 * - Pull mode: Master-initiated transactions [CMD:1][DUMMY:4][PAYLOAD:0-256]
 * - Push mode: Slave-initiated via IRQ [TYPE:1][PAYLOAD:N]
 */

#include "spi_slave.h"
// #include "gps_driver.h"  // TODO: Add when GPS driver is implemented
#include <string.h>
#ifdef DEBUG
#include "debug_uart.h"
#endif

// ============================================================================
// STATIC CONTEXT
// ============================================================================

static spi_slave_context_t ctx;

/** Current protocol mode (set by configuration frame on startup) */
static spi_protocol_mode_t s_protocol_mode = SPI_MODE_PULL;  // Default to pull mode

#ifdef DEBUG
/** Debug capture - updated by ISR/EXTI/arm, read from main loop */
static volatile spi_debug_capture_t s_dbg;
#endif

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

static void spi_gpio_init(void);
static void spi_peripheral_init(void);
static void dmamux_init(void);
static void exti_nss_init(void);
static void spi_rx_dma_start(uint8_t *buf, uint16_t len);
static void spi_tx_dma_start(uint8_t *buf, uint16_t len);
static void spi_slave_arm(void);
static void spi_slave_arm_push(void);
static void spi_slave_arm_push_idle(void);
static void spi_slave_arm_config(void);
static void spi_clear_errors(void);
static uint8_t radio_queue_count(radio_message_queue_t *q);
static void spi_slave_prepare_push(uint8_t push_type);

// ============================================================================
// INITIALIZATION FUNCTIONS
// ============================================================================

/**
 * @brief Configure GPIO pins for SPI1 alternate function
 *
 * This function configures the hardware GPIO pins for SPI communication.
 * All register writes are done directly (not using HAL) for precise control.
 *
 * Pin Assignments:
 * - PA4: NSS  (chip select, managed by hardware)
 * - PA5: SCK  (clock from master)
 * - PA6: MISO (master in, slave out - our TX line)
 * - PA7: MOSI (master out, slave in - our RX line)
 * - PB1: IRQ  (push mode interrupt, active high)
 */
static void spi_gpio_init(void) {
    // ── Enable GPIO Port Clocks ──
    // RCC->IOPENR is the peripheral clock enable register for GPIO ports.
    // Without enabling the clock, the GPIO registers are inaccessible (reads return 0).
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;

    // ── Configure SPI Pins as Alternate Function ──
    // MODER (Mode Register): Controls pin mode - input, output, AF, or analog
    // Each pin uses 2 bits: 00=input, 01=general output, 10=alternate function, 11=analog
    //
    // WHY alternate function mode?
    // In AF mode, the pin is controlled by a peripheral (SPI1), not by GPIO->ODR.
    // The SPI peripheral directly drives MISO and reads MOSI/SCK/NSS without CPU intervention.
    //
    // Register layout for MODER (32 bits, 2 bits per pin):
    // Bits [9:8]   = PA4 mode
    // Bits [11:10] = PA5 mode
    // Bits [13:12] = PA6 mode
    // Bits [15:14] = PA7 mode
    //
    // We clear all 8 bits (mask ~(...)) then set them to 0b10 (alternate function).
    GPIOA->MODER = (GPIOA->MODER
                    & ~((3 << (4*2)) | (3 << (5*2)) | (3 << (6*2)) | (3 << (7*2))))
                    | (2 << (4*2))   // PA4 NSS  → AF mode (0b10)
                    | (2 << (5*2))   // PA5 SCK  → AF mode (0b10)
                    | (2 << (6*2))   // PA6 MISO → AF mode (0b10)
                    | (2 << (7*2));  // PA7 MOSI → AF mode (0b10)

    // ── Configure Pull-Up/Pull-Down Resistors ──
    // PUPDR (Pull-Up/Pull-Down Register): Controls internal resistors
    // Each pin uses 2 bits: 00=no pull, 01=pull-up, 10=pull-down
    //
    // WHY pull-up on NSS?
    // - NSS (chip select) is active low - master pulls it low to select slave
    // - When master is unpowered or disconnected, NSS would float
    // - Floating NSS can cause phantom transactions due to noise pickup
    // - Pull-up ensures NSS stays high (inactive) when master is not driving it
    //
    // WHY only NSS and not other pins?
    // - SCK is always driven by master (never floats)
    // - MOSI is always driven by master (never floats)
    // - MISO is driven by us (never floats from our perspective)
    // - NSS can float during power-up or cable disconnect, so it needs the pull-up
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(3 << (4*2))) | (1 << (4*2));

    // ── Configure Output Speed ──
    // OSPEEDR (Output Speed Register): Controls slew rate (how fast pin transitions)
    // Each pin uses 2 bits: 00=low (2 MHz), 01=medium (10 MHz), 10=high (50 MHz), 11=very high (80 MHz)
    //
    // WHY very high speed for MISO and SCK?
    // - SPI can run up to 16 MHz on STM32G0 (APB clock / 2)
    // - Fast edges ensure clean transitions and minimize setup/hold violations
    // - Higher slew rate reduces edge ringing and signal integrity issues
    // - MISO (our output) must have fast edges to meet master's setup time requirements
    // - SCK (master's clock) benefits from our input buffer being rated for high speed
    //
    // WHY not MOSI or NSS?
    // - We're only setting output speed for pins where it matters (our driven output)
    // - MISO is the only pin WE drive, so it's the most critical
    // - SCK is included for good measure (helps input buffer respond faster)
    //
    // Note: This uses |= (OR-assignment) instead of full assignment, so it only
    // modifies the bits for MISO and SCK, leaving other pins unchanged.
    GPIOA->OSPEEDR |= (3 << (6*2))   // PA6 MISO → very high speed (0b11 = 80 MHz)
                    | (3 << (5*2));  // PA5 SCK  → very high speed (0b11 = 80 MHz)

    // ── Set Alternate Function Selection ──
    // AFR (Alternate Function Register): Selects which peripheral controls the pin
    // AFR[0] = AFRL = pins 0-7, AFR[1] = AFRH = pins 8-15
    // Each pin uses 4 bits (0-15) to select AF0-AF15
    //
    // WHY AF0 for SPI1?
    // - Each STM32 pin can be controlled by multiple peripherals (e.g., USART, SPI, I2C)
    // - The AF number selects which peripheral: AF0, AF1, AF2, etc.
    // - On STM32G0B1, SPI1 on PA4-7 is mapped to AF0 (see datasheet Table 16)
    // - Setting the wrong AF number would disconnect the SPI peripheral from the pins
    //
    // Register layout for AFR[0] (32 bits, 4 bits per pin):
    // Bits [19:16] = PA4 alternate function
    // Bits [23:20] = PA5 alternate function
    // Bits [27:24] = PA6 alternate function
    // Bits [31:28] = PA7 alternate function
    //
    // We clear all 16 bits (mask 0xF per pin) then set them to SPI_AF_NUM (0 for SPI1).
    GPIOA->AFR[0] = (GPIOA->AFR[0]
                    & ~((0xF << (4*4)) | (0xF << (5*4))
                       | (0xF << (6*4)) | (0xF << (7*4))))
                    | (SPI_AF_NUM << (4*4))    // PA4 NSS  → AF0 (SPI1)
                    | (SPI_AF_NUM << (5*4))    // PA5 SCK  → AF0 (SPI1)
                    | (SPI_AF_NUM << (6*4))    // PA6 MISO → AF0 (SPI1)
                    | (SPI_AF_NUM << (7*4));   // PA7 MOSI → AF0 (SPI1)

    // ── Configure Push Mode IRQ Pin (PB1) ──
    // This pin is used to signal the master that we have data to send (push mode).
    //
    // WHY general output mode (not alternate function)?
    // - This pin is NOT controlled by SPI peripheral - we manually assert/deassert it
    // - We use GPIO->BSRR (bit set/reset register) to drive it high/low directly
    // - No peripheral connection needed, so we use mode 0b01 (general output)
    //
    // WHY active high (not active low)?
    // - STM32 GPIO defaults to push-pull output (can drive high or low strongly)
    // - Active high is simpler: high = data ready, low = idle
    // - Master can detect rising edge (idle → data ready) easily with EXTI
    // - No need for open-drain or external pull-up resistors
    //
    // Initial state: LOW (inactive)
    // - Master should not see an IRQ assertion during our initialization
    // - We'll assert it high only when we actually have data to push
    GPIOB->MODER = (GPIOB->MODER & ~(3 << (1*2))) | (1 << (1*2));  // Mode 0b01 = output
    GPIOB->BSRR = IRQ_GPIO_PIN << 16;  // Write to upper 16 bits of BSRR → reset (low)
    //
    // Note on BSRR (Bit Set/Reset Register):
    // - Lower 16 bits [15:0]: Writing 1 SETS the corresponding pin (drives high)
    // - Upper 16 bits [31:16]: Writing 1 RESETS the corresponding pin (drives low)
    // - This allows atomic set/reset without read-modify-write (no race conditions)
    // - IRQ_GPIO_PIN is GPIO_PIN_1 (bit 1), so IRQ_GPIO_PIN << 16 = bit 17
}

/**
 * @brief Configure SPI1 peripheral registers
 *
 * This is a register-level configuration (not using HAL) for maximum control
 * over the SPI peripheral's behavior and timing.
 */
static void spi_peripheral_init(void) {
    // Enable SPI1 clock (SPI1 is on APB2, not APB1!)
    RCC->APBENR2 |= RCC_APBENR2_SPI1EN;

    // Disable SPI during configuration
    SPI1->CR1 &= ~SPI_CR1_SPE;

    // ── CR1 Configuration ──
    // All zeros = slave mode, CPOL=0, CPHA=0 (SPI Mode 0), MSB first, hardware NSS
    SPI1->CR1 = 0;
    //
    // Bit-by-bit breakdown:
    // - CPHA=0: Data sampled on first (leading) clock edge, shifted on second edge
    //           This is SPI Mode 0 (CPOL=0, CPHA=0), the most common mode.
    // - CPOL=0: Clock idles low (inactive state is logic 0)
    // - MSTR=0: Slave mode (we are the slave, master drives SCK)
    // - SSM=0:  Hardware NSS management (NSS pin controls chip select, not software)
    //           CRITICAL: NSS must be used for automatic transaction framing
    // - LSBFIRST=0: MSB transmitted first (standard for most protocols)

    // ── CR2 Configuration ──
    // This is the CRITICAL register for STM32G0 SPI operation.
    // Getting these bits wrong will cause FIFO underruns, missed bytes, or race conditions.
    SPI1->CR2 = (7 << SPI_CR2_DS_Pos)    // DS=0b0111 → 8-bit data frames
              | SPI_CR2_FRXTH             // CRITICAL: FIFO RX threshold = 1 byte (not 2!)
              | SPI_CR2_RXNEIE;           // RXNE interrupt enabled (for command byte)
    //
    // WHY FRXTH (FIFO RX Threshold)?
    // - STM32G0 SPI has a 4-byte RX FIFO (stores received bytes before reading)
    // - FRXTH=0: RXNE flag set when FIFO ≥ 2 bytes (16-bit threshold)
    // - FRXTH=1: RXNE flag set when FIFO ≥ 1 byte (8-bit threshold)
    // - We MUST use FRXTH=1 because we need the RXNE interrupt to fire IMMEDIATELY
    //   after the first byte arrives (the command byte). Without FRXTH, we'd wait
    //   for 2 bytes, missing our critical 4-byte timing window.
    //
    // WHY RXNEIE (RX Not Empty Interrupt Enable)?
    // - We need to capture the command byte in an ISR so we can decode it and patch
    //   the TX buffer before the master clocks out bytes 5-260 (the response region).
    // - This gives us a ~32 µs window (4 dummy bytes × 8 µs/byte @ 1 MHz) to prepare.
    //
    // WHY NO TXDMAEN or RXDMAEN?
    // - TXDMAEN must not be set before SPE=1, otherwise TXE DMA requests fire while
    //   the SPI is disabled and consume bytes from tx_buf that get discarded on SPE enable.
    // - RXDMAEN is deliberately OFF because we need the RXNE interrupt to capture
    //   the first byte. If RXDMAEN were on, the DMA would steal byte 0 before the
    //   ISR could read it, and we'd lose the command byte.
    // - Both are enabled at the appropriate time by spi_slave_arm().

    // Enable SPI peripheral
    SPI1->CR1 |= SPI_CR1_SPE;
}

/**
 * @brief Configure DMAMUX to route SPI requests to DMA channels
 *
 * WHAT IS DMAMUX?
 * ───────────────
 * DMAMUX (DMA Multiplexer) is a routing fabric that connects peripheral DMA
 * requests to DMA controller channels. On STM32G0, peripherals don't have
 * hardwired DMA channels - instead, DMAMUX allows ANY peripheral to trigger
 * ANY DMA channel by writing a request ID to the DMAMUX channel configuration.
 *
 * WHY DO WE NEED THIS?
 * ────────────────────
 * - STM32G0 has 7 DMA channels but 50+ DMA-capable peripherals
 * - Without DMAMUX, we'd be limited to hardwired peripheral-channel pairs
 * - DMAMUX lets us choose which channels to use for each peripheral
 * - This is MANDATORY on STM32G0 - DMA won't work without DMAMUX configuration
 *
 * CRITICAL INDEXING GOTCHA:
 * ─────────────────────────
 * - DMA channels are 1-indexed in hardware names: DMA1_Channel1, DMA1_Channel2, etc.
 * - DMAMUX channels are 0-indexed in register names: DMAMUX1_Channel0, DMAMUX1_Channel1, etc.
 * - DMA1_Channel1 is controlled by DMAMUX1_Channel0 (not DMAMUX1_Channel1!)
 * - DMA1_Channel2 is controlled by DMAMUX1_Channel1 (not DMAMUX1_Channel2!)
 *
 * HOW IT WORKS:
 * ─────────────
 * 1. Peripheral (SPI1) generates a DMA request signal (SPI1_RX or SPI1_TX)
 * 2. DMAMUX sees the request and checks which channel is configured for it
 * 3. DMAMUX forwards the request to the selected DMA channel
 * 4. DMA channel executes the transfer (peripheral ↔ memory)
 *
 * CONFIGURATION:
 * ──────────────
 * We write the peripheral's DMA request ID to DMAMUX_ChannelX->CCR (Channel Configuration Register).
 * Request IDs are listed in RM0444 Table 59 "DMAMUX request mapping".
 */
static void dmamux_init(void) {
    // ── Route SPI1_RX to DMA1 Channel 1 ──
    // SPI1_RX request ID = 16 (from RM0444 Table 59)
    // When SPI1 receives a byte, it triggers DMAREQ_SPI_RX (ID 16)
    // DMAMUX forwards this to DMA1_Channel1 (configured by DMAMUX1_Channel0)
    // DMA1_Channel1 then reads from SPI1->DR and writes to our rx_buf
    //
    // CCR (Channel Configuration Register) format:
    // Bits [6:0]: DMAREQ_ID (peripheral request ID, 0-127)
    // Other bits: Reserved or not used in basic operation
    DMAMUX1_Channel0->CCR = DMAREQ_SPI_RX;  // 16 = SPI1_RX

    // ── Route SPI1_TX to DMA1 Channel 2 ──
    // SPI1_TX request ID = 17 (from RM0444 Table 59)
    // When SPI1's TX FIFO has space, it triggers DMAREQ_SPI_TX (ID 17)
    // DMAMUX forwards this to DMA1_Channel2 (configured by DMAMUX1_Channel1)
    // DMA1_Channel2 then reads from our tx_buf and writes to SPI1->DR
    DMAMUX1_Channel1->CCR = DMAREQ_SPI_TX;  // 17 = SPI1_TX

    // After this configuration:
    // - Every RXNE event from SPI1 will trigger DMA1_Channel1 (if RXDMAEN is set)
    // - Every TXE event from SPI1 will trigger DMA1_Channel2 (if TXDMAEN is set)
    //
    // Note: DMAMUX stays configured for the entire session. We don't need to
    // reconfigure it between transactions - only the DMA channel registers change.
}

/**
 * @brief Configure EXTI for NSS rising edge detection
 *
 * WHAT IS EXTI?
 * ─────────────
 * EXTI (External Interrupt/Event Controller) generates interrupts on GPIO pin edges.
 * We use it to detect when NSS (chip select) goes high, which signals transaction end.
 *
 * WHY USE EXTI FOR NSS?
 * ─────────────────────
 * - SPI peripheral has no "transaction complete" interrupt
 * - NSS is controlled by master: low = transaction active, high = transaction done
 * - Rising edge on NSS (low → high) is the perfect "transaction complete" signal
 * - Allows us to finalize the transaction (pop queues, re-arm DMA, etc.)
 *
 * WHY NOT USE DMA TC (Transfer Complete)?
 * ────────────────────────────────────────
 * - DMA TC fires when DMA finishes its programmed byte count (e.g., 260 bytes)
 * - But master might end the transaction early! (e.g., send only 6 bytes for CMD_RADIO_RXBUF_LEN)
 * - DMA would still wait for 260 bytes, never fire TC, and lock up
 * - NSS rising edge is the UNIVERSAL transaction end signal, regardless of length
 *
 * HOW EXTI WORKS:
 * ───────────────
 * 1. GPIO pin (PA4) transitions from low to high
 * 2. EXTI detects the edge and sets a pending flag (RPR1 register)
 * 3. If the interrupt is enabled (IMR1) and edge type matches (RTSR1), EXTI triggers an interrupt
 * 4. NVIC routes the interrupt to EXTI4_15_IRQHandler()
 * 5. Our handler checks the flag, clears it, and processes the transaction end
 */
static void exti_nss_init(void) {
    // ── Route PA4 to EXTI Line 4 ──
    // EXTI lines are numbered 0-15, corresponding to GPIO pins 0-15 across all ports.
    // PA4, PB4, PC4, etc. all compete for EXTI line 4 - we must select which port.
    //
    // EXTICR (EXTI Configuration Register) array selects the port for each line:
    // - EXTICR[0] configures lines 0-3 (we don't use this)
    // - EXTICR[1] configures lines 4-7 (WE USE THIS for line 4)
    // - EXTICR[2] configures lines 8-11
    // - EXTICR[3] configures lines 12-15
    //
    // Each line gets 8 bits (1 byte) in the register:
    // Bits [7:0]   = Line 4 port selection
    // Bits [15:8]  = Line 5 port selection
    // Bits [23:16] = Line 6 port selection
    // Bits [31:24] = Line 7 port selection
    //
    // Port encoding: 0x00 = Port A, 0x01 = Port B, 0x02 = Port C, etc.
    //
    // We want PA4 → EXTI line 4, so we write 0x00 (Port A) to bits [7:0] of EXTICR[1].
    EXTI->EXTICR[1] = (EXTI->EXTICR[1] & ~(0xFF << 0)) | (NSS_EXTI_PORT << 0);

    // ── Enable Rising Edge Trigger ──
    // RTSR1 (Rising Trigger Selection Register 1) controls which edges trigger interrupts.
    // Bit N = 1: Rising edge on line N triggers interrupt
    // Bit N = 0: Rising edge on line N ignored
    //
    // WHY rising edge (not falling)?
    // - NSS is active low: master pulls NSS low to start transaction, releases high to end
    // - Falling edge (high → low) = transaction START (too early to process)
    // - Rising edge (low → high) = transaction END (perfect time to finalize)
    //
    // We set bit 4 to enable rising edge detection on line 4 (PA4).
    EXTI->RTSR1 |= (1 << NSS_EXTI_LINE);

    // ── Enable Falling Edge Trigger ──
    // FTSR1 (Falling Trigger Selection Register 1) controls falling edge triggers.
    // Bit N = 1: Falling edge on line N triggers interrupt
    // Bit N = 0: Falling edge on line N ignored
    //
    // WHY ALSO falling edge?
    // - Falling edge (high → low) = transaction START
    // - We use this to set nss_busy flag, preventing IRQ assertion mid-transaction
    // - This prevents GPS/Radio push from asserting IRQ while master is sending radio TX
    // - Without this, slave could start clocking GPS on MISO while master isn't reading MISO
    //
    // We set bit 4 to enable falling edge detection on line 4 (PA4).
    EXTI->FTSR1 |= (1 << NSS_EXTI_LINE);

    // ── Unmask the Interrupt ──
    // IMR1 (Interrupt Mask Register 1) controls which EXTI lines can generate interrupts.
    // Bit N = 1: Line N can trigger interrupt (unmasked)
    // Bit N = 0: Line N cannot trigger interrupt (masked)
    //
    // Even if RTSR1 detects a rising edge, the interrupt won't fire unless IMR1 unmasks it.
    // This is a global enable/disable switch for each EXTI line.
    //
    // We set bit 4 to unmask EXTI line 4.
    EXTI->IMR1 |= (1 << NSS_EXTI_LINE);

    // ── Enable EXTI4_15 Interrupt in NVIC ──
    // NVIC (Nested Vectored Interrupt Controller) manages all interrupt priorities and enables.
    // On STM32G0, EXTI lines 4-15 share a single interrupt vector: EXTI4_15_IRQn
    //
    // Interrupt Priority:
    // - Priority 0 = highest (SPI1 RXNE, must respond in <32 µs)
    // - Priority 1 = high (DMA TC, should process quickly)
    // - Priority 2 = medium (EXTI NSS, can wait until transaction fully done)
    //
    // WHY lower priority than SPI/DMA?
    // - SPI RXNE fires during the transaction (command byte arrival, critical timing)
    // - DMA TC fires near the end (all bytes received)
    // - EXTI fires AFTER the transaction (NSS already high, no rush)
    // - If EXTI had higher priority, it could preempt the critical RXNE handler and cause corruption
    //
    // NVIC_SetPriority: Configures the priority level (0-3 on Cortex-M0+)
    // NVIC_EnableIRQ: Unmasks the interrupt in the NVIC (global enable)
    NVIC_SetPriority(EXTI4_15_IRQn, 2);  // Medium priority (lower than SPI/DMA)
    NVIC_EnableIRQ(EXTI4_15_IRQn);       // Enable the interrupt vector
}

// ============================================================================
// DMA HELPER FUNCTIONS
// ============================================================================

/**
 * @brief Start RX DMA: SPI1_DR → buf (peripheral → memory)
 *
 * Configures DMA1 Channel 1 to read from SPI1->DR and write to a buffer.
 * This function sets up the channel but does NOT start the transfer - the
 * transfer only begins when SPI1 asserts RXNE and RXDMAEN is enabled.
 *
 * @param buf Destination buffer (must be aligned to 4-byte boundary for optimal performance)
 * @param len Number of bytes to receive (1-65535)
 */
static void spi_rx_dma_start(uint8_t *buf, uint16_t len) {
    // ── Step 1: Disable Channel ──
    // CRITICAL: DMA registers are READ-ONLY while the channel is enabled.
    // Attempting to write to CPAR, CMAR, CNDTR while EN=1 has no effect.
    // We MUST disable the channel before reconfiguration.
    //
    // CCR (Channel Configuration Register) bit 0 = EN (Enable)
    // Writing 0 to EN initiates the disable sequence, but it's not instant.
    // The channel waits for any in-flight transfer to complete before fully disabling.
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    while (DMA1_Channel1->CCR & DMA_CCR_EN);  // Busy-wait until EN bit reads back as 0
    //
    // WHY the busy-wait?
    // - If we wrote to CPAR/CMAR too early, the old transfer might still be running
    // - The old transfer would corrupt memory by writing to the old address
    // - Waiting ensures the channel is fully quiescent before reconfiguration

    // ── Step 2: Clear Interrupt Flags ──
    // IFCR (Interrupt Flag Clear Register) clears pending flags from previous transfers.
    // Writing 1 to a bit in IFCR clears the corresponding flag in ISR (Interrupt Status Register).
    //
    // CGIF1 = Clear Global Interrupt Flag for Channel 1
    // This is a shortcut that clears ALL flags (TC, HT, TE) for channel 1 in one write.
    //
    // WHY clear flags?
    // - Leftover flags from the previous transfer could cause spurious interrupts
    // - If TCIF1 (transfer complete) is still set, the ISR would fire immediately after we enable EN
    // - Clearing ensures a clean slate for the new transfer
    DMA1->IFCR = DMA_IFCR_CGIF1;

    // ── Step 3: Set Source, Destination, and Count ──
    // These registers define the DMA transfer parameters:
    //
    // CPAR (Channel Peripheral Address Register):
    // - Source address for peripheral → memory (RX DMA)
    // - Always points to SPI1->DR (the SPI data register at 0x4001300C on STM32G0B1)
    // - DMA reads from this address every time SPI1 asserts RXNE
    //
    // CMAR (Channel Memory Address Register):
    // - Destination address for peripheral → memory (RX DMA)
    // - Points to our rx_buf (or rx_buf[1] if we skip the command byte)
    // - DMA writes received bytes here, incrementing after each transfer (MINC=1)
    //
    // CNDTR (Channel Number of Data to Transfer Register):
    // - How many transfers to perform (1-65535)
    // - Decrements after each transfer: len → len-1 → ... → 0
    // - When CNDTR reaches 0, the transfer is complete and TCIF flag is set
    //
    // WHY &SPI1->DR and not just SPI1->DR?
    // - We need the ADDRESS of the register, not its value
    // - DMA controller uses this address to issue hardware read cycles on the AHB bus
    DMA1_Channel1->CPAR  = (uint32_t)&SPI1->DR;   // Source: SPI data register
    DMA1_Channel1->CMAR  = (uint32_t)buf;         // Destination: our buffer
    DMA1_Channel1->CNDTR = len;                   // Number of bytes

    // ── Step 4: Configure Channel Control Register (CCR) ──
    // CCR controls the behavior of the DMA channel. Key bits:
    //
    // DIR (Data Transfer Direction) - bit 4:
    // - DIR=0: Read from peripheral (CPAR), write to memory (CMAR) [WE USE THIS]
    // - DIR=1: Read from memory (CMAR), write to peripheral (CPAR) [TX DMA uses this]
    //
    // MINC (Memory Increment) - bit 7:
    // - MINC=1: Increment CMAR after each transfer (buf[0], buf[1], buf[2], ...)
    // - MINC=0: Keep CMAR fixed (all bytes written to same address)
    // - We MUST set MINC=1 or all RX bytes would overwrite the first byte
    //
    // PINC (Peripheral Increment) - bit 6:
    // - PINC=1: Increment CPAR after each transfer
    // - PINC=0: Keep CPAR fixed (all reads from same peripheral register)
    // - We leave PINC=0 because SPI1->DR is always the same address
    //
    // PSIZE (Peripheral Size) - bits [9:8]:
    // - 00 = 8-bit, 01 = 16-bit, 10 = 32-bit
    // - CRITICAL: Must match SPI frame size! We use 8-bit SPI (CR2.DS = 0b0111)
    // - If we used 16-bit or 32-bit, DMA would read 2 or 4 bytes per transfer, corrupting the stream
    //
    // MSIZE (Memory Size) - bits [11:10]:
    // - 00 = 8-bit, 01 = 16-bit, 10 = 32-bit
    // - Should match PSIZE for byte-to-byte transfers
    // - We use 8-bit to write individual bytes to rx_buf
    //
    // PL (Priority Level) - bits [13:12]:
    // - 00 = low, 01 = medium, 10 = high, 11 = very high
    // - RX has higher priority than TX because overruns lose data permanently
    // - If RX and TX both request service, DMA controller services RX first
    // - We use 0b10 (high priority)
    //
    // TCIE (Transfer Complete Interrupt Enable) - bit 1:
    // - TCIE=1: Trigger interrupt when CNDTR reaches 0
    // - Allows us to process the received data in an ISR
    //
    // TEIE (Transfer Error Interrupt Enable) - bit 3:
    // - TEIE=1: Trigger interrupt on bus errors (e.g., invalid address, AHB fault)
    // - Rare, but good for debugging (e.g., if buf pointer is garbage)
    //
    // We write all these bits in one assignment (not using |= to avoid race conditions).
    DMA1_Channel1->CCR = DMA_CCR_MINC              // Memory increment
                       | DMA_CCR_TCIE              // Transfer complete interrupt
                       | DMA_CCR_TEIE              // Transfer error interrupt
                       | (2 << DMA_CCR_PL_Pos);    // High priority (0b10)
    // Note: DIR=0 (peripheral→memory) is the default, so we don't need to set it explicitly
    // Note: PSIZE=00 and MSIZE=00 (8-bit) are the default, so we don't need to set them

    // ── Step 5: Enable the Channel ──
    // Setting EN=1 activates the channel. From this point on:
    // - DMA listens for requests from DMAMUX (which forwards SPI1_RX requests)
    // - Each time SPI1->SR.RXNE=1 and SPI1->CR2.RXDMAEN=1, DMA reads SPI1->DR
    // - DMA writes the byte to buf, increments buf address, decrements CNDTR
    // - When CNDTR=0, DMA sets TCIF1 and triggers DMA1_Channel1_IRQHandler
    //
    // CRITICAL: The transfer does NOT start yet!
    // Even with EN=1, DMA waits for SPI1 to assert RXNE AND for RXDMAEN to be set.
    // In pull mode, RXDMAEN is deliberately OFF until the RXNE ISR enables it.
    // This allows the ISR to capture the command byte before DMA takes over.
    DMA1_Channel1->CCR |= DMA_CCR_EN;
}

/**
 * @brief Start TX DMA: buf → SPI1_DR (memory → peripheral)
 *
 * Configures DMA1 Channel 2 to read from a buffer and write to SPI1->DR.
 * The transfer begins immediately once TXDMAEN is set in SPI1->CR2.
 *
 * @param buf Source buffer containing data to transmit
 * @param len Number of bytes to transmit (1-65535)
 */
static void spi_tx_dma_start(uint8_t *buf, uint16_t len) {
    // ── Step 1: Disable Channel ──
    // Same as RX DMA - must disable before reconfiguration.
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    while (DMA1_Channel2->CCR & DMA_CCR_EN);  // Wait for disable to complete

    // ── Step 2: Clear Interrupt Flags ──
    // Clear all pending flags for Channel 2 to avoid spurious interrupts.
    DMA1->IFCR = DMA_IFCR_CGIF2;

    // ── Step 3: Set Source, Destination, and Count ──
    // For TX DMA (memory → peripheral), the roles are reversed compared to RX:
    //
    // CPAR (Channel Peripheral Address Register):
    // - Destination address for memory → peripheral (TX DMA)
    // - Always points to SPI1->DR
    // - DMA writes to this address every time SPI1 asserts TXE (TX empty)
    //
    // CMAR (Channel Memory Address Register):
    // - Source address for memory → peripheral (TX DMA)
    // - Points to our tx_buf
    // - DMA reads from here, incrementing after each transfer (MINC=1)
    //
    // CNDTR (Channel Number of Data to Transfer Register):
    // - How many bytes to send (1-65535)
    // - Decrements after each transfer until it reaches 0
    DMA1_Channel2->CPAR  = (uint32_t)&SPI1->DR;   // Destination: SPI data register
    DMA1_Channel2->CMAR  = (uint32_t)buf;         // Source: our buffer
    DMA1_Channel2->CNDTR = len;                   // Number of bytes

    // ── Step 4: Configure Channel Control Register (CCR) ──
    // Similar to RX DMA, but with DIR=1 for the opposite direction.
    //
    // DIR (Data Transfer Direction) - bit 4:
    // - DIR=1: Read from memory (CMAR), write to peripheral (CPAR) [WE USE THIS]
    // - This is the key difference from RX DMA (which uses DIR=0)
    //
    // MINC (Memory Increment) - bit 7:
    // - MINC=1: Increment CMAR after each transfer (buf[0], buf[1], buf[2], ...)
    // - Essential for streaming from tx_buf sequentially
    //
    // PSIZE and MSIZE:
    // - Both 00 (8-bit) to match SPI frame size
    // - Default values, so we don't explicitly set them
    //
    // PL (Priority Level) - bits [13:12]:
    // - We use 0b01 (medium priority) for TX
    // - WHY medium instead of high?
    //   - RX overruns lose data permanently (SPI overwrites old byte with new one)
    //   - TX underruns just send garbage bytes (usually zeros from FIFO)
    //   - If DMA can only service one channel, we want it to be RX
    //   - In practice, both channels get serviced quickly, but RX has priority
    //
    // TCIE and TEIE:
    // - We DON'T enable TX interrupts (TCIE=0, TEIE=0)
    // - WHY no TX complete interrupt?
    //   - We don't care when TX finishes - we care when the TRANSACTION finishes
    //   - Transaction end is signaled by NSS rising edge (EXTI), not by DMA TC
    //   - TX DMA finishing just means we've loaded all bytes into SPI TX FIFO
    //   - The actual transmission over SPI might still be in progress
    //   - NSS rising edge is the authoritative "transaction done" signal
    DMA1_Channel2->CCR = DMA_CCR_DIR               // Memory → peripheral (bit 4 = 1)
                       | DMA_CCR_MINC              // Memory increment
                       | (1 << DMA_CCR_PL_Pos);    // Medium priority (0b01)

    // ── Step 5: Enable the Channel ──
    // Setting EN=1 activates the channel. From this point on:
    // - DMA listens for requests from DMAMUX (which forwards SPI1_TX requests)
    // - Each time SPI1->SR.TXE=1 (TX FIFO not full) and SPI1->CR2.TXDMAEN=1, DMA writes to SPI1->DR
    // - DMA reads from buf, writes to SPI1->DR, increments buf address, decrements CNDTR
    // - When CNDTR=0, transfer is complete (but we don't get an interrupt for this)
    //
    // CRITICAL TX FIFO PREFETCH:
    // When we enable the channel AND TXDMAEN is already set, the DMA will IMMEDIATELY
    // fire multiple transfers to fill the SPI TX FIFO (4 bytes deep on STM32G0).
    // This is why we need 4 dummy bytes - the prefetch pulls bytes 0-3 before the
    // master even clocks them out, so we have time to patch bytes 4-260 in the RXNE ISR.
    DMA1_Channel2->CCR |= DMA_CCR_EN;
}

/**
 * @brief Clear SPI error flags and drain RX FIFO
 *
 * WHAT ERRORS CAN OCCUR?
 * ──────────────────────
 * - OVR (Overrun): RX FIFO full, new byte arrives, old byte lost
 * - UDR (Underrun): TX FIFO empty, master clocks, garbage byte sent
 * - MODF (Mode Fault): NSS pulled low while in master mode (doesn't apply to us)
 * - FRE (Frame Error): Occurs in TI mode (we don't use TI mode)
 *
 * WHY DRAIN THE FIFO?
 * ───────────────────
 * - If a transaction aborts mid-stream (e.g., NSS glitch), bytes may be stuck in FIFO
 * - Leftover bytes would corrupt the next transaction
 * - Draining ensures we start with a clean FIFO
 *
 * HOW TO CLEAR OVR FLAG?
 * ──────────────────────
 * - OVR is a sticky flag - once set, it stays set until explicitly cleared
 * - Clearing sequence (from RM0444 Section 32.4.9):
 *   1. Read SPI_DR (pops the byte that caused the overrun)
 *   2. Read SPI_SR (clears the OVR flag)
 * - We do this in a loop to drain all bytes, then read SR one final time
 */
static void spi_clear_errors(void) {
    volatile uint32_t dummy;

    // ── Drain RX FIFO ──
    // FRLVL (FIFO Reception Level) in SPI_SR indicates how many bytes are in RX FIFO:
    // - 0b00 = empty (0 bytes)
    // - 0b01 = 1/4 full (1 byte)
    // - 0b10 = 1/2 full (2 bytes)
    // - 0b11 = full (4 bytes) - this is bad, overrun imminent!
    //
    // SPI_SR_FRLVL is a 2-bit mask (bits [10:9]) that reads non-zero if FIFO has any data.
    // We loop while FRLVL != 0, reading one byte at a time until FIFO is empty.
    //
    // WHY 8-bit read?
    // - SPI1->DR is a 16-bit register, but we configured 8-bit frames (CR2.DS = 0b0111)
    // - Reading 16 bits would pop 2 bytes from FIFO in one read (if FRXTH=0)
    // - Reading 8 bits pops exactly 1 byte (since FRXTH=1)
    // - The cast to (volatile uint8_t *) forces an 8-bit bus access
    //
    // WHY volatile?
    // - Without volatile, the compiler might optimize away the read (it's unused)
    // - volatile tells the compiler: "This read has side effects (pops FIFO), don't remove it!"
    while (SPI1->SR & SPI_SR_FRLVL) {
        dummy = *(volatile uint8_t *)&SPI1->DR;  // Pop one byte from FIFO
    }

    // ── Clear OVR Flag ──
    // After draining, FRLVL=0 but OVR might still be set.
    // Reading SR clears the OVR flag (per reference manual).
    //
    // We read SR into a dummy variable to ensure the compiler doesn't optimize it away.
    // The (void) cast suppresses "unused variable" warnings.
    dummy = SPI1->SR;
    (void)dummy;  // Tell compiler we intentionally discarded the value

    // After this function:
    // - RX FIFO is empty
    // - OVR flag is cleared
    // - SPI is ready for a clean transaction
}

/**
 * @brief Get radio queue message count
 */
static uint8_t radio_queue_count(radio_message_queue_t *q) {
    if (radio_message_queue_empty(q)) {
        return 0;
    }
    return (q->head - q->tail + RADIO_MESSAGE_QUEUE_LEN) % RADIO_MESSAGE_QUEUE_LEN;
}

// ============================================================================
// ARM FUNCTION (Transaction Preparation)
// ============================================================================

/**
 * @brief Arm SPI slave for next transaction (PULL MODE)
 *
 * Stops all DMA, clears errors, zeros TX buffer, and restarts DMA
 * in the correct initial state (RXNEIE enabled, RXDMAEN disabled).
 *
 * CRITICAL TIMING CONSTRAINT:
 * This function sets up the "hybrid RXNE interrupt + DMA" architecture.
 * The first byte triggers an interrupt, giving us a 4-byte window (32 µs @ 1 MHz)
 * to decode the command and patch the TX buffer before bytes 5-260 are clocked out.
 */
static void spi_slave_arm(void) {
    // ── Step 1: Stop everything ──
    SPI1->CR1 &= ~SPI_CR1_SPE;
#ifdef DEBUG
    s_dbg.pre_arm_sr = SPI1->SR;  // Capture FTLVL[12:11] to detect stale TX FIFO bytes
#endif
    SPI1->CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_RXNEIE);

    // Disable both DMA channels
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;

    // Clear all DMA flags
    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;

    // Reset SPI peripheral via RCC to flush TX FIFO
    // SPE=0 alone does NOT flush the TX FIFO on STM32G0. Stale bytes from
    // the previous transaction persist and get sent before DMA's fresh data,
    // causing a byte offset in the response. RCC reset clears all SPI
    // registers, flushes both TX and RX FIFOs, and clears error flags.
    RCC->APBRSTR2 |= RCC_APBRSTR2_SPI1RST;
    RCC->APBRSTR2 &= ~RCC_APBRSTR2_SPI1RST;

    // ── Step 2: Reset state ──
    ctx.state = SPI_STATE_IDLE;
    ctx.current_cmd = 0;
    ctx.payload_processed = false;

    // ── Step 3: Prepare TX buffer ──
    // Zero the entire buffer. For read commands, response data will be
    // patched in at offset CMD_OVERHEAD (5 bytes = 1 cmd + 4 dummy) by the RXNE ISR.
    //
    // WHY zeros? If the master sends a write command (e.g., CMD_RADIO_TX), we don't
    // have response data to send, so zeros are fine. For read commands, the RXNE ISR
    // will overwrite bytes 5-260 with the actual response before they're transmitted.
    memset(ctx.tx_buf, 0x00, sizeof(ctx.tx_buf));

    // ── Step 4: Start TX DMA ──
    // TX DMA runs from the start, streaming from tx_buf for the full
    // maximum transaction size. During command byte and dummy bytes,
    // it sends zeros (which master ignores). During response region,
    // it sends whatever we patched in (or zeros for write commands).
    //
    // CRITICAL: TX DMA must be active from the START because the TX FIFO will
    // immediately pull 4 bytes when master starts clocking. If we tried to enable
    // TX DMA later, we'd get FIFO underruns and corrupt the transaction.
    spi_tx_dma_start(ctx.tx_buf, MAX_TRANSACTION_SIZE);

    // ── Step 5: Start RX DMA (but gated) ──
    // RX DMA is configured for all bytes after the command byte.
    // It points at rx_buf starting at index 1 (byte 0 is the command,
    // which the RXNE ISR reads manually).
    //
    // CRITICAL: We configure the DMA channel but DON'T enable RXDMAEN yet.
    // The DMA channel is armed and waiting, but the SPI won't send it requests
    // until we flip RXDMAEN on in the RXNE ISR. This prevents the DMA from
    // stealing byte 0 (the command byte) before we can read it.
    spi_rx_dma_start(&ctx.rx_buf[1], MAX_TRANSACTION_SIZE - 1);

    // ── Step 6: Configure SPI CR2 (WITHOUT TXDMAEN) ──
    SPI1->CR2 = (7 << SPI_CR2_DS_Pos)    // 8-bit data frames
              | SPI_CR2_FRXTH            // 1-byte FIFO threshold (trigger RXNE every byte)
              | SPI_CR2_RXNEIE;          // RXNE interrupt for command byte
    // Note: RXDMAEN deliberately OFF - enabled after RXNE ISR reads command byte
    // Note: TXDMAEN deliberately OFF here - enabled AFTER SPE to prevent premature
    //       DMA prefetch while SPI is disabled (SPE=0). Setting TXDMAEN before SPE
    //       causes TXE DMA requests that consume tx_buf bytes into the FIFO, which
    //       are then discarded when SPE=1 resets the SPI state machine, creating a
    //       byte offset in the TX stream.

    // ── Step 7: Enable SPI ──
    SPI1->CR1 |= SPI_CR1_SPE;

    // ── Step 8: Enable TX DMA ──
    // Now that SPE=1, TXE reflects real FIFO state. DMA will prefetch bytes
    // from tx_buf[0] into the TX FIFO correctly.
    SPI1->CR2 |= SPI_CR2_TXDMAEN;

#ifdef DEBUG
    s_dbg.arm_count++;
    s_dbg.arm_sr = SPI1->SR;
    s_dbg.arm_cr2 = SPI1->CR2;
    s_dbg.arm_tx_cndtr = DMA1_Channel2->CNDTR;
    s_dbg.arm_tx_cmar = DMA1_Channel2->CMAR;
    s_dbg.arm_dmamux0 = DMAMUX1_Channel0->CCR;
    s_dbg.arm_dmamux1 = DMAMUX1_Channel1->CCR;
#endif
}

/**
 * @brief Arm SPI slave for push mode transaction
 *
 * PUSH MODE vs PULL MODE DIFFERENCES:
 * ────────────────────────────────────
 * - PULL mode: Master sends [CMD:1][DUMMY:4][PAYLOAD:N], we respond based on command
 *   - First byte (CMD) captured by RXNE interrupt for fast decoding
 *   - Remaining bytes handled by DMA after RXNEIE disabled and RXDMAEN enabled
 *   - TX buffer patched during dummy byte window (32 µs @ 1 MHz)
 *
 * - PUSH mode: We send [TYPE:1][PAYLOAD:N], master optionally sends data back
 *   - No command byte to decode - we already know what we're sending
 *   - Both TX and RX DMA enabled from the start (no hybrid RXNE + DMA)
 *   - TX buffer pre-loaded by spi_slave_prepare_push() before transaction starts
 *   - RX buffer captures any data master sends (simultaneous TX/RX)
 *
 * CRITICAL DIFFERENCES IN CR2 CONFIGURATION:
 * ──────────────────────────────────────────
 * - RXNEIE: OFF (no need for command byte interrupt)
 * - TXDMAEN: ON from the start (TX buffer already prepared)
 * - RXDMAEN: ON from the start (capture any master data simultaneously)
 *
 * WHY NO RXNE INTERRUPT IN PUSH MODE?
 * ────────────────────────────────────
 * - We're the initiator - we already know what data to send (TYPE byte + payload)
 * - No command to decode, no TX buffer to patch mid-transaction
 * - Master might send data back (e.g., radio TX while we're pushing GPS), but we
 *   don't need to process it until NSS rises (transaction complete)
 */
static void spi_slave_arm_push(void) {
    // ── Step 1: Stop everything ──
    // Disable SPI peripheral and all DMA activity
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_RXNEIE);

    // Disable both DMA channels
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;

    // Clear all DMA flags
    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;

    // Reset SPI peripheral via RCC to flush TX FIFO (see spi_slave_arm)
    RCC->APBRSTR2 |= RCC_APBRSTR2_SPI1RST;
    RCC->APBRSTR2 &= ~RCC_APBRSTR2_SPI1RST;

    // ── Step 2: Clear RX buffer ──
    // Zero the RX buffer so we can detect if master sent real data vs dummy bytes.
    // After the transaction, spi_slave_rx_has_valid_data() checks for non-zero pattern.
    // If master sends all zeros (dummy bytes), we know it was just clocking out our data.
    // If master sends non-zero data, we know it sent a radio TX message simultaneously.
    memset(ctx.rx_buf, 0x00, sizeof(ctx.rx_buf));

    // ── Step 3: Start TX DMA with the prepared push data ──
    // tx_buf was already loaded by spi_slave_prepare_push() with:
    // - Byte 0: PUSH_TYPE_RADIO (0x01) or PUSH_TYPE_GPS (0x05)
    // - Bytes 1-N: Radio message (256 bytes) or GPS data (48 bytes)
    //
    // ctx.tx_dma_length was set by spi_slave_prepare_push() to the correct size:
    // - PUSH_RADIO_TOTAL = 257 bytes (1 type + 256 payload)
    // - PUSH_GPS_TOTAL = 49 bytes (1 type + 48 payload)
    //
    // WHY variable length?
    // - Master doesn't know the push payload size until it reads the TYPE byte
    // - We only DMA the exact number of bytes needed, not MAX_TRANSACTION_SIZE
    // - Saves time (shorter transactions) and reduces RX FIFO overrun risk
    spi_tx_dma_start(ctx.tx_buf, ctx.tx_dma_length);

    // ── Step 4: Start RX DMA to capture full transaction ──
    // RX DMA captures everything master sends, even though we're pushing data.
    // This enables SIMULTANEOUS TX/RX:
    // - We send GPS fix (49 bytes)
    // - Master sends radio TX message (257 bytes) at the same time
    // - Both transfers complete, both are valid
    //
    // WHY MAX_TRANSACTION_SIZE for RX (not tx_dma_length)?
    // - We don't know how much master will send
    // - Master might send more bytes than we're transmitting (e.g., we send 49, master sends 257)
    // - If RX DMA length is too short, we'd get an overrun
    // - MAX_TRANSACTION_SIZE (261 bytes) ensures we can capture anything master sends
    spi_rx_dma_start(ctx.rx_buf, MAX_TRANSACTION_SIZE);

    // ── Step 5: Configure SPI CR2 for push mode ──
    // KEY DIFFERENCE FROM PULL MODE: Both TXDMAEN and RXDMAEN enabled from the start.
    //
    // Pull mode CR2 configuration (from spi_slave_arm):
    //   SPI1->CR2 = DS | FRXTH | RXNEIE | TXDMAEN;   // RXDMAEN OFF initially
    //
    // Push mode CR2 configuration (here):
    //   SPI1->CR2 = DS | FRXTH | TXDMAEN | RXDMAEN;  // RXNEIE OFF, both DMA ON
    //
    // WHY both DMA ON?
    // - No command byte to intercept - we already prepared TX buffer
    // - TX DMA can start streaming immediately when master clocks SCK
    // - RX DMA can capture master data simultaneously
    // - No need for the hybrid RXNE + DMA handoff
    // ── Step 5: Configure SPI CR2 (WITHOUT DMA enables) ──
    SPI1->CR2 = (7 << SPI_CR2_DS_Pos)    // 8-bit data frames
              | SPI_CR2_FRXTH;            // 1-byte FIFO threshold

    // ── Step 6: Enable SPI ──
    SPI1->CR1 |= SPI_CR1_SPE;

    // ── Step 6b: Enable DMA after SPE ──
    // TXDMAEN and RXDMAEN must be set AFTER SPE=1 to prevent premature
    // DMA transfers while SPI is disabled. See spi_slave_arm() for details.
    SPI1->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;

    // ── Step 7: Update state ──
    // HAVE_DATA state indicates:
    // - IRQ line has been asserted (or will be after this function returns)
    // - TX and RX DMAs are armed and ready
    // - Waiting for master to assert NSS and start clocking
    ctx.state = SPI_STATE_HAVE_DATA;
    ctx.payload_processed = false;
}

/**
 * @brief Arm SPI slave for push-ready idle state (STRICT MODE)
 *
 * This function arms the SPI in a "push-ready" idle state for PUSH mode only.
 * Unlike spi_slave_arm() (pull mode) or spi_slave_arm_push() (active push),
 * this state is waiting for data to become available before asserting IRQ.
 *
 * WHEN TO CALL THIS:
 * ──────────────────
 * - After a push transaction completes in PUSH mode
 * - When s_protocol_mode == SPI_MODE_PUSH
 * - To enforce strict protocol mode (no pull commands accepted)
 *
 * STATE MACHINE FLOW (PUSH MODE):
 * ────────────────────────────────
 * IDLE (this state) → [data available] → spi_slave_prepare_push()
 *                                      → spi_slave_arm_push()
 *                                      → HAVE_DATA (IRQ asserted)
 *                                      → [master clocks data]
 *                                      → ACTIVE
 *                                      → [NSS rises]
 *                                      → spi_slave_arm_push_idle() (back to IDLE)
 *
 * CONFIGURATION:
 * ──────────────
 * - State = IDLE (ready for new transaction)
 * - TX/RX buffers zeroed (clean slate)
 * - SPI enabled but no DMAs running yet
 * - No RXNEIE (not needed in push mode, no command byte to capture)
 * - DMAs will be configured and started when spi_slave_prepare_push() is called
 *
 * DIFFERENCE FROM spi_slave_arm() (PULL MODE):
 * ─────────────────────────────────────────────
 * Pull mode (spi_slave_arm):
 * - RXNEIE enabled (waiting for command byte interrupt)
 * - TX DMA armed for 261 bytes (max transaction)
 * - RX DMA configured but RXDMAEN disabled (enabled in RXNE ISR)
 * - Ready to respond to master commands immediately
 *
 * Push idle (this function):
 * - No RXNEIE (not waiting for commands)
 * - DMAs not started yet (will be started by spi_slave_prepare_push)
 * - Waiting for spi_slave_tick() to detect data and prepare push
 * - Master commands NOT supported in this state (strict push mode)
 */
static void spi_slave_arm_push_idle(void) {
    // ── Step 1: Stop Everything ──
    // Disable SPI and all DMA activity to safely reconfigure
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_RXNEIE);

    // Disable both DMA channels
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;  // RX DMA off
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;  // TX DMA off

    // Clear all DMA interrupt flags for both channels
    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;

    // Reset SPI peripheral via RCC to flush TX FIFO (see spi_slave_arm)
    RCC->APBRSTR2 |= RCC_APBRSTR2_SPI1RST;
    RCC->APBRSTR2 &= ~RCC_APBRSTR2_SPI1RST;

    // ── Step 2: Reset State ──
    ctx.state = SPI_STATE_IDLE;  // Ready for new transaction
    ctx.current_cmd = 0;         // No command (push mode doesn't use commands)
    ctx.payload_processed = false;

    // ── Step 3: Zero Buffers ──
    // Clear TX and RX buffers to ensure clean state for next push
    // When spi_slave_prepare_push() is called, it will load fresh data into tx_buf
    memset(ctx.tx_buf, 0x00, sizeof(ctx.tx_buf));
    memset(ctx.rx_buf, 0x00, sizeof(ctx.rx_buf));

    // ── Step 4: Arm TX DMA (zeros) and RX DMA ──
    // Even in idle, we arm both DMAs so that the master can send radio TX
    // messages at any time. TX DMA sends zeros (dummy data on MISO), while
    // RX DMA captures whatever the master sends on MOSI.
    //
    // When spi_slave_tick() detects push data, it calls prepare_push() which
    // calls arm_push() — that function does its own SPI reset and DMA re-arm,
    // cleanly replacing these idle DMAs.
    ctx.tx_dma_length = MAX_TRANSACTION_SIZE;
    spi_tx_dma_start(ctx.tx_buf, MAX_TRANSACTION_SIZE);
    spi_rx_dma_start(ctx.rx_buf, MAX_TRANSACTION_SIZE);

    // ── Step 5: Configure SPI CR2 ──
    // 8-bit data frames, 1-byte FIFO threshold
    // DMA enables are set AFTER SPE to prevent premature transfers
    SPI1->CR2 = (7 << SPI_CR2_DS_Pos)    // 8-bit data frames (DS=0b0111)
              | SPI_CR2_FRXTH;            // 1-byte FIFO threshold (trigger RXNE every byte)

    // ── Step 6: Enable SPI ──
    SPI1->CR1 |= SPI_CR1_SPE;

    // ── Step 7: Enable DMA after SPE ──
    // TXDMAEN and RXDMAEN must be set AFTER SPE=1 to prevent premature
    // DMA transfers while SPI is disabled.
    SPI1->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;

    // At this point:
    // - state = IDLE
    // - SPI enabled with TX/RX DMA armed (zeros on MISO, captures MOSI)
    // - No IRQ asserted (IRQ line is low/inactive)
    // - Master can send radio TX and slave will capture it in NSS handler
    // - When push data is available, spi_slave_tick() calls prepare_push()

#ifdef DEBUG
    debug_uart_log_spi_idle_arm();
#endif
}

/**
 * @brief Arm SPI slave for configuration frame reception
 *
 * Used on startup to wait for the configuration frame from the master.
 * Arms DMA to receive CONFIG_FRAME_SIZE bytes (8 bytes).
 */
static void spi_slave_arm_config(void) {
    // ── Step 1: Stop everything ──
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_RXNEIE);

    // Disable both DMA channels
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;

    // Clear all DMA flags
    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;

    // Reset SPI peripheral via RCC to flush TX FIFO (see spi_slave_arm)
    RCC->APBRSTR2 |= RCC_APBRSTR2_SPI1RST;
    RCC->APBRSTR2 &= ~RCC_APBRSTR2_SPI1RST;

    // ── Step 2: Set state ──
    ctx.state = SPI_STATE_UNCONFIGURED;

    // ── Step 3: Zero buffers ──
    memset(ctx.tx_buf, 0x00, sizeof(ctx.tx_buf));
    memset(ctx.rx_buf, 0x00, sizeof(ctx.rx_buf));

    // ── Step 4: Start TX DMA (send zeros as dummy data) ──
    spi_tx_dma_start(ctx.tx_buf, CONFIG_FRAME_SIZE);

    // ── Step 5: Start RX DMA to receive config frame ──
    spi_rx_dma_start(ctx.rx_buf, CONFIG_FRAME_SIZE);

    // ── Step 6: Configure SPI CR2 for config reception ──
    // Both TX and RX DMA enabled from the start
    SPI1->CR2 = (7 << SPI_CR2_DS_Pos)    // 8-bit data frames
              | SPI_CR2_FRXTH            // 1-byte FIFO threshold
              | SPI_CR2_TXDMAEN          // TX DMA enabled
              | SPI_CR2_RXDMAEN;         // RX DMA enabled

    // ── Step 7: Enable SPI ──
    SPI1->CR1 |= SPI_CR1_SPE;
}

/**
 * @brief Prepare TX buffer for push and arm DMAs
 *
 * Loads the TX buffer with [TYPE:1][PAYLOAD:N] and configures
 * DMAs for the push transaction.
 *
 * @param push_type PUSH_TYPE_RADIO or PUSH_TYPE_GPS
 */
static void spi_slave_prepare_push(uint8_t push_type) {
    // ── CRITICAL: Check if master is currently mid-transaction ──
    // If NSS is low (busy flag set), master is actively clocking.
    // Asserting IRQ now would cause slave to start pushing data on MISO,
    // but master isn't reading MISO (it's sending on MOSI).
    // Result: GPS/Radio push data would be LOST.
    //
    // Solution: Defer push to next tick. Data stays in queue, will be sent
    // after master completes current transaction (NSS rises, busy flag clears).
    if (ctx.nss_busy) {
        ctx.irq_deferred_busy++;  // Diagnostic counter
        return;  // Exit early - do NOT prepare push, do NOT assert IRQ
    }

    // Zero the TX buffer first
    memset(ctx.tx_buf, 0x00, sizeof(ctx.tx_buf));

    ctx.pending_push_type = push_type;

    switch (push_type) {
    case PUSH_TYPE_RADIO: {
        // Load type byte
        ctx.tx_buf[0] = PUSH_TYPE_RADIO;

        // Load payload from queue (peek, don't dequeue yet)
        // Use tail pointer for FIFO behavior (push oldest message first)
        uint8_t *msg_ptr;
        if (radio_message_queue_tail_pointer(ctx.radio_queue, &msg_ptr)) {
            memcpy(&ctx.tx_buf[PUSH_TYPE_BYTES], msg_ptr, PUSH_RADIO_PAYLOAD);
        }

        // Set TX length: type byte + radio payload
        ctx.tx_dma_length = PUSH_RADIO_TOTAL;  // 1 + 256 = 257
        break;
    }

    case PUSH_TYPE_GPS: {
        // Load type byte
        ctx.tx_buf[0] = PUSH_TYPE_GPS;

        // Load parsed GPS fix from fix queue (peek, don't dequeue yet)
        const gps_fix_t *fix_ptr;
        if (gps_fix_queue_tail_pointer(ctx.gps_fix_queue, &fix_ptr)) {
            memcpy(&ctx.tx_buf[PUSH_TYPE_BYTES], (const uint8_t *)fix_ptr, PUSH_GPS_PAYLOAD);
        }

        // Set TX length: type byte + gps_fix_t payload
        ctx.tx_dma_length = PUSH_GPS_TOTAL;  // 1 + 48 = 49
        break;
    }

    default:
        // Unknown push type - don't arm
        return;
    }

    // Arm DMAs for push transaction
    spi_slave_arm_push();

    // Assert IRQ to notify master
    spi_slave_assert_irq();

#ifdef DEBUG
    debug_uart_log_spi_push_arm(push_type, ctx.tx_dma_length);
    debug_uart_log_spi_push_event("IRQ_ASSERT_PB1", push_type, ctx.tx_dma_length, 0);
#endif
}

// ============================================================================
// INTERRUPT HANDLERS
// ============================================================================

/**
 * @brief SPI1 RXNE Interrupt Handler - THE CRITICAL PATH
 *
 * This fires when the first byte (command) arrives and RXNE flag is set.
 * This is the heart of the hybrid RXNE + DMA design.
 *
 * TIMING CONSTRAINT: This entire function must complete in < 32 µs @ 1 MHz SPI clock
 * (4 dummy bytes × 8 µs/byte). If we exceed this window, the master will start
 * clocking out response bytes before we've patched the TX buffer, causing corruption.
 *
 * Steps:
 * 1. Read command byte manually (8-bit DR read to pop FIFO)
 * 2. Decode command and patch tx_buf if needed (read commands: memcpy response data)
 * 3. Disable RXNEIE (no more byte-by-byte interrupts)
 * 4. Enable RXDMAEN (hand off bytes 1-260 to DMA for zero CPU overhead)
 *
 * WHY THIS WORKS:
 * - Master sends: [CMD:1][DUMMY:4][PAYLOAD:0-256]
 * - We read CMD immediately, then have 4 byte-times to prepare
 * - Response data goes at offset 5 (CMD_OVERHEAD), so even if TX FIFO prefetches
 *   4 bytes, it only gets CMD + 4 dummy bytes (all zeros, harmless)
 * - By the time master clocks byte 5, our memcpy is done and response is ready
 */
void spi_slave_spi1_irq_handler(void) {
    // Verify this is an RXNE interrupt
    if ((SPI1->SR & SPI_SR_RXNE) && (SPI1->CR2 & SPI_CR2_RXNEIE)) {

        // ── Read command byte ──
        // CRITICAL: Must use 8-bit read to pop exactly ONE byte from RX FIFO.
        // A 16-bit or 32-bit read would pop multiple bytes and corrupt the stream.
        // This read also automatically clears the RXNE flag.
        uint8_t cmd = *(volatile uint8_t *)&SPI1->DR;
        ctx.current_cmd = cmd;
        ctx.rx_buf[0] = cmd;  // Save command for later processing

#ifdef DEBUG
        s_dbg.isr_count++;
        s_dbg.isr_cmd = cmd;
        s_dbg.isr_sr = SPI1->SR;
        s_dbg.isr_tx_cndtr = DMA1_Channel2->CNDTR;
        memcpy((void*)s_dbg.isr_tx_snap, ctx.tx_buf, 8);
#endif

        // ── Disable RXNE interrupt ──
        // We got the command byte. We don't want RXNE firing for every subsequent byte
        // (that would be 260 more interrupts!). DMA will handle the rest silently.
        SPI1->CR2 &= ~SPI_CR2_RXNEIE;

        // ── Process command and patch TX buffer ──
        // The master is clocking dummy bytes 1-4 right now.
        // We have ~4 byte-times to complete this before real data starts.

        switch (cmd) {

        // ────────────────────────────────────
        // READ COMMANDS: Slave sends data to master
        // Patch tx_buf with response data starting at CMD_OVERHEAD
        // ────────────────────────────────────

        case CMD_RADIO_RX_LIFO: {
            // Get pointer to most recent radio message (don't pop yet)
            uint8_t *msg_ptr;
            if (radio_message_queue_head_pointer(ctx.radio_queue, &msg_ptr)) {
                memcpy(&ctx.tx_buf[CMD_OVERHEAD], msg_ptr, 256);
            }
            // If queue empty, tx_buf already zeroed
            break;
        }

        case CMD_RADIO_RX_FIFO: {
            // Get pointer to oldest radio message (don't pop yet)
            uint8_t *msg_ptr;
            if (radio_message_queue_tail_pointer(ctx.radio_queue, &msg_ptr)) {
                memcpy(&ctx.tx_buf[CMD_OVERHEAD], msg_ptr, 256);
            }
            break;
        }

        case CMD_RADIO_RXBUF_LEN: {
            // Return number of messages in radio queue (0-255)
            ctx.tx_buf[CMD_OVERHEAD] = radio_queue_count(ctx.radio_queue);
#ifdef DEBUG
            debug_uart_log_spi_buflen(ctx.tx_buf[CMD_OVERHEAD]);
#endif
            break;
        }

        case CMD_GPS_RX: {
            // Get pointer to oldest GPS sample (don't pop yet)
            uint8_t *gps_ptr;
            if (gps_sample_queue_tail_pointer(ctx.gps_queue, &gps_ptr)) {
                memcpy(&ctx.tx_buf[CMD_OVERHEAD], gps_ptr, GPS_SAMPLE_SIZE);
            }
            break;
        }

        // ────────────────────────────────────
        // WRITE COMMANDS: Master sends data to slave
        // No action needed - RX DMA will capture payload
        // ────────────────────────────────────

        case CMD_RADIO_TX:
            // Payload will land at rx_buf[CMD_OVERHEAD..CMD_OVERHEAD+255]
            // via RX DMA. We'll process it in DMA TC ISR or EXTI ISR.
            break;

        default:
            // Unknown command - DMA will still capture bytes,
            // but we'll ignore them
            ctx.unknown_commands++;
            break;
        }

        // ── Enable RX DMA ──
        // This is the critical handoff from interrupt-driven to DMA-driven.
        // From this point on, all incoming bytes trigger DMA requests instead
        // of RXNE interrupts. The DMA silently handles all remaining bytes.
        SPI1->CR2 |= SPI_CR2_RXDMAEN;

        ctx.state = SPI_STATE_ACTIVE;
    }
}

/**
 * @brief DMA1 Channel 1 (RX) Interrupt Handler
 *
 * Fires when RX DMA transfer completes or errors.
 * Transfer complete means all MAX_TRANSACTION_SIZE-1 bytes have been received.
 * This only happens for full-length transactions (261 bytes total).
 */
void spi_slave_dma1_ch1_irq_handler(void) {

    // ── Transfer Complete ──
    if (DMA1->ISR & DMA_ISR_TCIF1) {
        DMA1->IFCR = DMA_IFCR_CTCIF1;  // Clear flag

        // All 260 bytes received (after command byte).
        // For write commands, process the payload.
        if (ctx.current_cmd == CMD_RADIO_TX && !ctx.payload_processed) {
            // Payload is at rx_buf[CMD_OVERHEAD..CMD_OVERHEAD+255]
            // (byte 0 is cmd from RXNE ISR, bytes 1-4 are dummy, bytes 5-260 are payload)
            if (ctx.radio_tx_queue) {
                radio_message_enqueue(256, &ctx.rx_buf[CMD_OVERHEAD], ctx.radio_tx_queue);
#ifdef DEBUG
                /* Log SPI radio TX to debug console */
                debug_uart_log_spi_radio_tx(&ctx.rx_buf[CMD_OVERHEAD], 256);
#endif
                ctx.payload_processed = true;
            }
        }

        // Transaction cleanup will be handled by NSS rising edge EXTI
    }

    // ── Transfer Error ──
    if (DMA1->ISR & DMA_ISR_TEIF1) {
        DMA1->IFCR = DMA_IFCR_CTEIF1;  // Clear flag
        ctx.transfer_errors++;

        // DMA bus error - reset everything
        spi_slave_arm();
    }
}

/**
 * @brief NSS EXTI Handler (line 4, PA4)
 *
 * Fires when NSS rising edge detected (chip select released by master).
 * This is the UNIVERSAL "transaction complete" signal for both pull and push modes.
 *
 * WHY NSS RISING EDGE = TRANSACTION COMPLETE?
 * ───────────────────────────────────────────
 * - Master controls NSS (chip select) as the transaction framing signal
 * - Falling edge (high → low): Master asserts NSS, transaction STARTS, SPI activates
 * - Rising edge (low → high): Master releases NSS, transaction ENDS, SPI deactivates
 * - We detect rising edge with EXTI to finalize the transaction (pop queues, re-arm, etc.)
 *
 * WHY NOT USE DMA TC (Transfer Complete)?
 * ────────────────────────────────────────
 * - DMA TC fires when the programmed byte count is reached (e.g., 260 bytes)
 * - But master might send FEWER bytes (e.g., 6 bytes for CMD_RADIO_RXBUF_LEN)
 * - DMA would wait forever for 260 bytes, never fire TC, and lock up
 * - NSS rising edge happens regardless of transaction length - it's the authoritative end signal
 *
 * WHAT THIS HANDLER DOES:
 * ───────────────────────
 * 1. Checks which state we're in (UNCONFIGURED, IDLE, HAVE_DATA, ACTIVE)
 * 2. Finalizes the transaction based on mode:
 *    - Pull mode (ACTIVE): Check if write command payload fully received, pop queues if needed
 *    - Push mode (HAVE_DATA): Deassert IRQ, pop sent message, check for simultaneous master TX
 *    - Config mode (UNCONFIGURED): Parse configuration frame, set protocol mode
 * 3. Re-arms SPI for the next transaction
 *
 * INTERRUPT TIMING:
 * ─────────────────
 * - This interrupt has LOWER priority (2) than SPI RXNE (0) and DMA TC (1)
 * - If RXNE fires during this handler, RXNE preempts (higher priority)
 * - This is intentional - RXNE has tight timing constraints (<32 µs), EXTI does not
 */
void spi_slave_nss_exti_handler(void) {

    // ── Check for falling edge (NSS asserted = transaction start) ──
    // FPR1 (Falling edge Pending Register 1) has a bit set for each EXTI line that detected a falling edge.
    // Bit 4 corresponds to EXTI line 4 (PA4 = NSS).
    //
    // WHY handle falling edge?
    // - Falling edge (high → low) = master asserts NSS, transaction begins
    // - We set nss_busy flag to prevent slave from asserting IRQ mid-transaction
    // - This prevents GPS/Radio push from interfering with active master radio TX
    //
    // CRITICAL: Handle falling edge FIRST before rising edge check.
    uint32_t falling_pending = EXTI->FPR1 & (1 << NSS_EXTI_LINE);

    if (falling_pending) {
        // ── Clear falling edge pending flag ──
        // FPR1 is write-1-to-clear: writing 1 to bit 4 clears the pending flag.
        EXTI->FPR1 = (1 << NSS_EXTI_LINE);

        // ── Set busy flag ──
        // Master is now active (NSS low = transaction in progress).
        // Prevent spi_slave_prepare_push() from asserting IRQ during this transaction.
        ctx.nss_busy = true;
        ctx.nss_falling_events++;

        // Nothing else to do on falling edge - return immediately
        return;
    }

    // ── Check for rising edge (NSS deasserted = transaction complete) ──
    // RPR1 (Rising edge Pending Register 1) has a bit set for each EXTI line that detected a rising edge.
    // Bit 4 corresponds to EXTI line 4 (PA4 = NSS).
    //
    // WHY check the bit?
    // - This handler covers EXTI lines 4-15 (EXTI4_15_IRQn)
    // - Other EXTI lines (5, 6, ..., 15) might share this handler
    // - We only care about line 4 (NSS), so we check bit 4 specifically
    uint32_t rising_pending = EXTI->RPR1 & (1 << NSS_EXTI_LINE);

    if (rising_pending) {
        // Track whether a push transmission was incomplete (for two-phase master reads).
        // When true, we skip re-arming so DMA/FIFO state is preserved for the next CS assertion.
        bool push_incomplete = false;

        // Track whether the handler already re-armed (e.g. not-yet-read path called arm_push).
        // When true, we skip the bottom-of-handler re-arm.
        bool push_handled = false;

        // ── Clear rising edge pending flag ──
        // RPR1 is write-1-to-clear: writing 1 to bit 4 clears the pending flag.
        // This MUST be done to acknowledge the interrupt, or it will fire again immediately.
        //
        // Note: We write ONLY bit 4, not all bits. Writing 1s to other bits would
        // accidentally clear pending flags for other EXTI lines.
        EXTI->RPR1 = (1 << NSS_EXTI_LINE);

        // ── Clear busy flag ──
        // Transaction complete (NSS high = master released chip select).
        // Safe to assert IRQ again for push mode.
        ctx.nss_busy = false;
        ctx.nss_rising_events++;

#ifdef DEBUG
        s_dbg.exti_count++;
        s_dbg.exti_sr = SPI1->SR;
        s_dbg.exti_tx_cndtr = DMA1_Channel2->CNDTR;
        s_dbg.exti_rx_cndtr = DMA1_Channel1->CNDTR;
        memcpy((void*)s_dbg.exti_tx_snap, ctx.tx_buf, 8);
        memcpy((void*)s_dbg.exti_rx_snap, ctx.rx_buf, 8);
#endif

        // ── Handle based on state ──
        switch (ctx.state) {

        case SPI_STATE_UNCONFIGURED:
            // ══════════════════════════════════════════════════════════════
            // CONFIGURATION FRAME RECEIVED
            // ══════════════════════════════════════════════════════════════
            // On power-up, the slave waits for the master to send a configuration frame
            // before entering normal operation mode. This allows the flight controller to
            // dynamically select pull vs push mode and configure GPS settings.
            //
            // Configuration Frame Structure (8 bytes):
            // ┌────────┬──────────┬──────────────────┬────────────┐
            // │ Byte 0 │ Byte 1   │ Byte 2           │ Bytes 3-7  │
            // ├────────┼──────────┼──────────────────┼────────────┤
            // │ Mode   │ GPS Rate │ GPS Constellation│ Reserved   │
            // └────────┴──────────┴──────────────────┴────────────┘
            //
            // Mode byte (byte 0):
            //   0x00 = Pull mode (master-initiated, command-based)
            //   0x01 = Push mode (slave-initiated via IRQ assertion)
            //
            // GPS Rate byte (byte 1):
            //   Update rate in Hz (1-10 Hz typical for GPS modules)
            //
            // GPS Constellation byte (byte 2):
            //   Bitmask for enabled constellations (GPS, GLONASS, Galileo, BeiDou)
            //
            // Reserved bytes (3-7):
            //   Future expansion for additional configuration parameters
            {
                config_frame_t *config = (config_frame_t*)ctx.rx_buf;

                // ── Validate and Set Protocol Mode ──
                // The mode byte determines how the slave operates for the entire session.
                // Pull mode is the fallback/default if master sends an invalid mode byte.
                //
                // WHY validate?
                // - Master might send garbage data due to electrical noise
                // - Invalid mode byte (e.g., 0xFF) could cause undefined behavior
                // - Defaulting to PULL mode is safest (works with all masters)
                if (config->mode == SPI_MODE_PULL) {
                    s_protocol_mode = SPI_MODE_PULL;
                } else if (config->mode == SPI_MODE_PUSH) {
                    s_protocol_mode = SPI_MODE_PUSH;
                } else {
                    // Invalid mode - default to PULL (conservative choice)
                    s_protocol_mode = SPI_MODE_PULL;
                }

                // ── Apply GPS Configuration ──
                // TODO: Add GPS configuration handling when GPS driver supports it
                // The GPS driver would expose functions like:
                //   - gps_set_update_rate(config->gps_rate);        // Set fix rate (1-10 Hz)
                //   - gps_set_constellation(config->gps_constellation); // Enable/disable GNSS constellations
                //
                // For now, GPS runs with compile-time defaults.

                // ── Increment Transaction Counter ──
                // Configuration frame counts as a transaction for diagnostic purposes.
                ctx.transactions_completed++;

#ifdef DEBUG
                debug_uart_log_spi_push_event("CONFIG",
                                               (uint8_t)s_protocol_mode, 0, 0);
#endif

                // ── Transition to Normal Operation ──
                // spi_slave_arm() configures the SPI for the first real transaction:
                // - Clears buffers, resets state to IDLE
                // - Arms TX and RX DMA for pull mode (hybrid RXNE + DMA)
                // - If mode is PUSH, spi_slave_tick() will later detect data and prepare push transactions
                spi_slave_arm();
            }
            break;

        case SPI_STATE_IDLE:
            // NSS rose while idle. Could be a glitch, or the master sent
            // a radio TX message while no push data was queued.
            // Since arm_push_idle() now arms RX DMA, check if master sent data.
            {
                uint16_t rx_remaining = DMA1_Channel1->CNDTR;
                uint16_t rx_received = MAX_TRANSACTION_SIZE - rx_remaining;

                if (rx_received >= PUSH_RADIO_PAYLOAD && spi_slave_rx_has_valid_data()) {
                    if (ctx.radio_tx_queue) {
                        radio_message_enqueue(PUSH_RADIO_PAYLOAD, ctx.rx_buf, ctx.radio_tx_queue);
#ifdef DEBUG
                        debug_uart_log_spi_radio_tx(ctx.rx_buf, PUSH_RADIO_PAYLOAD);
                        debug_uart_log_spi_push_event("IDLE-RX", 0, 0, rx_received);
#endif
                        ctx.master_tx_received++;
                    }
                }
#ifdef DEBUG
                else if (rx_received > 0) {
                    debug_uart_log_spi_push_event("IDLE-GLITCH", 0, 0, rx_received);
                }
#endif
            }
            // Re-arm happens at bottom of handler via arm_push_idle()
            break;

        case SPI_STATE_HAVE_DATA:
            // ══════════════════════════════════════════════════════════════
            // PUSH MODE TRANSACTION COMPLETE
            // ══════════════════════════════════════════════════════════════
            // Sequence of events:
            // 1. We had data (GPS fix or radio message)
            // 2. spi_slave_prepare_push() loaded tx_buf and asserted IRQ
            // 3. Master saw IRQ, asserted NSS, clocked data out
            // 4. Master released NSS (rising edge) → this interrupt fires
            //
            // Now we finalize the push transaction:
            // - Verify FULL message was transmitted (CRITICAL for data integrity)
            // - Deassert IRQ ONLY if transmission complete
            // - Pop message from queue ONLY if transmission complete
            // - If incomplete: Keep IRQ high, keep message in queue (automatic retry)
            // - Check if master sent data back simultaneously (enqueue if yes)

            // ── Check How Many Bytes We Transmitted (TX DMA Verification) ──
            // CNDTR (Channel Number of Data to Transfer Register) counts DOWN from initial length to 0.
            // If CNDTR > 0, the DMA didn't finish (master ended transaction early).
            // If CNDTR = 0, all bytes were sent successfully.
            //
            // Example for PUSH_GPS_TOTAL (49 bytes):
            // - Initial: CNDTR = 49, tx_sent = 0
            // - After 1 byte: CNDTR = 48, tx_sent = 1
            // - After 49 bytes: CNDTR = 0, tx_sent = 49 (success!)
            //
            // CRITICAL: We require the FULL message to be transmitted (tx_sent >= tx_dma_length).
            // Partial transmission = incomplete message = data loss if we dequeue.
            // Solution: Keep IRQ high, keep message in queue, master will retry.
            {
                uint16_t tx_remaining = DMA1_Channel2->CNDTR;
                uint16_t tx_sent = ctx.tx_dma_length - tx_remaining;

                // Store for diagnostics
                ctx.tx_dma_sent = tx_sent;

                // ── Verify TX Completion ──
                if (tx_sent >= ctx.tx_dma_length) {
                    // ✅ SUCCESS: Full message transmitted

                    // ── Check for Simultaneous Master TX (before pop/deassert) ──
                    // While we pushed data, the master might have sent a radio TX
                    // message simultaneously. If it did, the master was NOT reading
                    // our push data — it was sending. We must:
                    // 1. Enqueue the master's radio TX
                    // 2. NOT pop our push message (master didn't consume it)
                    // 3. Re-arm push with same data so master can read it next time
                    uint16_t rx_remaining = DMA1_Channel1->CNDTR;
                    uint16_t rx_received = MAX_TRANSACTION_SIZE - rx_remaining;

                    if (rx_received >= PUSH_RADIO_PAYLOAD && spi_slave_rx_has_valid_data()) {
                        // Master sent radio TX — push data NOT consumed by master
                        if (ctx.radio_tx_queue) {
                            radio_message_enqueue(PUSH_RADIO_PAYLOAD, ctx.rx_buf, ctx.radio_tx_queue);
#ifdef DEBUG
                            debug_uart_log_spi_radio_tx(ctx.rx_buf, PUSH_RADIO_PAYLOAD);
                            debug_uart_log_spi_push_event("NOT-YET-READ", ctx.pending_push_type,
                                                          tx_sent, rx_received);
#endif
                            ctx.master_tx_received++;
                        }
                        // Re-arm push with same data (tx_buf still has it from prepare_push)
                        // IRQ stays asserted, pending_push_type stays set
                        spi_slave_arm_push();
                        push_handled = true;
                    } else {
                        // Normal push read — master consumed our data
#ifdef DEBUG
                        debug_uart_log_spi_push_event("COMPLETE", ctx.pending_push_type,
                                                      tx_sent, rx_received);
#endif
                        spi_slave_deassert_irq();

                        // Remove the message from the queue
                        if (ctx.pending_push_type == PUSH_TYPE_RADIO && ctx.radio_queue) {
#ifdef DEBUG
                            debug_uart_log_spi_radio_read(&ctx.tx_buf[PUSH_TYPE_BYTES], tx_sent - PUSH_TYPE_BYTES);
#endif
                            radio_message_queue_pop(ctx.radio_queue);
                        } else if (ctx.pending_push_type == PUSH_TYPE_GPS && ctx.gps_fix_queue) {
#ifdef DEBUG
                            debug_uart_log_spi_gps_read(&ctx.tx_buf[PUSH_TYPE_BYTES], tx_sent - PUSH_TYPE_BYTES);
#endif
                            gps_fix_queue_pop(ctx.gps_fix_queue);
                        }

                        ctx.pending_push_type = 0;
                        ctx.push_transactions++;
                    }

                } else {
                    // ❌ INCOMPLETE: Not enough bytes transmitted
                    //
                    // Master ended transaction early (two-phase read phase 1, or glitch).
                    // Keep IRQ high, keep message in queue, master will come back for phase 2.
                    ctx.tx_incomplete_count++;
                    push_incomplete = true;
#ifdef DEBUG
                    {
                        uint16_t rx_remaining_inc = DMA1_Channel1->CNDTR;
                        uint16_t rx_received_inc = MAX_TRANSACTION_SIZE - rx_remaining_inc;
                        debug_uart_log_spi_push_event("INCOMPLETE", ctx.pending_push_type,
                                                      tx_sent, rx_received_inc);
                    }
#endif
                }
            }

            // ── Clear Pending Push Type ──
            // Skip on incomplete push (two-phase phase 1) and on push_handled
            // (not-yet-read: master sent radio TX, push data kept for retry).
            if (!push_incomplete && !push_handled) {
                ctx.pending_push_type = 0;
                ctx.transactions_completed++;
            }
            break;

        case SPI_STATE_ACTIVE:
            // ══════════════════════════════════════════════════════════════
            // PULL MODE TRANSACTION COMPLETE
            // ══════════════════════════════════════════════════════════════
            // Sequence of events:
            // 1. Master asserted NSS, sent command byte
            // 2. RXNE interrupt fired, we read command byte, patched tx_buf, enabled RXDMAEN
            // 3. DMA handled remaining bytes (dummy + payload)
            // 4. Master released NSS (rising edge) → this interrupt fires
            //
            // Now we finalize the pull transaction:
            // - For READ commands (CMD_RADIO_RX_LIFO/FIFO, CMD_GPS_RX): Pop message from queue
            // - For WRITE commands (CMD_RADIO_TX): Enqueue received payload
            // - Check for SPI errors (overrun)

            // ── Handle Write Commands ──
            // For write commands, master sends data TO us (e.g., CMD_RADIO_TX sends 256 bytes).
            // We need to check if we received the full payload and enqueue it.
            //
            // WHY check payload_processed flag?
            // - The DMA TC interrupt might have already processed the payload (if DMA finished before NSS rose)
            // - This handler might also fire if master ends transaction early (before DMA TC)
            // - payload_processed prevents double-enqueueing the same message
            if (ctx.current_cmd == CMD_RADIO_TX && !ctx.payload_processed) {
                // ── Calculate How Many Payload Bytes We Received ──
                // RX DMA was armed for MAX_TRANSACTION_SIZE - 1 bytes (260 bytes, skipping command byte).
                // Layout: [DUMMY:4][PAYLOAD:256]
                //
                // CNDTR counts down: 260 → 259 → ... → 0 as bytes arrive.
                // Bytes received = initial_count - remaining = 260 - CNDTR
                uint16_t remaining = DMA1_Channel1->CNDTR;
                uint16_t total_rx_bytes = (MAX_TRANSACTION_SIZE - 1) - remaining;

                // total_rx_bytes includes DUMMY bytes (4) + PAYLOAD bytes (0-256).
                // We only care about the PAYLOAD bytes.
                //
                // Example for full transaction:
                // - total_rx_bytes = 260 (4 dummy + 256 payload)
                // - payload_bytes = 260 - 4 = 256 ✓
                //
                // Example for short transaction (master aborted early):
                // - total_rx_bytes = 10 (4 dummy + 6 payload)
                // - payload_bytes = 10 - 4 = 6 (partial)
                uint16_t payload_bytes = 0;
                if (total_rx_bytes > PULL_DUMMY_BYTES) {
                    payload_bytes = total_rx_bytes - PULL_DUMMY_BYTES;
                }

                if (payload_bytes == 256) {
                    // ── Full Payload Received ──
                    // Master sent all 256 bytes. Enqueue to radio queue for transmission.
                    //
                    // Payload location: rx_buf[CMD_OVERHEAD] = rx_buf[5]
                    // - rx_buf[0] = command byte (0x04 = CMD_RADIO_TX)
                    // - rx_buf[1..4] = dummy bytes (ignored)
                    // - rx_buf[5..260] = 256-byte radio message
                    //
                    // WHY check if DMA TC already processed it?
                    // - If DMA TC fired before NSS EXTI, payload_processed = true
                    // - This if-block would be skipped, preventing double-enqueue
                    // - If NSS EXTI fires first (short transaction), we process it here
                    if (ctx.radio_tx_queue) {
                        radio_message_enqueue(256, &ctx.rx_buf[CMD_OVERHEAD], ctx.radio_tx_queue);
#ifdef DEBUG
                        // Log to debug UART if in Debug build
                        debug_uart_log_spi_radio_tx(&ctx.rx_buf[CMD_OVERHEAD], 256);
#endif
                        ctx.payload_processed = true;  // Mark as processed
                    }
                } else if (payload_bytes > 0) {
                    // ── Partial Payload Received ──
                    // Master ended transaction early (NSS rose before sending all 256 bytes).
                    // This is abnormal - could be a master bug, electrical glitch, or intentional abort.
                    //
                    // Options:
                    // 1. Enqueue partial message (risky - radio might not handle partial frames)
                    // 2. Discard and increment error counter (safe - we're doing this)
                    // 3. Log for debugging (future enhancement)
                    //
                    // For now, we silently discard partial payloads.
                    // The transaction will be re-armed and master can retry.
                }
                // else: payload_bytes = 0 (only dummy bytes received, master sent no payload)
            }

            // ── Handle Read Commands ──
            // For read commands, we already sent data to master via TX DMA.
            // Now pop the message from the queue since master has received it.
            //
            // WHY pop in EXTI handler (not in RXNE ISR or DMA TC)?
            // - RXNE ISR: Too early - transaction hasn't completed, master might abort
            // - DMA TC: Fires when DMA finishes (260 bytes), but master might have ended early
            // - NSS EXTI: Fires when master releases NSS (authoritative transaction end)
            //
            // We verify the full payload was transmitted before popping. If the
            // master ended early, the message stays in the queue for retry.
            {
                uint16_t tx_remaining = DMA1_Channel2->CNDTR;
                uint16_t tx_sent = MAX_TRANSACTION_SIZE - tx_remaining;

                if (ctx.current_cmd == CMD_RADIO_RX_FIFO) {
                    if (tx_sent >= PULL_RADIO_TOTAL && ctx.radio_queue) {
#ifdef DEBUG
                        debug_uart_log_spi_radio_read(&ctx.tx_buf[CMD_OVERHEAD], PULL_RADIO_PAYLOAD);
#endif
                        radio_message_queue_pop(ctx.radio_queue);
                    }
                } else if (ctx.current_cmd == CMD_RADIO_RX_LIFO) {
                    if (tx_sent >= PULL_RADIO_TOTAL && ctx.radio_queue) {
#ifdef DEBUG
                        debug_uart_log_spi_radio_read(&ctx.tx_buf[CMD_OVERHEAD], PULL_RADIO_PAYLOAD);
#endif
                        radio_message_queue_pop_lifo(ctx.radio_queue);
                    }
                } else if (ctx.current_cmd == CMD_GPS_RX) {
                    // GPS: Do NOT pop — master always receives latest NMEA sentence.
                    // The double-buffer queue overwrites on new data, so repeated reads
                    // return the same sentence until a new one arrives from the GPS module.
                    if (tx_sent >= PULL_GPS_TOTAL && ctx.gps_queue) {
#ifdef DEBUG
                        debug_uart_log_spi_gps_read(&ctx.tx_buf[CMD_OVERHEAD], PULL_GPS_PAYLOAD);
#endif
                    }
                }
            }

            // ── Check for SPI Overrun Errors ──
            // OVR (Overrun) flag is set when:
            // - RX FIFO is full (4 bytes)
            // - New byte arrives on MOSI
            // - Old byte is lost (overwritten)
            //
            // This indicates the DMA couldn't keep up with the SPI clock rate.
            // Causes: DMA priority too low, AHB bus stalled, RXDMAEN disabled too long, etc.
            //
            // We increment a diagnostic counter but don't abort - the transaction is already done.
            if (SPI1->SR & SPI_SR_OVR) {
                ctx.overrun_errors++;
            }

            // ── Increment Total Transaction Counter ──
            ctx.transactions_completed++;
            break;

        default:
            // Unexpected state
            break;
        }

        // ── Re-arm for next transaction ──
        // PROTOCOL MODE ENFORCEMENT (STRICT MODE):
        // After any transaction completes, we re-arm based on the configured protocol mode.
        // This ensures:
        // - PUSH mode stays armed for push (strict enforcement, no pull commands accepted)
        // - PULL mode stays armed for pull (command-based transactions only)
        //
        // s_protocol_mode is set during startup configuration frame reception:
        // - 0x00 = SPI_MODE_PULL: Master-initiated, command-based protocol
        // - 0x01 = SPI_MODE_PUSH: Slave-initiated via IRQ assertion
        //
        // WHY strict mode enforcement?
        // - Prevents mode mixing (slave responding to commands while configured for push)
        // - Ensures consistent behavior: PUSH mode = only push, PULL mode = only pull
        // - Avoids deadlock: In push mode, RXNEIE is not enabled, so pull commands would hang
        // ── Re-arm SPI for next transaction ──
        // Skip re-arm when:
        // - push_incomplete: DMA/FIFO still valid for two-phase read phase 2
        // - push_handled: not-yet-read path already called arm_push() to re-arm
        if (!push_incomplete && !push_handled) {
            if (s_protocol_mode == SPI_MODE_PUSH) {
                spi_slave_arm_push_idle();
            } else {
                spi_slave_arm();
            }
        }

    }  // End of if (rising_pending)

    // Note: If neither falling nor rising edge was pending (shouldn't happen),
    // we simply return. This handles the case where multiple EXTI lines share
    // this handler but we only configured line 4.
}

// ============================================================================
// PUBLIC API IMPLEMENTATION
// ============================================================================

/**
 * @brief Initialize SPI slave peripheral
 *
 * Configures SPI1 as slave with hardware NSS, sets up DMA channels,
 * and arms for operation based on the protocol mode set by
 * spi_slave_set_protocol_mode().
 */
void spi_slave_init(radio_message_queue_t *radio_queue, radio_message_queue_t *radio_tx_queue, gps_sample_queue_t *gps_queue, gps_fix_queue_t *gps_fix_queue) {
    // Clear context structure
    memset(&ctx, 0, sizeof(ctx));

    // Store queue pointers
    ctx.radio_queue = radio_queue;
    ctx.radio_tx_queue = radio_tx_queue;
    ctx.gps_queue = gps_queue;            // Pull mode: raw NMEA
    ctx.gps_fix_queue = gps_fix_queue;    // Push mode: parsed GPS fixes

    // ── Enable peripheral clocks ──
    RCC->IOPENR  |= RCC_IOPENR_GPIOBEN;    // GPIO port B
    RCC->AHBENR  |= RCC_AHBENR_DMA1EN;     // DMA1
    RCC->APBENR2 |= RCC_APBENR2_SPI1EN;    // SPI1 (on APB2, not APB1!)

    // ── Initialize peripherals ──
    spi_gpio_init();
    dmamux_init();
    spi_peripheral_init();
    exti_nss_init();

    // ── Enable interrupts in NVIC ──
    // SPI1 RXNE interrupt (highest priority - must respond fast)
    NVIC_SetPriority(SPI1_IRQn, 0);
    NVIC_EnableIRQ(SPI1_IRQn);

    // DMA1 Channel 1 interrupt (high priority)
    NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    // EXTI already enabled in exti_nss_init()

    // ── Arm for normal operation based on configured mode ──
    // The protocol mode must have been set via spi_slave_set_protocol_mode()
    // before calling this function (typically after receiving config frame).
    if (s_protocol_mode == SPI_MODE_PUSH) {
        // Push mode: Arm in push-ready idle state
        spi_slave_arm_push_idle();
    } else {
        // Pull mode: Arm for command-based transactions (default)
        spi_slave_arm();
    }
}

/**
 * @brief Get current state (for debugging)
 */
spi_slave_state_t spi_slave_get_state(void) {
    return ctx.state;
}

/**
 * @brief Get context pointer (for debugging)
 */
const spi_slave_context_t* spi_slave_get_context(void) {
    return &ctx;
}

/**
 * @brief Reset error counters
 */
void spi_slave_reset_errors(void) {
    ctx.overrun_errors = 0;
    ctx.transfer_errors = 0;
    ctx.unknown_commands = 0;
    ctx.master_tx_received = 0;
}

#ifdef DEBUG
const spi_debug_capture_t* spi_slave_get_debug_capture(void) {
    return (const spi_debug_capture_t*)&s_dbg;
}
#endif

/**
 * @brief Set the protocol mode (pull or push)
 *
 * This function sets the static protocol mode variable that determines
 * how the SPI slave operates. Must be called after receiving and parsing
 * the configuration frame from the master.
 *
 * @param mode SPI_MODE_PULL (0x00) or SPI_MODE_PUSH (0x01)
 */
void spi_slave_set_protocol_mode(spi_protocol_mode_t mode) {
    s_protocol_mode = mode;
}

// ============================================================================
// PUSH MODE IMPLEMENTATION
// ============================================================================

/**
 * @brief Assert IRQ line to master (active high)
 *
 * Sets PB1 high to signal the master that data is ready in push mode.
 *
 * PUSH MODE PROTOCOL:
 * ───────────────────
 * 1. Slave has data (GPS fix or radio message)
 * 2. Slave prepares TX buffer with [TYPE:1][PAYLOAD:N]
 * 3. Slave asserts IRQ (this function) → PB1 goes HIGH
 * 4. Master detects IRQ rising edge (via EXTI or polling)
 * 5. Master asserts NSS and clocks data out
 * 6. Slave deasserts IRQ when transaction completes
 *
 * IRQ SIGNAL CHARACTERISTICS:
 * ───────────────────────────
 * - Active HIGH (not active low!)
 * - Push-pull output (not open-drain)
 * - Driven by GPIO (not by SPI peripheral)
 * - Low = idle (no data), High = data ready
 *
 * WHY ACTIVE HIGH?
 * ────────────────
 * - Simpler than active low (no external pull-up needed)
 * - Master can detect rising edge easily with EXTI
 * - GPIO defaults to low on reset (safe default)
 */
void spi_slave_assert_irq(void) {
    // ── Use BSRR to Set Pin High ──
    // BSRR (Bit Set/Reset Register) allows atomic set/reset without read-modify-write.
    // This prevents race conditions (e.g., ISR changing pin while we're reading ODR).
    //
    // BSRR Register Layout (32 bits):
    // ┌──────────────────┬──────────────────┐
    // │ Bits [31:16]     │ Bits [15:0]      │
    // │ BR (Bit Reset)   │ BS (Bit Set)     │
    // └──────────────────┴──────────────────┘
    //
    // Writing 1 to BS[n] (bits [15:0]): Sets pin n HIGH
    // Writing 1 to BR[n] (bits [31:16]): Resets pin n LOW
    // Writing 0 to any bit: No effect
    //
    // Example for PB1 (IRQ_GPIO_PIN = GPIO_PIN_1 = 0x0002 = bit 1):
    // - IRQ_GPIO_PIN = 0x0002 (bit 1 set)
    // - Writing to BSRR: GPIOB->BSRR = 0x0002
    // - Bits [15:0] = 0x0002 → bit 1 of BS → Sets PB1 HIGH
    //
    // WHY BSRR instead of ODR?
    // - ODR (Output Data Register) requires read-modify-write: ODR |= (1 << 1)
    // - If an interrupt fires between read and write, the change could be lost
    // - BSRR is atomic - no race condition, one write operation
    GPIOB->BSRR = IRQ_GPIO_PIN;  // Write bit 1 to BS field → PB1 = HIGH
    ctx.irq_asserted = true;
}

/**
 * @brief Deassert IRQ line to master (inactive low)
 *
 * Sets PB1 low to signal the master that the push transaction is complete.
 *
 * WHEN TO DEASSERT?
 * ─────────────────
 * - After NSS rising edge (transaction complete)
 * - Before checking for more data to push
 * - CRITICAL: Must deassert BEFORE asserting again, or master sees continuous IRQ
 */
void spi_slave_deassert_irq(void) {
    // ── Use BSRR to Reset Pin Low ──
    // To reset a pin (drive low), we write to the BR field (bits [31:16]).
    //
    // Example for PB1 (IRQ_GPIO_PIN = GPIO_PIN_1 = 0x0002 = bit 1):
    // - We want to write to BR[1] (bit 17 of BSRR)
    // - IRQ_GPIO_PIN << 16 = 0x0002 << 16 = 0x00020000 (bit 17 set)
    // - Writing to BSRR: GPIOB->BSRR = 0x00020000
    // - Bits [31:16] = 0x0002 → bit 1 of BR → Resets PB1 LOW
    //
    // WHY shift by 16?
    // - IRQ_GPIO_PIN is the bit position in the lower half (BS field)
    // - To access the upper half (BR field), we shift left by 16 bits
    // - IRQ_GPIO_PIN << 16 moves bit 1 to bit 17 (BR[1])
    GPIOB->BSRR = IRQ_GPIO_PIN << 16;  // Write bit 1 to BR field → PB1 = LOW
    ctx.irq_asserted = false;
}

/**
 * @brief Check if RX buffer contains valid data (not dummy bytes)
 *
 * Used in PUSH MODE to detect if master sent real data (radio TX message)
 * while we were pushing data to it (simultaneous TX/RX).
 *
 * THE PROBLEM:
 * ────────────
 * In push mode, both TX and RX DMA are enabled. While we're sending data to master,
 * master might:
 * 1. Just clock SCK to receive our data (sends dummy bytes: 0x00 or 0xFF)
 * 2. Send a radio TX message simultaneously (sends real 256-byte payload)
 *
 * We need to distinguish case 1 (ignore) from case 2 (enqueue radio message).
 *
 * THE SOLUTION:
 * ─────────────
 * Sample the first RX_PATTERN_SAMPLE_SIZE (16) bytes of the RX buffer.
 * Count how many are 0x00 and how many are 0xFF.
 * - If ≥14 out of 16 bytes are 0x00 or 0xFF → dummy data (ignore)
 * - If <14 out of 16 bytes are 0x00 or 0xFF → real data (enqueue)
 *
 * WHY THIS WORKS:
 * ───────────────
 * - Dummy bytes: Master sends all 0x00 (common) or all 0xFF (SPI idle state)
 *   - 16 dummy bytes would have 16 zeros or 16 ones → fails threshold
 * - Real radio messages: Typically have varied content (packet headers, data, CRC)
 *   - 16 bytes of real data would have <14 zeros and <14 ones → passes threshold
 *
 * EDGE CASES:
 * ───────────
 * - What if radio message starts with 14+ bytes of 0x00?
 *   - Rare in practice (most protocols have headers, sync words, etc.)
 *   - Could improve detection by checking entropy or specific patterns
 * - What if master sends mixed dummy + real data?
 *   - If master sends <256 bytes, we might miss the real data
 *   - This is a protocol violation - master should send full messages
 *
 * TUNING PARAMETERS:
 * ──────────────────
 * - RX_PATTERN_SAMPLE_SIZE = 16: Number of bytes to sample
 * - RX_PATTERN_THRESHOLD = 14: Minimum zeros/ones to consider it dummy data
 * - Adjust if false positives (dummy detected as real) or false negatives (real detected as dummy)
 *
 * @return true if RX buffer appears to contain real radio message data
 * @return false if RX buffer appears to contain dummy bytes (0x00 or 0xFF)
 */
bool spi_slave_rx_has_valid_data(void) {
    uint8_t zeros = 0;
    uint8_t ones = 0;

    // ── Sample the First N Bytes ──
    // We don't check all 256 bytes because:
    // 1. Performance: Checking 256 bytes takes too long in an ISR
    // 2. Early detection: First 16 bytes are usually enough to distinguish
    // 3. Dummy data is uniform: If first 16 bytes are 0x00, rest are likely 0x00 too
    for (int i = 0; i < RX_PATTERN_SAMPLE_SIZE; i++) {
        if (ctx.rx_buf[i] == 0x00) zeros++;
        if (ctx.rx_buf[i] == 0xFF) ones++;
    }

    // ── Apply Threshold Test ──
    // If zeros ≥ 14: Most bytes are 0x00 → dummy data
    // If ones ≥ 14: Most bytes are 0xFF → dummy data
    // If both < 14: Bytes are varied → real data
    //
    // Example 1: Dummy data (all zeros)
    //   zeros = 16, ones = 0
    //   zeros < 14? NO → return false (dummy)
    //
    // Example 2: Real data (packet: [0xAA, 0x55, 0x01, 0x02, ...])
    //   zeros = 2, ones = 0
    //   zeros < 14? YES, ones < 14? YES → return true (real)
    //
    // Example 3: Edge case (14 zeros, 2 non-zeros)
    //   zeros = 14, ones = 0
    //   zeros < 14? NO → return false (classified as dummy, might be false negative)
    return (zeros < RX_PATTERN_THRESHOLD && ones < RX_PATTERN_THRESHOLD);
}

/**
 * @brief Periodic tick function for push mode
 *
 * Call this from the main loop at regular intervals (e.g., every 1-10 ms).
 * When data is available and the slave is idle, this function:
 * 1. Checks if GPS or radio queue has pending data
 * 2. Prepares TX buffer with [TYPE:1][PAYLOAD:N]
 * 3. Arms TX and RX DMA
 * 4. Asserts IRQ to notify master
 *
 * WHEN TO CALL THIS?
 * ──────────────────
 * - In PUSH mode: Call this continuously from main loop
 * - In PULL mode: Not needed (master initiates all transactions)
 *
 * PUSH MODE TIMING:
 * ─────────────────
 * - GPS fixes arrive every 100-1000 ms (1-10 Hz)
 * - Radio messages arrive asynchronously (bursty traffic)
 * - Calling this every 1-10 ms ensures low latency (<10 ms from data arrival to IRQ assertion)
 *
 * PRIORITY: GPS > Radio
 * ──────────────────────
 * WHY prioritize GPS?
 * - GPS payloads are smaller (49 bytes vs 257 bytes) → faster to transmit
 * - GPS data is time-sensitive (position fix should be transmitted ASAP)
 * - Radio messages can tolerate slightly higher latency (buffered in queue)
 * - If both queues have data, push GPS first, then radio on next tick
 */
void spi_slave_tick(void) {
    // ── Guard: Only operate in PUSH mode ──
    // In PULL mode, the master initiates all transactions via commands.
    // Calling prepare_push() in PULL mode would override the pull-mode arm
    // configuration (disabling RXNEIE, enabling TXDMAEN/RXDMAEN), causing
    // the slave to send push data instead of responding to pull commands.
    if (s_protocol_mode != SPI_MODE_PUSH) {
        return;
    }

#ifdef DEBUG
    // ── Periodic Stats & Tick Diagnostics ──
    {
        static uint32_t last_stats_tick = 0;
        uint32_t now = HAL_GetTick();
        if (now - last_stats_tick >= 2000) {
            last_stats_tick = now;
            debug_uart_log_spi_stats(ctx.push_transactions,
                                      ctx.transactions_completed,
                                      ctx.tx_incomplete_count,
                                      ctx.master_tx_received,
                                      ctx.overrun_errors);
            // Show tick guard state so we can see WHY IRQ isn't asserting
            bool gps_has = (ctx.gps_fix_queue && !gps_fix_queue_empty(ctx.gps_fix_queue));
            bool radio_has = (ctx.radio_queue && !radio_message_queue_empty(ctx.radio_queue));
            uint8_t nss_high = (GPIOA->IDR & GPIO_PIN_4) ? 1 : 0;
            debug_uart_log_spi_push_event("TICK_STATE", (uint8_t)ctx.state,
                                           (uint16_t)((gps_has << 1) | radio_has),
                                           (uint16_t)((ctx.irq_asserted << 1) | nss_high));
        }
    }
#endif

    // ── Guard: Only Process if Idle ──
    // If we're mid-transaction (ACTIVE, HAVE_DATA, or UNCONFIGURED), don't start a new push.
    // Starting a push while a transaction is active would:
    // - Corrupt the ongoing transaction (DMA channels already running)
    // - Cause race conditions (TX buffer being modified while DMA reads it)
    // - Assert IRQ while NSS is low (protocol violation)
    //
    // WHY check state != IDLE?
    // - UNCONFIGURED: Waiting for config frame, can't push yet
    // - IDLE: Ready for new transaction ✓
    // - HAVE_DATA: Already pushed, waiting for master to respond
    // - ACTIVE: Pull mode transaction in progress
    if (ctx.state != SPI_STATE_IDLE) {
        return;
    }

    // ── Guard: Don't Push if IRQ Already Asserted ──
    // If IRQ is already high, we've already prepared a push transaction.
    // We're just waiting for master to respond (assert NSS and clock data out).
    //
    // WHY check irq_asserted?
    // - Prevents double-preparation (wasting CPU cycles)
    // - Ensures we wait for master to finish current push before starting next one
    // - Prevents IRQ line from toggling (which could confuse master's edge detector)
    if (ctx.irq_asserted) {
        return;
    }

    // ── CRITICAL GUARD: Check if NSS is Currently Low (Transaction Active) ──
    // RACE CONDITION PREVENTION:
    // If master has already asserted NSS (NSS = low), a transaction is in progress.
    // Calling spi_slave_prepare_push() now would:
    // 1. Call spi_slave_arm_push() which disables SPI during active transaction
    // 2. Clear RX FIFO while MOSI data is arriving (data loss)
    // 3. Modify TX buffer while DMA is reading from it (corruption)
    // 4. Assert IRQ while NSS is already low (protocol violation)
    //
    // GPIO Pin State Reading:
    // - GPIOA->IDR (Input Data Register) reads the current logic level of GPIO pins
    // - GPIO_PIN_4 = PA4 (NSS pin)
    // - If (IDR & GPIO_PIN_4) == 0, NSS is LOW → transaction active
    // - If (IDR & GPIO_PIN_4) != 0, NSS is HIGH → bus idle
    //
    // Solution: If NSS is low, defer push to next tick (after current transaction completes)
    if ((GPIOA->IDR & GPIO_PIN_4) == 0) {
        // NSS is low = transaction in progress
        // Increment diagnostic counter
        ctx.nss_collisions++;
        return;  // Defer push to next tick
    }

    // ── Priority 1: Check for GPS Data ──
    // GPS data has higher priority because:
    // - Smaller payload (49 bytes) → faster to transmit, less SPI bus time
    // - Time-sensitive (position fix should be delivered ASAP for navigation)
    // - Less frequent (1-10 Hz) compared to potential radio traffic bursts
    //
    // If GPS queue has data, prepare and push it immediately.
    if (ctx.gps_fix_queue && !gps_fix_queue_empty(ctx.gps_fix_queue)) {
        spi_slave_prepare_push(PUSH_TYPE_GPS);
        return;  // Exit early - push GPS first, check radio on next tick
    }

    // ── Priority 2: Check for Radio Data ──
    // Radio messages are pushed if GPS queue is empty.
    // Radio payloads are larger (257 bytes) but can tolerate slightly higher latency.
    //
    // If radio queue has data, prepare and push it.
    if (ctx.radio_queue && !radio_message_queue_empty(ctx.radio_queue)) {
        spi_slave_prepare_push(PUSH_TYPE_RADIO);
        return;
    }

    // ── No Data to Push ──
    // Both queues are empty. Do nothing and return.
    // Next tick will check again (polling loop).
}
