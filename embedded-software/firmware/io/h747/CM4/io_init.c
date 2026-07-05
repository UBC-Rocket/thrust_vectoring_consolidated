/**
 * @file    io_init.c
 * @brief   IO-layer phase for CM4. Brings up transport services in the
 *          right order. Scheduler hand-off lives in app_start_kernel
 *          (app/cm4/app_init_cm4.c).
 *
 * UBC Rocket, 2026
 */
#include "io_sys/io_init.h"
#include "io_sys/io_sd.h"
#include "io_sys/io_status_led.h"
#include "io_sys/io_timestamp.h"
#include "io_sys/io_intercore.h"

#include "usart.h"          /* TEMP diagnostic: all CM4 UART handles */
#include "stm32h7xx_hal.h"  /* HAL_Delay for the boot LED-identify blink */
#include <string.h>

extern void io_spi_init(void);   /* declared in io_spi.c */

/* TEMP diagnostic helper (remove after bring-up): force one UART to 115200 and
 * send a labelled probe line, so a host serial terminal reads a clean label
 * regardless of the UART's configured baud. Used to find which UART (if any)
 * is physically wired to the STLINK-V3 VCP. */
static void io_diag_probe_uart(UART_HandleTypeDef *h, const char *msg) {
    h->Init.BaudRate = 115200U;
    if (HAL_UART_Init(h) != HAL_OK) {
        return;
    }
    (void)HAL_UART_Transmit(h, (uint8_t *)msg, (uint16_t)strlen(msg), 100U);
}

void io_init_cm4(void) {
    io_timestamp_init();
    io_status_led_init();
    io_intercore_init();

    io_spi_init();      /* per-bus spi_queue instances + dispatch table */
    io_sd_init();
    /* io_uart_open for streaming UARTs happens lazily from each driver.
     * IO_UART_SERVO_BUS (UART8) is used on-demand by the servo drivers. */

    /* ===================== TEMP BRING-UP DIAGNOSTIC =====================
     * Remove this whole block after diagnosis.
     *
     * (a) LED identify: blink all three firmware LEDs in unison 3× so we can
     *     tell which LEDs are ours vs. power/STLINK LEDs.
     * (b) All-UART VCP probe: send a labelled line on EVERY CM4 UART (forced
     *     to 115200) here in io_init — before dev_init — so a stalled sensor
     *     can't suppress it. Whichever "PROBE <name>" line appears in the
     *     serial terminal identifies the UART wired to the STLINK-V3 VCP.
     *     Nothing on any ⇒ the VCP isn't wired to a board UART (needs a jumper
     *     or a USB-UART adapter).
     * (c) Staged boot tracer: GREEN here = io_init done; dev_init lights
     *     YELLOW; app_init lights RED solid; mission_manager blinks RED. */
    for (int i = 0; i < 3; ++i) {
        io_status_led_set(IO_LED_GREEN,  true);
        io_status_led_set(IO_LED_YELLOW, true);
        io_status_led_set(IO_LED_RED,    true);
        HAL_Delay(150);
        io_status_led_set(IO_LED_GREEN,  false);
        io_status_led_set(IO_LED_YELLOW, false);
        io_status_led_set(IO_LED_RED,    false);
        HAL_Delay(150);
    }

    io_diag_probe_uart(&hlpuart1, "PROBE LPUART1_PA9\r\n");
    io_diag_probe_uart(&huart1,   "PROBE USART1_PB14\r\n");
    io_diag_probe_uart(&huart3,   "PROBE USART3_PD8\r\n");
    io_diag_probe_uart(&huart6,   "PROBE USART6_PC6\r\n");
    io_diag_probe_uart(&huart4,   "PROBE UART4_PD1\r\n");
    io_diag_probe_uart(&huart5,   "PROBE UART5_PB6\r\n");
    io_diag_probe_uart(&huart7,   "PROBE UART7_PE8\r\n");

    /* GREEN solid = io_init_cm4() reached its end. */
    io_status_led_set(IO_LED_GREEN, true);
    /* ================== END TEMP BRING-UP DIAGNOSTIC ==================== */
}
