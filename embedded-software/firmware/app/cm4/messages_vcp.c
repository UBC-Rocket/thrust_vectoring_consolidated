/**
 * @file    messages_vcp.c
 * @brief   Bridge between the debug UART (io_uart) and the messages runtime.
 *
 * Provides the receive path (UART RX → COBS decode → messages_handle_inbound)
 * and the transmit sink (messages publish/response → COBS encode → io_uart_send)
 * for the messages registry's "vcp" channel.
 *
 * UART: IO_UART_DEBUG → hlpuart1 (LPUART1, 115200 baud, match_byte=0x00 for
 * COBS framing). Full BDMA RX (circular) + NVIC IRQ handled by the LPUART1
 * IRQHandler glue in stm32h7xx_it.c.
 *
 * UBC Rocket, 2026
 */

#include "app/messages_vcp.h"
#include "io/io_uart.h"
#include "messages/messages.h"
#include "cobs/cobs.h"
#include "generated/messages/registry.h"

#include <string.h>

#define VCP_UART  (&IO_UART_DEBUG)

/* Per-frame scratch. COBS overhead is ceil(N/254)+1; for our 256-byte
 * payload + envelope budget, the encoded frame stays comfortably under
 * 300 bytes. Add one for the trailing 0x00 delimiter. */
#define VCP_FRAME_SCRATCH_SIZE  (COBS_ENCODE_MAX(300U) + 1U)

static uint8_t s_decode_buf[300];
static uint8_t s_encode_buf[VCP_FRAME_SCRATCH_SIZE];

/* RX callback fired from io_uart on each character-match (0x00 = COBS
 * frame boundary). `data` includes the trailing 0x00; we strip it
 * before COBS-decoding. */
static void vcp_on_rx(const uint8_t *data, size_t len, void *user)
{
    (void)user;
    if (len < 2U) {
        /* Lone 0x00 (idle) or noise — skip. */
        return;
    }
    /* Strip the trailing 0x00 delimiter the IRQ left in the buffer. */
    size_t enc_len = (data[len - 1U] == 0U) ? (len - 1U) : len;
    size_t dec_len = cobs_decode(data, enc_len, s_decode_buf, sizeof(s_decode_buf));
    if (dec_len == 0U) {
        return; /* malformed frame; quietly drop */
    }
    (void)messages_handle_inbound(CH_VCP, s_decode_buf, dec_len);
}

/* TX sink the messages runtime calls for any record routed to CH_VCP. */
static bool vcp_sink(const uint8_t *record, size_t record_len)
{
    size_t enc_len = cobs_encode(record, record_len, s_encode_buf,
                                  sizeof(s_encode_buf) - 1U);
    if (enc_len == 0U) {
        return false;
    }
    s_encode_buf[enc_len++] = 0x00; /* frame delimiter */

    /* Modest timeout — the io_uart_send path uses blocking HAL transmit. */
    return io_uart_send(VCP_UART, s_encode_buf, enc_len, 100U) == IO_OK;
}

void messages_vcp_init(void)
{
    (void)io_uart_open(VCP_UART, vcp_on_rx, NULL);
    messages_channel_sink_set(CH_VCP, vcp_sink);
}
