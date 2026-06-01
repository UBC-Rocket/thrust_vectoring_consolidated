/**
 * @file    messages_vcp.h
 * @brief   Bring-up entry point for the messages-runtime VCP channel.
 *
 * UBC Rocket, 2026
 */
#ifndef APP_MESSAGES_VCP_H
#define APP_MESSAGES_VCP_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Open the VCP UART, hook RX → dispatcher, and register the TX
 *        sink with the messages runtime (channel CH_VCP).
 *
 * Call once at boot after messages_init() and after IO has been brought
 * up (io_init_cm4 + log_service_init).
 */
void messages_vcp_init(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MESSAGES_VCP_H */
