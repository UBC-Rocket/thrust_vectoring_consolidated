/**
 * @file    messages_udp.h
 * @brief   Bring-up entry point for the messages-runtime UDP channel (CM7).
 *
 * UBC Rocket, 2026
 */
#ifndef APP_MESSAGES_UDP_H
#define APP_MESSAGES_UDP_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise lwIP, open a UDP socket, register the CH_UDP TX sink,
 *        and hook RX → messages_handle_inbound.
 *
 * Currently a scaffold — the body is a no-op sink registration until the
 * LwIP middleware is regenerated into Middlewares/Third_Party/LwIP/.
 * See messages_udp.c for the lwIP-raw shape to fill in.
 */
void messages_udp_init(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MESSAGES_UDP_H */
