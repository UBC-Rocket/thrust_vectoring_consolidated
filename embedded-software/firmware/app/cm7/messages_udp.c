/**
 * @file    messages_udp.c
 * @brief   Bridge between lwIP UDP and the messages runtime (CM7).
 *
 * PHASE-2 SCAFFOLDING. The IOC has lwIP enabled on CM7 but CubeMX has not
 * yet been re-run to drop the LwIP middleware sources into
 * Middlewares/Third_Party/LwIP/. Until that happens this file ships a
 * no-op sink registration so messages routing for CH_UDP completes cleanly
 * (silently dropping records) without forcing every consumer to skip CH_UDP
 * in their routing config.
 *
 * When LwIP lands, swap the stubbed bodies for the lwIP-raw shape sketched
 * inline below. The destination IP + port live in the registry
 * (channels.udp.transport_config.port — 9000 today). Hardcode the dest
 * IP at compile time for v1; runtime configurability lands when the
 * console gains a set_route-style command for the destination.
 *
 * UBC Rocket, 2026
 */

#include "app/messages_udp.h"
#include "messages/messages.h"
#include "generated/messages/registry.h"

#include <stddef.h>
#include <stdint.h>

/* TODO(lwip): once the middleware is in the tree, add:
 *
 *   #include "lwip/udp.h"
 *   #include "lwip/init.h"
 *   #include "lwip/timeouts.h"
 *
 *   static struct udp_pcb *s_pcb;
 *   static ip_addr_t       s_dest;
 *   #define UDP_DEST_PORT 9000U     // matches channels.udp.transport_config.port
 *
 *   static void udp_rx_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
 *                          const ip_addr_t *addr, u16_t port) {
 *       // pbuf may chain; walk it into a contiguous buffer up to MESSAGES_MAX_PAYLOAD_SIZE
 *       // then call messages_handle_inbound(CH_UDP, buf, len). pbuf_free(p).
 *   }
 *
 *   static bool udp_sink(const uint8_t *record, size_t record_len) {
 *       struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, (u16_t)record_len, PBUF_RAM);
 *       if (!p) return false;
 *       memcpy(p->payload, record, record_len);
 *       err_t err = udp_sendto(s_pcb, p, &s_dest, UDP_DEST_PORT);
 *       pbuf_free(p);
 *       return err == ERR_OK;
 *   }
 */

static bool udp_sink_noop(const uint8_t *record, size_t record_len)
{
    /* lwIP not yet available — accept the call but drop the bytes.
     * Returning false lets the messages runtime bump the drop counter so
     * the absence of UDP transport is visible in system.drop_counter. */
    (void)record; (void)record_len;
    return false;
}

void messages_udp_init(void)
{
    /* TODO(lwip):
     *   1. tcpip_init(NULL, NULL); on first call only
     *   2. ethernetif_init() / netif_add() / netif_set_default()
     *   3. dhcp_start() OR netif_set_ipaddr() for static IP
     *   4. s_pcb = udp_new(); udp_bind(s_pcb, IP_ADDR_ANY, UDP_LOCAL_PORT);
     *      udp_recv(s_pcb, udp_rx_cb, NULL);
     *   5. IP4_ADDR(&s_dest, 192, 168, 1, 100);   // ground-station IP
     *   6. messages_channel_sink_set(CH_UDP, udp_sink);
     */
    messages_channel_sink_set(CH_UDP, udp_sink_noop);
}
