/**
 * @file    messages_udp.c
 * @brief   Bridge between lwIP UDP and the messages runtime (CM7).
 *
 * The IOC has lwIP enabled on CM7 and CubeMX has generated the middleware
 * + the MX_LWIP_Init entry point (called from StartDefaultTask in
 * Core/Src/freertos.c before we touch anything here).
 *
 * Our setup runs from StartDefaultTask AFTER MX_LWIP_Init, via the USER
 * CODE block in freertos.c calling messages_udp_init(). That ordering is
 * important: lwIP raw-API calls (udp_new, udp_bind, etc.) require the
 * tcpip_thread to exist.
 *
 * Threading: lwIP's raw API is single-threaded — strictly tcpip_thread.
 * Any call from other threads must wrap with LOCK_TCPIP_CORE() /
 * UNLOCK_TCPIP_CORE() (CORE_LOCKING is enabled by default in lwIP 2.x).
 * Our udp_sink runs on whatever task called PUB_*; it acquires the lock
 * before pbuf_alloc + udp_sendto. The udp_recv callback already runs on
 * tcpip_thread so no lock needed there.
 *
 * DHCP: enabled in lwipopts.h but CubeMX-generated MX_LWIP_Init does NOT
 * call dhcp_start() — we kick it off here after the netif is up.
 *
 * UBC Rocket, 2026
 */

#include "app/messages_udp.h"
#include "messages/messages.h"
#include "generated/messages/registry.h"

#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"
#include "lwip/dhcp.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"

#include <string.h>

/* Local UDP port we listen on for inbound commands. Matches the
 * registry's channels.udp.transport_config.port (= 9000). */
#define UDP_LOCAL_PORT  9000U

/* Where outbound telemetry / responses get sent. Compile-time for now;
 * runtime configurability lands when the console gains a set-destination
 * command. Set to 0.0.0.0 (default route) until DHCP / static config
 * settles — udp_sendto will silently fail until s_dest is real. */
static ip_addr_t       s_dest;
static u16_t           s_dest_port = UDP_LOCAL_PORT;
static struct udp_pcb *s_pcb;

/* gnetif lives in LWIP/App/lwip.c (CubeMX-generated). */
extern struct netif gnetif;

/* ---- Receive callback — runs on tcpip_thread ----------------------- */
static void udp_rx_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                       const ip_addr_t *addr, u16_t port)
{
    (void)arg; (void)pcb;

    /* Remember the sender so responses go back to whoever just talked to
     * us. Simple "follow the last requester" — fine for one ground
     * station; if we ever multiplex, the command response needs to carry
     * the destination explicitly. */
    if (addr != NULL) {
        s_dest = *addr;
        s_dest_port = port;
    }

    /* Walk the pbuf chain into a contiguous scratch buffer. Records that
     * don't fit our envelope budget get dropped. */
    static uint8_t scratch[MESSAGES_MAX_PAYLOAD_SIZE +
                           MESSAGES_ENVELOPE_LENGTH_SIZE +
                           MESSAGES_ENVELOPE_HEADER_SIZE +
                           MESSAGES_ENVELOPE_CRC_SIZE];
    if (p->tot_len <= sizeof(scratch)) {
        u16_t copied = pbuf_copy_partial(p, scratch, p->tot_len, 0);
        (void)messages_handle_inbound(CH_UDP, scratch, copied);
    }
    pbuf_free(p);
}

/* ---- TX sink — called from any task by the messages runtime --------- */
static bool udp_sink(const uint8_t *record, size_t record_len)
{
    if (s_pcb == NULL || ip_addr_isany(&s_dest)) {
        /* lwIP not yet up, or we haven't learned a destination — drop
         * (the messages runtime will bump the udp drop counter). */
        return false;
    }
    if (record_len == 0U || record_len > 0xFFFFU) {
        return false;
    }

    bool ok = false;
    LOCK_TCPIP_CORE();
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, (u16_t)record_len, PBUF_RAM);
    if (p != NULL) {
        memcpy(p->payload, record, record_len);
        ok = (udp_sendto(s_pcb, p, &s_dest, s_dest_port) == ERR_OK);
        pbuf_free(p);
    }
    UNLOCK_TCPIP_CORE();
    return ok;
}

/* ---- Init: runs from freertos.c StartDefaultTask after MX_LWIP_Init -- */
void messages_udp_init(void)
{
    /* lwIP's UDP raw API setup. tcpip_thread already exists at this point. */
    LOCK_TCPIP_CORE();

    /* DHCP first so we get an IP. If there's no DHCP server (point-to-
     * point, no router), the netif stays at 0.0.0.0 and outbound sends
     * fail — that's the right failure mode for "no network here". */
    dhcp_start(&gnetif);

    s_pcb = udp_new();
    if (s_pcb != NULL) {
        (void)udp_bind(s_pcb, IP_ADDR_ANY, UDP_LOCAL_PORT);
        udp_recv(s_pcb, udp_rx_cb, NULL);
    }

    ip_addr_set_zero(&s_dest);  /* learned on first inbound packet */

    UNLOCK_TCPIP_CORE();

    messages_channel_sink_set(CH_UDP, udp_sink);
}
