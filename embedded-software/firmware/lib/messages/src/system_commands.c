/**
 * @file system_commands.c
 * @brief Built-in handlers for system.* commands.
 *
 * Phase-2 ships exactly one: system.get_build_info — a pure-read command
 * that returns registry version + CRC32 + git SHA + build flavor so the
 * ground console can verify it's talking to a firmware built against the
 * same registry. Adding lock_routes / set_route handlers comes when
 * routing mutability lands.
 *
 * Call messages_system_commands_register() once at boot, after
 * messages_init(). NOOP if MESSAGES_HAVE_NANOPB isn't defined.
 *
 * @ UBC Rocket, 2026
 */

#include "messages/messages.h"
#include "generated/messages/registry.h"

#if defined(MESSAGES_HAVE_NANOPB)

#include "generated/proto/messages.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

#include <string.h>

/* Build-time identity. Override via -DMESSAGES_GIT_SHA=\"..\" /
 * -DMESSAGES_BUILD_FLAVOR=N from the firmware CMake; placeholder
 * values land in the binary otherwise. */
#ifndef MESSAGES_GIT_SHA
#define MESSAGES_GIT_SHA "unknown"
#endif
#ifndef MESSAGES_BUILD_FLAVOR
#define MESSAGES_BUILD_FLAVOR 0   /* 0=debug, 1=release */
#endif

static int handle_get_build_info(const uint8_t *req_bytes, size_t req_len,
                                  uint8_t *resp_bytes, size_t resp_cap,
                                  size_t *resp_len)
{
    /* Request is empty — ignore any bytes that arrived. */
    (void)req_bytes; (void)req_len;

    messages_CmdSystemGetBuildInfoResponse resp = messages_CmdSystemGetBuildInfoResponse_init_zero;
    resp.registry_version = REGISTRY_VERSION;
    resp.registry_hash    = REGISTRY_CRC32;
    resp.build_flavor     = (uint32_t)MESSAGES_BUILD_FLAVOR;

    pb_ostream_t stream = pb_ostream_from_buffer(resp_bytes, resp_cap);
    if (!pb_encode(&stream, messages_CmdSystemGetBuildInfoResponse_fields, &resp)) {
        *resp_len = 0;
        return -1;
    }
    *resp_len = stream.bytes_written;
    return 0;
}

void messages_system_commands_register(void)
{
    (void)messages_register_command_handler(
        MOD_SYSTEM, CMD_SYSTEM_GET_BUILD_INFO, handle_get_build_info);
}

#else  /* !MESSAGES_HAVE_NANOPB */

/* Without nanopb, the handler can't encode — register a no-op. */
void messages_system_commands_register(void) {}

#endif
