/**
 * @file messages.h
 * @brief UBC Rocket message registry — publish-side runtime (phase 1).
 *
 * This is the firmware C runtime that backs the PUB_<MOD>_<NAME> macros emitted
 * by the message registry codegen. It owns envelope assembly, per-channel
 * routing fan-out, per-(message, channel) token-bucket rate-limiting, and
 * publishes a periodic drop-counter snapshot.
 *
 * Phase 1 scope:
 *   - Class A (packed struct) publishes only. Class B is a stub (phase 2 nanopb).
 *   - SD channel sink is wired through a caller-provided callback (see
 *     messages_sd_sink_set). VCP and UDP sinks are stubs that just increment
 *     the per-channel drop counter.
 *
 * Wire format (little-endian, packed; identical across all consumers):
 * @code
 *   [u16 length]              // bytes that follow this field; length = 12 + N + 2
 *   [u8  class]               // MSG_CLASS_A=0x41 or MSG_CLASS_B=0x42
 *   [u8  module_id]
 *   [u16 msg_id]
 *   [u64 t_us_publish]        // envelope time (when published)
 *   [u8  payload[N]]
 *   [u16 crc16_ccitt]         // CRC-16/CCITT-FALSE over [class..payload]
 * @endcode
 *
 * @ UBC Rocket, 2026
 */

#ifndef MESSAGES_MESSAGES_H
#define MESSAGES_MESSAGES_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Wire-format constants                                                      */
/* -------------------------------------------------------------------------- */

/** @brief Size of the envelope header that precedes the payload (class .. t_us). */
#define MESSAGES_ENVELOPE_HEADER_SIZE  12U

/** @brief Size of the trailing CRC field. */
#define MESSAGES_ENVELOPE_CRC_SIZE     2U

/** @brief Size of the leading length field. */
#define MESSAGES_ENVELOPE_LENGTH_SIZE  2U

/**
 * @brief Total on-the-wire record size for a payload of N bytes.
 *
 * = 2 (length) + 12 (header) + N (payload) + 2 (crc).
 */
#define MESSAGES_RECORD_SIZE(payload_n) \
    (MESSAGES_ENVELOPE_LENGTH_SIZE + MESSAGES_ENVELOPE_HEADER_SIZE + \
     (size_t)(payload_n) + MESSAGES_ENVELOPE_CRC_SIZE)

/** @brief Maximum envelope payload size we can route in phase 1. */
#define MESSAGES_MAX_PAYLOAD_SIZE      256U

/* -------------------------------------------------------------------------- */
/* Sink callback type                                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief Sink callback signature for a channel.
 *
 * The runtime calls this with the fully-assembled envelope (length field,
 * header, payload, CRC — i.e. exactly @ref MESSAGES_RECORD_SIZE bytes).
 * Implementations must copy the buffer if they need to keep it; the pointer
 * is invalid after return.
 *
 * @param record     Pointer to the assembled record (length field first).
 * @param record_len Total bytes in @p record.
 * @return true on accept, false if the sink dropped the record (e.g. full).
 */
typedef bool (*messages_sink_fn_t)(const uint8_t *record, size_t record_len);

/* -------------------------------------------------------------------------- */
/* Lifecycle                                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialise the message runtime.
 *
 * Zeroes drop counters and token-bucket state. Call once per core, after the
 * SD route is initialised but before any task starts publishing.
 */
void messages_init(void);

/**
 * @brief Register the SD-channel sink callback.
 *
 * The app should wire this to a thin shim that hands the binary blob to the
 * existing SD log task (see firmware/lib/log_records and the SD log task in
 * Core/Src/SD_logging). The runtime does not own the SD writer.
 *
 * If no SD sink is registered, the SD channel acts as a drop-everything stub
 * and increments the SD drop counter for every routed message.
 *
 * @param fn Sink callback (may be NULL to deregister).
 */
void messages_sd_sink_set(messages_sink_fn_t fn);

/**
 * @brief Register a sink for any channel by ID.
 *
 * Generalises messages_sd_sink_set — used by the VCP RX driver to install
 * the response-out path for commands, and by future channels (UDP, radio,
 * etc.). messages_sd_sink_set is now a thin wrapper that calls this with
 * channel_id = CH_SD.
 */
void messages_channel_sink_set(uint8_t channel_id, messages_sink_fn_t fn);

/* -------------------------------------------------------------------------- */
/* Publish                                                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief Publish a Class A packed-struct payload.
 *
 * Called by the PUB_<MOD>_<NAME> macros emitted by codegen. Not for direct
 * use by app code.
 *
 * Looks up the routing entry, iterates enabled channels, applies the
 * token-bucket rate-limit per (msg, channel), assembles the envelope, and
 * hands it to each channel's sink.
 *
 * @param module_id    Module identifier (MOD_*).
 * @param msg_id       Message identifier (MSG_*).
 * @param payload      Pointer to packed payload struct.
 * @param payload_size Size of payload in bytes; must equal the routing
 *                     entry's payload_size for Class A.
 * @return true if at least one channel accepted the message,
 *         false if the message was dropped on every enabled channel (rate
 *         limited, sink full, or no enabled channels).
 *
 * @note Task-context only. Not safe from ISR in phase 1.
 */
bool messages_publish_a(uint8_t module_id, uint16_t msg_id,
                        const void *payload, uint16_t payload_size);

/**
 * @brief Publish a Class B (nanopb) payload — phase 2 stub.
 *
 * @return Always false in phase 1.
 */
bool messages_publish_b(uint8_t module_id, uint16_t msg_id,
                        const void *payload, uint16_t payload_size);

/**
 * @brief Emit a periodic snapshot of the per-channel drop counters.
 *
 * Internally publishes the system.drop_counter Class A message containing the
 * current counters. Intended to be called from a 1 Hz housekeeping tick.
 */
void messages_publish_drop_counters(void);

/* -------------------------------------------------------------------------- */
/* Introspection (test / diagnostics)                                         */
/* -------------------------------------------------------------------------- */

/**
 * @brief Read the current drop counters.
 *
 * @param channel_id Channel identifier (CH_SD, CH_VCP, CH_UDP).
 * @return Number of drops on that channel since boot (or messages_init).
 */
uint32_t messages_get_drops(uint8_t channel_id);

/* -------------------------------------------------------------------------- */
/* Command dispatch (inbound)                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Command handler signature.
 *
 * The dispatcher hands the raw nanopb-encoded request bytes; the handler is
 * responsible for nanopb-decoding them into its typed request struct (the
 * generated messages_Cmd<Mod><Cmd>Request_t), doing the work, and nanopb-
 * encoding a typed response into the provided buffer. Returns 0 on success,
 * nonzero on internal error (the dispatcher then encodes an empty response
 * payload — typically the typed response struct carries its own ok/error_code
 * fields, which is the preferred way to surface failure).
 *
 * @param req_bytes  Decoded request payload (post-COBS / post-envelope).
 * @param req_len    Length of req_bytes.
 * @param resp_bytes Buffer for the response payload to be filled.
 * @param resp_cap   Capacity of resp_bytes (MESSAGES_MAX_PAYLOAD_SIZE).
 * @param resp_len   [out] Bytes written into resp_bytes.
 */
typedef int (*messages_cmd_handler_t)(
    const uint8_t *req_bytes, size_t req_len,
    uint8_t *resp_bytes, size_t resp_cap, size_t *resp_len);

/** @brief Maximum number of distinct commands that can be registered. */
#define MESSAGES_MAX_REGISTERED_COMMANDS  16U

/**
 * @brief Register a handler for one (module_id, cmd_id) pair.
 *
 * @param module_id Module identifier (MOD_*).
 * @param cmd_id    Command identifier (CMD_<MOD>_<NAME>).
 * @param fn        Handler function. NULL deregisters.
 * @return true if the slot was assigned, false if the registry is full.
 */
bool messages_register_command_handler(uint8_t module_id, uint16_t cmd_id,
                                        messages_cmd_handler_t fn);

/**
 * @brief Look up the registered handler for a command.
 *
 * @return Handler function or NULL if no handler is registered.
 */
messages_cmd_handler_t messages_lookup_command_handler(uint8_t module_id,
                                                       uint16_t cmd_id);

/**
 * @brief Dispatch one inbound command frame.
 *
 * Caller hands the assembled envelope (length .. crc) — same wire format that
 * goes outbound. The dispatcher validates CRC + class + envelope, decodes
 * the (module_id, cmd_id) tuple (commands ride the same id namespace as
 * messages: msg_id = cmd_id for command messages), looks up the registered
 * handler, calls it, and feeds the response envelope back into the same
 * channel sink (so the host sees `command response` records on the wire).
 *
 * @param channel_id Channel the inbound frame arrived on; used to send the
 *                   response back to the same channel.
 * @param record     Pointer to the inbound record (length field first).
 * @param record_len Bytes available at @p record.
 * @return true if a handler ran (even if it returned nonzero), false if the
 *         frame was rejected (bad CRC, unknown command, sink full).
 */
bool messages_handle_inbound(uint8_t channel_id,
                              const uint8_t *record, size_t record_len);

/**
 * @brief Register the built-in system.* command handlers.
 *
 * Currently registers: get_build_info. Call once at boot after
 * messages_init(). No-op if MESSAGES_HAVE_NANOPB isn't defined (the
 * host-side test harness doesn't link nanopb).
 */
void messages_system_commands_register(void);

#ifdef __cplusplus
}
#endif

#endif /* MESSAGES_MESSAGES_H */
