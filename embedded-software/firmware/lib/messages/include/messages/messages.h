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

#ifdef __cplusplus
}
#endif

#endif /* MESSAGES_MESSAGES_H */
