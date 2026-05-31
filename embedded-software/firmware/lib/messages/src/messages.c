/**
 * @file messages.c
 * @brief Publish-side runtime for the UBC Rocket message registry (phase 1).
 *
 * Owns envelope assembly, routing fan-out, token-bucket rate limiting, and
 * drop counters. Class A (packed struct) publishes only; Class B is a phase-2
 * stub.
 *
 * Routing table and message IDs come from codegen headers under
 * firmware/generated/messages/. The runtime is intentionally agnostic to the
 * specific message catalogue — every routing decision flows through the table.
 *
 * Concurrency:
 *   - Drop counters use __atomic_fetch_add so ISR / cross-task increments
 *     don't tear (currently only task context, but the API is small and the
 *     atomic op is cheap).
 *   - Token-bucket state is per (msg_id, channel) and is allowed to race in
 *     pathological cases: at worst we drop or admit one extra message. The
 *     spec calls this acceptable; no locks.
 *
 * @ UBC Rocket, 2026
 */

#include "messages/messages.h"
#include "messages/crc16.h"

#include <string.h>

/* --- Codegen contract -----------------------------------------------------
 *
 * These headers are emitted by tools/messages_codegen into
 * firmware/generated/messages/. They will appear at integration time. For
 * host tests we shim them with the tests/fake_*.h files placed on the
 * include path before this lib's own includes.
 */
#include "generated/messages/registry.h"
#include "generated/messages/routing.h"
#include "generated/messages/types.h"

/* Optional dependency: real firmware uses the timestamp lib's DWT cycle
 * counter. The test harness shims this header to provide a fake clock. */
#include "timestamp/timestamp.h"

/* -------------------------------------------------------------------------- */
/* Compile-time sanity                                                        */
/* -------------------------------------------------------------------------- */

_Static_assert(CH_COUNT >= 1, "CH_COUNT must be at least 1");
_Static_assert(MSG_CLASS_A == 0x41, "MSG_CLASS_A wire constant mismatch");
_Static_assert(MSG_CLASS_B == 0x42, "MSG_CLASS_B wire constant mismatch");

/* -------------------------------------------------------------------------- */
/* Tunables                                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief Upper bound on the number of routing entries we can rate-limit.
 *
 * Sized generously for phase 1 (a few dozen messages). If we ever exceed
 * this we silently disable rate limiting for the overflow entries — the
 * runtime still functions, just without back-pressure.
 */
#ifndef MESSAGES_MAX_ENTRIES
#define MESSAGES_MAX_ENTRIES 128U
#endif

/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint64_t last_us;     /* time of last token-bucket refill */
    uint32_t tokens_q;    /* tokens scaled by 1e6 (so we can refill fractionally) */
    uint8_t  initialised; /* 0 until first admit() call (zero-init means fresh) */
} bucket_state_t;

static bucket_state_t s_buckets[MESSAGES_MAX_ENTRIES][CH_COUNT];
static uint32_t       s_drops[CH_COUNT];
static messages_sink_fn_t s_sd_sink;

/* -------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief Find the index of (module_id, msg_id) in the routing table.
 *
 * Linear scan. Routing tables are small (tens of entries); a sorted-by-id
 * binary search would be a micro-optimisation we'll do if profiling demands.
 *
 * @return Index in [0, messages_routing_table_len), or SIZE_MAX if not found.
 */
static size_t find_routing_index(uint8_t module_id, uint16_t msg_id)
{
    for (size_t i = 0U; i < messages_routing_table_len; ++i) {
        const messages_routing_entry_t *e = &messages_routing_table[i];
        if (e->module_id == module_id && e->msg_id == msg_id) {
            return i;
        }
    }
    return (size_t)-1;
}

/**
 * @brief Token-bucket admission test.
 *
 * Capacity = max_rate_hz, refill rate = max_rate_hz tokens/sec. Tokens are
 * stored scaled by 1e6 to track sub-token refills between calls without FP.
 *
 * @return true to admit, false to drop.
 */
static bool token_bucket_admit(size_t routing_idx, uint8_t channel_id,
                               uint16_t max_rate_hz, uint64_t now_us)
{
    if (max_rate_hz == 0U) {
        /* Rate limit of 0 means "blocked on this channel" — drop. */
        return false;
    }

    if (routing_idx >= MESSAGES_MAX_ENTRIES) {
        /* Out of bucket slots — admit (no back-pressure for this entry). */
        return true;
    }

    bucket_state_t *b = &s_buckets[routing_idx][channel_id];

    /* Initial state: full bucket, last_us = now. */
    if (b->initialised == 0U) {
        b->initialised = 1U;
        b->last_us = now_us;
        b->tokens_q = (uint32_t)max_rate_hz * 1000000U;
    }

    uint64_t dt_us = now_us - b->last_us;
    b->last_us = now_us;

    /* refill = max_rate_hz * dt_us / 1e6 tokens. Scaled: tokens_q += max_rate_hz * dt_us. */
    uint64_t refill_q = (uint64_t)max_rate_hz * dt_us;
    uint64_t capacity_q = (uint64_t)max_rate_hz * 1000000U;
    uint64_t new_q = (uint64_t)b->tokens_q + refill_q;
    if (new_q > capacity_q) {
        new_q = capacity_q;
    }

    /* Need one full token (1e6 scaled). */
    if (new_q < 1000000U) {
        b->tokens_q = (uint32_t)new_q;
        return false;
    }

    new_q -= 1000000U;
    b->tokens_q = (uint32_t)new_q;
    return true;
}

/**
 * @brief Atomically bump the per-channel drop counter.
 */
static void bump_drop(uint8_t channel_id)
{
    if (channel_id >= CH_COUNT) {
        return;
    }
#if defined(__GNUC__) || defined(__clang__)
    __atomic_fetch_add(&s_drops[channel_id], 1U, __ATOMIC_RELAXED);
#else
    s_drops[channel_id] += 1U;
#endif
}

/**
 * @brief Serialise envelope into @p out.
 *
 * @param out          Destination buffer, must hold MESSAGES_RECORD_SIZE(payload_size) bytes.
 * @param msg_class    MSG_CLASS_A or MSG_CLASS_B.
 * @param module_id    Module ID.
 * @param msg_id       Message ID.
 * @param t_us_publish Envelope timestamp.
 * @param payload      Payload bytes (may be NULL iff payload_size==0).
 * @param payload_size Payload byte count.
 * @return Total record bytes written.
 */
static size_t assemble_envelope(uint8_t *out,
                                uint8_t msg_class,
                                uint8_t module_id,
                                uint16_t msg_id,
                                uint64_t t_us_publish,
                                const uint8_t *payload,
                                uint16_t payload_size)
{
    /* length = bytes after the length field = 12 + N + 2 */
    uint16_t length = (uint16_t)(MESSAGES_ENVELOPE_HEADER_SIZE + payload_size + MESSAGES_ENVELOPE_CRC_SIZE);

    size_t off = 0U;
    /* [u16 length] LE */
    out[off++] = (uint8_t)(length & 0xFFU);
    out[off++] = (uint8_t)((length >> 8) & 0xFFU);

    /* Start of CRC region (class .. payload). */
    size_t crc_start = off;

    /* [u8 class] */
    out[off++] = msg_class;
    /* [u8 module_id] */
    out[off++] = module_id;
    /* [u16 msg_id] LE */
    out[off++] = (uint8_t)(msg_id & 0xFFU);
    out[off++] = (uint8_t)((msg_id >> 8) & 0xFFU);
    /* [u64 t_us_publish] LE */
    for (unsigned int i = 0U; i < 8U; ++i) {
        out[off++] = (uint8_t)((t_us_publish >> (8U * i)) & 0xFFU);
    }

    /* [u8 payload[N]] */
    if (payload_size > 0U && payload != NULL) {
        memcpy(&out[off], payload, payload_size);
        off += payload_size;
    }

    /* [u16 crc16] LE — over [class..payload] */
    uint16_t crc = messages_crc16(&out[crc_start], off - crc_start);
    out[off++] = (uint8_t)(crc & 0xFFU);
    out[off++] = (uint8_t)((crc >> 8) & 0xFFU);

    return off;
}

/**
 * @brief Dispatch a fully-assembled record to one channel.
 *
 * Phase 1: only CH_SD has a real sink. VCP and UDP increment drops and return.
 */
static bool dispatch_to_channel(uint8_t channel_id, const uint8_t *record, size_t record_len)
{
    if (channel_id == CH_SD) {
        messages_sink_fn_t fn = s_sd_sink;
        if (fn == NULL) {
            bump_drop(CH_SD);
            return false;
        }
        if (!fn(record, record_len)) {
            bump_drop(CH_SD);
            return false;
        }
        return true;
    }

    /* CH_VCP / CH_UDP / future channels: stub. */
    bump_drop(channel_id);
    return false;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

void messages_init(void)
{
    memset(s_buckets, 0, sizeof(s_buckets));
    memset(s_drops, 0, sizeof(s_drops));
    s_sd_sink = NULL;
    /* Build CRC table eagerly so the first publish doesn't pay the init cost. */
    (void)messages_crc16(NULL, 0U);
}

void messages_sd_sink_set(messages_sink_fn_t fn)
{
    s_sd_sink = fn;
}

uint32_t messages_get_drops(uint8_t channel_id)
{
    if (channel_id >= CH_COUNT) {
        return 0U;
    }
#if defined(__GNUC__) || defined(__clang__)
    return __atomic_load_n(&s_drops[channel_id], __ATOMIC_RELAXED);
#else
    return s_drops[channel_id];
#endif
}

bool messages_publish_a(uint8_t module_id, uint16_t msg_id,
                        const void *payload, uint16_t payload_size)
{
    size_t idx = find_routing_index(module_id, msg_id);
    if (idx == (size_t)-1) {
        /* Unknown message — quietly drop. The codegen guarantees a routing
         * entry exists for every registered message; this branch only hits
         * if app code calls publish with hand-rolled IDs. */
        return false;
    }

    const messages_routing_entry_t *entry = &messages_routing_table[idx];

    if (entry->class != MSG_CLASS_A) {
        return false;
    }
    if (payload_size != entry->payload_size) {
        return false;
    }
    if (payload_size > MESSAGES_MAX_PAYLOAD_SIZE) {
        return false;
    }

    /* Assemble once, dispatch to each enabled channel. The envelope is
     * identical across channels (same t_us_publish), which matches the spec. */
    uint8_t record[MESSAGES_ENVELOPE_LENGTH_SIZE + MESSAGES_ENVELOPE_HEADER_SIZE +
                   MESSAGES_MAX_PAYLOAD_SIZE + MESSAGES_ENVELOPE_CRC_SIZE];
    uint64_t now_us = timestamp_us64();
    size_t record_len = assemble_envelope(record, MSG_CLASS_A, module_id, msg_id,
                                          now_us, (const uint8_t *)payload, payload_size);

    bool any_accepted = false;
    for (uint8_t ch = 0U; ch < (uint8_t)CH_COUNT; ++ch) {
        if ((entry->enabled_channels & (uint8_t)(1U << ch)) == 0U) {
            continue;
        }
        if (!token_bucket_admit(idx, ch, entry->max_rate_hz[ch], now_us)) {
            bump_drop(ch);
            continue;
        }
        if (dispatch_to_channel(ch, record, record_len)) {
            any_accepted = true;
        }
    }
    return any_accepted;
}

bool messages_publish_b(uint8_t module_id, uint16_t msg_id,
                        const void *payload, uint16_t payload_size)
{
    /* TODO(phase-2): nanopb-encode payload, then run through the same
     * routing / rate-limit / sink path as Class A. */
    (void)module_id;
    (void)msg_id;
    (void)payload;
    (void)payload_size;
    return false;
}

void messages_publish_drop_counters(void)
{
    msg_system_drop_counter_t snapshot;
#if defined(__GNUC__) || defined(__clang__)
    snapshot.sd_drops  = __atomic_load_n(&s_drops[CH_SD],  __ATOMIC_RELAXED);
    snapshot.vcp_drops = __atomic_load_n(&s_drops[CH_VCP], __ATOMIC_RELAXED);
    snapshot.udp_drops = __atomic_load_n(&s_drops[CH_UDP], __ATOMIC_RELAXED);
#else
    snapshot.sd_drops  = s_drops[CH_SD];
    snapshot.vcp_drops = s_drops[CH_VCP];
    snapshot.udp_drops = s_drops[CH_UDP];
#endif
    (void)messages_publish_a(MOD_SYSTEM, MSG_SYSTEM_DROP_COUNTER,
                             &snapshot, (uint16_t)sizeof(snapshot));
}
