/**
 * @file cobs.c
 * @brief COBS implementation.
 *
 * Tested against the canonical test vectors from RFC-style references:
 *   {0x00}                      -> {0x01,0x01}
 *   {0x11,0x22,0x00,0x33}       -> {0x03,0x11,0x22,0x02,0x33}
 *   254 non-zero bytes           -> {0xFF, 254 bytes}
 *   254 non-zero + 0x00          -> {0xFF, 254 bytes, 0x01, 0x01}
 *
 * @ UBC Rocket, 2026
 */

#include "cobs/cobs.h"

size_t cobs_encode(const uint8_t *src, size_t src_len,
                   uint8_t *dst, size_t dst_cap)
{
    if (dst_cap == 0U) {
        return 0U;
    }

    size_t code_idx = 0U;       /* index of the next "code" byte to fill in */
    size_t out_idx  = 1U;       /* writing position (we reserve dst[0] for code) */
    uint8_t code    = 1U;        /* distance to the next zero, plus 1 */

    for (size_t i = 0U; i < src_len; ++i) {
        if (src[i] == 0U) {
            /* Emit current code and start a new run. */
            if (code_idx >= dst_cap) return 0U;
            dst[code_idx] = code;
            code_idx = out_idx;
            if (out_idx >= dst_cap) return 0U;
            out_idx++;
            code = 1U;
        } else {
            if (out_idx >= dst_cap) return 0U;
            dst[out_idx++] = src[i];
            code++;
            if (code == 0xFFU) {
                /* Maximum run length; emit and reset (no implicit zero). */
                if (code_idx >= dst_cap) return 0U;
                dst[code_idx] = code;
                code_idx = out_idx;
                if (out_idx >= dst_cap) return 0U;
                out_idx++;
                code = 1U;
            }
        }
    }
    if (code_idx >= dst_cap) return 0U;
    dst[code_idx] = code;
    /* If the input ended right after a 0xFF run, code_idx == out_idx-1 and
     * the trailing code byte is itself the last output byte — that's the
     * one extra byte the algorithm reserves. */
    return out_idx;
}

size_t cobs_decode(const uint8_t *src, size_t src_len,
                   uint8_t *dst, size_t dst_cap)
{
    if (src_len == 0U) {
        return 0U;
    }
    size_t in_idx  = 0U;
    size_t out_idx = 0U;

    while (in_idx < src_len) {
        uint8_t code = src[in_idx++];
        if (code == 0U || in_idx + (size_t)(code - 1U) > src_len) {
            return 0U;  /* malformed: 0x00 must never appear in a COBS frame */
        }

        for (uint8_t k = 1U; k < code; ++k) {
            if (out_idx >= dst_cap) return 0U;
            dst[out_idx++] = src[in_idx++];
        }
        /* If the code is < 0xFF and we still have more bytes to read,
         * emit an implicit 0x00. The last code does not produce a 0x00. */
        if (code < 0xFFU && in_idx < src_len) {
            if (out_idx >= dst_cap) return 0U;
            dst[out_idx++] = 0U;
        }
    }
    return out_idx;
}
