/**
 * @file cobs.h
 * @brief Consistent Overhead Byte Stuffing (COBS) — encode/decode.
 *
 * COBS removes every 0x00 byte from a payload so 0x00 can be used as an
 * unambiguous frame delimiter on a streaming transport (UART). Overhead
 * is at most ceil(N/254) bytes per N-byte input. We use COBS for the VCP
 * channel — see embedded-software/messages/registry.json channels.vcp.
 *
 * Reference: "Consistent Overhead Byte Stuffing" — Cheshire & Baker, 1999.
 *
 * @ UBC Rocket, 2026
 */
#ifndef COMMS_COBS_H
#define COMMS_COBS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Worst-case encoded size for an input of @p n bytes (excluding terminator). */
#define COBS_ENCODE_MAX(n) ((n) + (n) / 254U + 1U)

/**
 * @brief Encode @p src into @p dst. Output contains no 0x00 bytes.
 *
 * @param src         Input buffer.
 * @param src_len     Bytes in @p src.
 * @param dst         Output buffer; must have at least COBS_ENCODE_MAX(src_len) bytes.
 * @param dst_cap     Capacity of @p dst.
 * @return Number of bytes written to @p dst, or 0 on buffer overflow.
 *
 * The caller is responsible for appending the 0x00 delimiter after the
 * returned bytes if the framed output is going on a stream.
 */
size_t cobs_encode(const uint8_t *src, size_t src_len,
                   uint8_t *dst, size_t dst_cap);

/**
 * @brief Decode @p src (one COBS frame, without delimiter) into @p dst.
 *
 * @param src         Encoded buffer (no trailing 0x00).
 * @param src_len     Bytes in @p src.
 * @param dst         Output buffer; up to @p src_len - 1 bytes are written.
 * @param dst_cap     Capacity of @p dst.
 * @return Number of bytes written to @p dst, or 0 on malformed input or
 *         buffer overflow.
 */
size_t cobs_decode(const uint8_t *src, size_t src_len,
                   uint8_t *dst, size_t dst_cap);

#ifdef __cplusplus
}
#endif

#endif /* COMMS_COBS_H */
