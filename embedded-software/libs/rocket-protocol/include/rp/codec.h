#ifndef RP_CODEC_H
#define RP_CODEC_H

#include <stdbool.h>
#include <stdint.h>

#include "pb.h"

#define RP_PACKET_MAX_SIZE (256)

typedef enum rp_codec_status {
    RP_CODEC_OK,
    RP_CODEC_NULL_POINTER,
    RP_CODEC_OVERFLOW,
    RP_CODEC_CHECKSUM_MISMATCH,
    RP_CODEC_ERROR,
} rp_codec_status_t;

typedef struct rp_packet_encode_result {
    size_t written;
    rp_codec_status_t status;
} rp_packet_encode_result_t;

typedef struct rp_packet_decode_result {
    rp_codec_status_t status;
} rp_packet_decode_result_t;

rp_packet_encode_result_t rp_packet_encode(uint8_t *packet, size_t packet_capacity,
                                           const pb_msgdesc_t *fields, const void *message);
rp_packet_decode_result_t rp_packet_decode(const uint8_t *packet, size_t packet_size,
                                           const pb_msgdesc_t *fields, void *message);

#endif // RP_CODEC_H
