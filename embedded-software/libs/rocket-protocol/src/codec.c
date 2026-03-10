#include "rp/codec.h"

#include <stdbool.h>
#include <stdint.h>
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"

#include "rp/cobs/cobs.h"
#include "rp/crc/crc.h"

static rp_codec_status_t cobs_to_codec_status(cobs_status_t status);

rp_packet_encode_result_t rp_packet_encode(uint8_t *packet, size_t packet_capacity,
                                           const pb_msgdesc_t *fields, const void *message)
{
    rp_packet_encode_result_t result = {
        .written = 0,
        .status = RP_CODEC_ERROR,
    };

    if (packet == NULL || fields == NULL || message == NULL) {
        result.status = RP_CODEC_NULL_POINTER;
        return result;
    }

    uint8_t pb_encoded[RP_PACKET_MAX_SIZE];
    pb_ostream_t pb_encode_stream = pb_ostream_from_buffer(pb_encoded, sizeof(pb_encoded));

    if (!pb_encode(&pb_encode_stream, fields, message)) {
        result.status = RP_CODEC_ERROR;
        return result;
    }

    size_t pb_encoded_size = pb_encode_stream.bytes_written;

    // Do we have enough room for the checksum?
    if (pb_encoded_size >= (sizeof(pb_encoded) - 2)) {
        result.status = RP_CODEC_OVERFLOW;
        return result;
    }

    uint16_t checksum = crc16_ccitt(pb_encoded, pb_encoded_size);

    // Append checksum as LE
    pb_encoded[pb_encoded_size++] = (checksum >> 0) & 0xFF;
    pb_encoded[pb_encoded_size++] = (checksum >> 8) & 0xFF;

    cobs_result_t cobs_result = cobs_encode(pb_encoded, pb_encoded_size, packet, packet_capacity);

    if (cobs_result.status != COBS_OK) {
        result.status = cobs_to_codec_status(cobs_result.status);
        return result;
    }

    result.written = cobs_result.written;
    result.status = RP_CODEC_OK;

    return result;
}

rp_packet_decode_result_t rp_packet_decode(const uint8_t *packet, size_t packet_size,
                                           const pb_msgdesc_t *fields, void *message)
{
    rp_packet_decode_result_t result = {
        .status = RP_CODEC_ERROR,
    };

    if (packet == NULL || fields == NULL || message == NULL) {
        result.status = RP_CODEC_NULL_POINTER;
        return result;
    }

    uint8_t cobs_decoded[RP_PACKET_MAX_SIZE];

    cobs_result_t cobs_result =
        cobs_decode(packet, packet_size, cobs_decoded, sizeof(cobs_decoded));
    size_t cobs_decoded_size = cobs_result.written;

    if (cobs_result.status != COBS_OK) {
        result.status = cobs_to_codec_status(cobs_result.status);
        return result;
    }

    // Expect data to have a checksum
    if (cobs_decoded_size < 2) {
        result.status = RP_CODEC_ERROR;
        return result;
    }

    uint16_t checksum = crc16_ccitt(cobs_decoded, cobs_decoded_size);

    if (checksum != CRC16_CCITT_RESIDUE) {
        result.status = RP_CODEC_CHECKSUM_MISMATCH;
        return result;
    }

    pb_istream_t pb_decode_stream = pb_istream_from_buffer(cobs_decoded, cobs_decoded_size - 2);

    if (!pb_decode(&pb_decode_stream, fields, message)) {
        result.status = RP_CODEC_ERROR;
        return result;
    }

    result.status = RP_CODEC_OK;

    return result;
}

static rp_codec_status_t cobs_to_codec_status(cobs_status_t status)
{
    switch (status) {
    case COBS_OK:
        return RP_CODEC_OK;
    case COBS_NULL_POINTER:
        return RP_CODEC_NULL_POINTER;
    case COBS_OUTPUT_OVERFLOW:
        return RP_CODEC_OVERFLOW;
    default:
        return RP_CODEC_ERROR;
    }
}
