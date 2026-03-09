#include "rp/cobs/cobs.h"

#include <stdint.h>

/**
 * Encodes the input data using COBS.
 *
 * @param data Buffer of bytes to encode
 * @param data_size Number of bytes to encode
 * @param output Output buffer to store the encoded data (including the delimiter)
 * @param output_capacity Maximum size of the output buffer
 * @return cobs_result_t
 */
cobs_result_t cobs_encode(const uint8_t *data, size_t data_size, uint8_t *output,
                          size_t output_capacity)
{
    cobs_result_t result = {
        .written = 0,
        .status = COBS_OK,
    };

    if (data == NULL || output == NULL) {
        result.status = COBS_NULL_POINTER;
        return result;
    }

    if (output_capacity < COBS_ENCODED_MIN_SIZE) {
        result.status = COBS_OUTPUT_OVERFLOW;
        return result;
    }

    size_t output_idx = 1; /**< Index to the next free byte in the output buffer */
    size_t code_idx = 0;   /**< Index to the code byte for the current block */
    uint8_t code = 0x01;   /**< Offset to the next delimiter byte */

    for (size_t i = 0; i < data_size; i++) {
        uint8_t byte = data[i];

        if (byte == COBS_DELIMITER_BYTE || code == 0xFF) {
            output[code_idx] = code;
            code = 0x01;

            code_idx = output_idx;
            output_idx++;

            if (output_idx >= output_capacity) {
                result.status = COBS_OUTPUT_OVERFLOW;
                return result;
            }

            if (byte == COBS_DELIMITER_BYTE) {
                continue;
            }
        }

        if (output_idx >= output_capacity) {
            result.status = COBS_OUTPUT_OVERFLOW;
            return result;
        }

        output[output_idx] = byte;
        output_idx++;
        code++;
    }

    if (output_idx >= output_capacity) {
        result.status = COBS_OUTPUT_OVERFLOW;
        return result;
    }

    output[code_idx] = code;
    output[output_idx] = COBS_DELIMITER_BYTE;
    output_idx++;

    result.status = COBS_OK;
    result.written = output_idx;

    return result;
}

/**
 * Decodes the input data using COBS.
 *
 * @param data Buffer of bytes to decode (including the delimiter)
 * @param data_size Number of bytes to decode
 * @param output Output buffer to store the decoded data
 * @param output_capacity Maximum size of the output buffer
 * @return cobs_result_t
 */
cobs_result_t cobs_decode(const uint8_t *data, size_t data_size, uint8_t *output,
                          size_t output_capacity)
{
    cobs_result_t result = {
        .written = 0,
        .status = COBS_OK,
    };

    if (data == NULL || output == NULL) {
        result.status = COBS_NULL_POINTER;
        return result;
    }

    if (data_size < COBS_ENCODED_MIN_SIZE) {
        result.status = COBS_INPUT_TOO_SHORT;
        return result;
    }

    size_t output_idx = 0;        /**< Index to the next free byte in the output buffer */
    uint8_t code = 0xFF;          /**< Offset to the next delimiter byte */
    uint8_t remaining_bytes = 0;  /**< Remaining bytes in the block */
    bool found_delimiter = false; /**< Is parsing terminated by reaching the delimiter */

    for (size_t i = 0; i < data_size; i++) {
        uint8_t byte = data[i];

        if (byte == COBS_DELIMITER_BYTE) {
            // Delimiter appeared in the middle of the encoded data
            if (remaining_bytes != 0) {
                result.status = COBS_UNEXPECTED_DELIMITER;
                return result;
            }

            // Actual end of COBS encoded data
            found_delimiter = true;
            break;
        }

        // Following code could write one byte to the output,
        // must check if there is room in the output buffer
        if (output_idx >= output_capacity) {
            result.status = COBS_OUTPUT_OVERFLOW;
            return result;
        }

        // Data bytes within a block
        if (remaining_bytes > 0) {
            output[output_idx] = byte;
            output_idx++;

            remaining_bytes--;

            continue;
        }

        // End of a block
        if (code != 0xFF) {
            output[output_idx] = COBS_DELIMITER_BYTE;
            output_idx++;
        }

        code = byte;
        remaining_bytes = code - 1;
    }

    if (!found_delimiter) {
        result.status = COBS_MISSING_DELIMITER;
        return result;
    }

    result.status = COBS_OK;
    result.written = output_idx;

    return result;
}

/**
 * Computes the maximum encoded size for some data after COBS encoding
 * including the delimiter.
 *
 * @param data_size Size of the data
 * @return size_t
 */
size_t cobs_get_max_encoded_size(size_t data_size)
{
    if (data_size == 0) {
        return COBS_ENCODED_MIN_SIZE;
    }

    // `ceil(a / b)` implemented as `(a + b - 1) / b`
    // See https://codeforces.com/blog/entry/78852
    size_t overhead = ((data_size + 254 - 1) / 254) + 1;

    return data_size + overhead;
}
