#ifndef RP_COBS_H
#define RP_COBS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define COBS_DELIMITER_BYTE (0x00)
#define COBS_ENCODED_MIN_SIZE (2)

typedef enum cobs_status {
    COBS_OK,
    COBS_NULL_POINTER,
    COBS_OUTPUT_OVERFLOW,
    COBS_INPUT_TOO_SHORT,
    COBS_UNEXPECTED_DELIMITER,
    COBS_MISSING_DELIMITER,
} cobs_status_t;

typedef struct cobs_result {
    size_t written;       /**< Number of bytes written to the output buffer */
    cobs_status_t status; /**< Status of the operation */
} cobs_result_t;

cobs_result_t cobs_encode(const uint8_t *data, size_t data_size, uint8_t *output,
                          size_t output_capacity);

cobs_result_t cobs_decode(const uint8_t *data, size_t data_size, uint8_t *output,
                          size_t output_capacity);

size_t cobs_get_max_encoded_size(size_t data_size);

#endif // RP_COBS_H
