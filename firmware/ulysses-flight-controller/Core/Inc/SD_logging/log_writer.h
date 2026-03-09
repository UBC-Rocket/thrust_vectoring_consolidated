#ifndef LOG_WRITER_H
#define LOG_WRITER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "SD_logging/log_records.h"

/**
 * @brief Initialise the SD log writer subsystem.
 *
 * Must be called after peripherals are set up. Returns false if the SD card
 * is unavailable or initialisation failed.
 */
bool log_writer_init(void);

/**
 * @brief Check whether the logger is ready to accept records.
 */
bool log_writer_ready(void);

/**
 * @brief Append a typed record to the log stream.
 *
 * @param type    Record type identifier.
 * @param payload Pointer to payload matching the schema for @p type.
 * @param payload_size Size of payload in bytes.
 * @return true if queued successfully, false otherwise.
 */
bool log_writer_append_record(log_record_type_t type,
                              const void *payload,
                              size_t payload_size);

/**
 * @brief Force any pending data to be written to the SD card.
 *
 * Pads the active buffer to the nearest sector and waits for the DMA transfer
 * to complete. Safe to call periodically or before shutdown.
 *
 * @return true if the flush succeeded, false otherwise.
 */
bool log_writer_flush(void);

#endif /* LOG_WRITER_H */
