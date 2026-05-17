/**
 * @file    dev_radio_rfd900.h
 * @brief   RFD900x telemetry radio driver. Opaque singleton.
 *
 * UBC Rocket, 2026
 */
#ifndef DEV_RADIO_RFD900_H
#define DEV_RADIO_RFD900_H

#include "io_sys/io_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RADIO_RFD900_MAX_FRAME 256

typedef struct {
    uint64_t t_us;
    uint16_t len;
    uint8_t  payload[RADIO_RFD900_MAX_FRAME];
} radio_frame_t;

typedef struct radio_rfd900 radio_rfd900_t;

bool            radio_rfd900_init        (void);
radio_rfd900_t *radio_rfd900_get         (void);
size_t          radio_rfd900_drain_frames(radio_rfd900_t *d, radio_frame_t *out, size_t max);
void            radio_rfd900_set_notify  (radio_rfd900_t *d, io_task_handle_t t, uint32_t bit);
bool            radio_rfd900_send        (radio_rfd900_t *d, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* DEV_RADIO_RFD900_H */
