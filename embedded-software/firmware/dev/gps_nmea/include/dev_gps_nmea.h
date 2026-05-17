/**
 * @file    dev_gps_nmea.h
 * @brief   NMEA GPS driver. Opaque singleton.
 *
 * UBC Rocket, 2026
 */
#ifndef DEV_GPS_NMEA_H
#define DEV_GPS_NMEA_H

#include "io_sys/io_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t t_us;
    double   latitude_deg;
    double   longitude_deg;
    float    altitude_m;
    float    ground_speed_mps;
    float    course_deg;
    float    hdop;
    uint8_t  fix_type;
    uint8_t  sats_used;
    bool     valid;
} gps_fix_t;

typedef struct gps_nmea gps_nmea_t;

bool        gps_nmea_init       (void);
gps_nmea_t *gps_nmea_get        (void);
size_t      gps_nmea_drain_fixes(gps_nmea_t *d, gps_fix_t *out, size_t max);
void        gps_nmea_set_notify (gps_nmea_t *d, io_task_handle_t t, uint32_t bit);

#ifdef __cplusplus
}
#endif

#endif /* DEV_GPS_NMEA_H */
