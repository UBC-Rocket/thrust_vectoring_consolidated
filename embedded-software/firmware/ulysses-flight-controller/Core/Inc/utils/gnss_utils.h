#ifndef GNSS_UTILS_H
#define GNSS_UTILS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float lat;
    float lon;
    float alt;
    uint8_t fix_quality;
    uint8_t num_sats;
} gps_data_t;

void longlat_to_meters(const float reference_point[3], const float longitude, const float latitude, const float altitude, float relative_distance[3]);

bool parse_gpgga(const char *nmea, gps_data_t *data);

#endif
