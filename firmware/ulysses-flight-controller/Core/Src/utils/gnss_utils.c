#include "utils/gnss_utils.h"
#include <stdio.h>
#include <string.h>

void longlat_to_meters(const float reference_point[3], const float gps[3],
                        float relative_distance[3])
{
    const float R = 6371000.0f;
    float delta_lat = gps[0] - reference_point[0];
    float delta_lon = gps[1] - reference_point[1];
    relative_distance[0] = R * delta_lat;
    relative_distance[1] = R * delta_lon;
    relative_distance[2] = gps[2] - reference_point[2];
}

static float nmea_to_decimal(float nmea_coord, char quadrant) {
    int degrees = (int)(nmea_coord / 100);
    float minutes = nmea_coord - (degrees * 100);
    float decimal = degrees + (minutes / 60.0f);
    if (quadrant == 'S' || quadrant == 'W') decimal = -decimal;
    return decimal;
}

bool parse_gpgga(const char *nmea, gps_data_t *data) {
    if (strncmp(nmea, "$GPGGA", 6) != 0) return false;

    char lat_dir, lon_dir;
    float raw_lat, raw_lon;

    int count = sscanf(nmea, "$GPGGA,%*f,%f,%c,%f,%c,%hhu,%hhu,%*f,%f",
                       &raw_lat, &lat_dir, &raw_lon, &lon_dir,
                       &data->fix_quality, &data->num_sats, &data->alt);

    if (count < 7) return false;

    data->lat = nmea_to_decimal(raw_lat, lat_dir);
    data->lon = nmea_to_decimal(raw_lon, lon_dir);

    return true;
}
