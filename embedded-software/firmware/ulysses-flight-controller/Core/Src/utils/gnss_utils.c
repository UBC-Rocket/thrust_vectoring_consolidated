#include "utils/gnss_utils.h"
#include <stdio.h>
#include <string.h>

#define LONGITUDE_ADJUSTMENT 0.65605928066 
#define PI 3.1415f

/* Math Helpers: convert lat/lon/alt to meters relative to reference. Does not mutate gps[]. */
void longlat_to_meters(const float reference_point[3], const float longitude, const float latitude, const float altitude, float relative_distance[3]) {
    const float R = 6371000.0f;
    float delta_lat = (latitude - reference_point[0]);
    float delta_lon = (longitude - reference_point[1]);
    relative_distance[0] = R * delta_lat * PI / 180;
    relative_distance[1] = R * delta_lon * PI / 180 * (float) LONGITUDE_ADJUSTMENT;
    relative_distance[2] = altitude - reference_point[2];  /* altitude in meters */
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
