/**
 * @file gps_fix.h
 * @brief Shared GPS fix structure for inter-board SPI communication.
 *
 * This header defines the canonical GPS fix structure used for communication
 * between the GNSS radio (slave) and the flight controller (master) over SPI.
 *
 * Both boards MUST use this definition to ensure wire compatibility.
 * Total size: 48 bytes, packed.
 *
 * @ UBC Rocket, 2026
 */

#ifndef GNSS_PROTOCOL_GPS_FIX_H
#define GNSS_PROTOCOL_GPS_FIX_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Parsed GPS fix structure transmitted over SPI (push mode).
 *
 * Layout is fixed at 48 bytes packed. Do not reorder or resize fields
 * without updating both the GNSS radio slave and flight controller master.
 */
typedef struct {
    /* Position */
    double   latitude;         /**< Degrees, signed (-90 to +90)          [8 bytes] */
    double   longitude;        /**< Degrees, signed (-180 to +180)        [8 bytes] */
    float    altitude_msl;     /**< Meters above sea level                [4 bytes] */

    /* Velocity & Heading */
    float    ground_speed;     /**< m/s                                   [4 bytes] */
    float    course;           /**< Degrees true north (0-360)            [4 bytes] */

    /* Quality indicators */
    uint8_t  fix_quality;      /**< 0=invalid, 1=GPS, 2=DGPS, etc.        [1 byte] */
    uint8_t  num_satellites;   /**< Number of satellites used             [1 byte] */
    uint8_t  padding1[2];      /**< Alignment padding                     [2 bytes] */
    float    hdop;             /**< Horizontal dilution of precision      [4 bytes] */

    /* Timestamp */
    uint32_t time_of_week_ms;  /**< GPS time of week in milliseconds      [4 bytes] */

    uint8_t  padding2[8];      /**< Reserved / padding to 48 bytes        [8 bytes] */
} __attribute__((packed)) gps_fix_t;

#define GPS_FIX_SIZE 48  /**< Wire size of gps_fix_t (static assert target) */

#ifdef __cplusplus
}
#endif

#endif /* GNSS_PROTOCOL_GPS_FIX_H */
