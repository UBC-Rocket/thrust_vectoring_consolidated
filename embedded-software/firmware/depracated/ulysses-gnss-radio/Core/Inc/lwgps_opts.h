#ifndef LWGPS_OPTS_H
#define LWGPS_OPTS_H

// Use double precision for better accuracy (gps_fix_t uses double)
#define LWGPS_CFG_DOUBLE            1

// Disable status callback support (not needed for our use case)
#define LWGPS_CFG_STATUS            0

// Enable parser for GPS sentence types
#define LWGPS_CFG_STATEMENT_GPGGA   1  // Position fix
#define LWGPS_CFG_STATEMENT_GPGSA   1  // Active satellites
#define LWGPS_CFG_STATEMENT_GPGSV   1  // Satellites in view
#define LWGPS_CFG_STATEMENT_GPRMC   1  // Recommended minimum data

// Support GLONASS (GN* sentences)
#define LWGPS_CFG_STATEMENT_GNGGA   1
#define LWGPS_CFG_STATEMENT_GNGSA   1
#define LWGPS_CFG_STATEMENT_GNGSV   1
#define LWGPS_CFG_STATEMENT_GNRMC   1

#endif // LWGPS_OPTS_H
