"""Auto-generated from Core/Inc/SD_logging/log_records.h
Do not edit manually. Run tools/logging/generate_log_schema.py instead.
"""

from __future__ import annotations

import struct

LOG_SCHEMA_VERSION = 3

TYPE_FORMATS = {'uint8_t': 'B', 'int8_t': 'b', 'uint16_t': 'H', 'int16_t': 'h', 'uint32_t': 'I', 'int32_t': 'i', 'uint64_t': 'Q', 'int64_t': 'q', 'float': 'f', 'double': 'd'}

RECORDS = {

    "flight_header": {
        "id": 16,
        "enum": "LOG_RECORD_TYPE_flight_header",
        "fields": [
            ("uint32_t", "timestamp_us"),
            ("uint32_t", "flight_magic"),
            ("uint32_t", "flight_counter"),
        ],
        "format": "<III",
        "struct": struct.Struct("<III"),
    },
    "accel_sample": {
        "id": 1,
        "enum": "LOG_RECORD_TYPE_accel_sample",
        "fields": [
            ("uint32_t", "timestamp_us"),
            ("float", "ax_mps2"),
            ("float", "ay_mps2"),
            ("float", "az_mps2"),
        ],
        "format": "<Ifff",
        "struct": struct.Struct("<Ifff"),
    },
    "gyro_sample": {
        "id": 2,
        "enum": "LOG_RECORD_TYPE_gyro_sample",
        "fields": [
            ("uint32_t", "timestamp_us"),
            ("float", "gx_rad_s"),
            ("float", "gy_rad_s"),
            ("float", "gz_rad_s"),
        ],
        "format": "<Ifff",
        "struct": struct.Struct("<Ifff"),
    },
    "baro_sample": {
        "id": 3,
        "enum": "LOG_RECORD_TYPE_baro_sample",
        "fields": [
            ("uint32_t", "timestamp_us"),
            ("int32_t", "temp_centi"),
            ("int32_t", "pressure_centi"),
            ("uint32_t", "seq"),
        ],
        "format": "<IiiI",
        "struct": struct.Struct("<IiiI"),
    },
    "state_snapshot": {
        "id": 4,
        "enum": "LOG_RECORD_TYPE_state_snapshot",
        "fields": [
            ("uint32_t", "timestamp_us"),
            ("float", "q_w"),
            ("float", "q_x"),
            ("float", "q_y"),
            ("float", "q_z"),
            ("float", "altitude_m"),
            ("float", "vel_n_mps"),
            ("float", "vel_e_mps"),
            ("float", "vel_d_mps"),
            ("float", "omega_bx_rad_s"),
            ("float", "omega_by_rad_s"),
            ("float", "omega_bz_rad_s"),
            ("uint8_t", "flight_state"),
            ("uint8_t", "estop_active"),
            ("uint16_t", "reserved"),
        ],
        "format": "<IfffffffffffBBH",
        "struct": struct.Struct("<IfffffffffffBBH"),
    },
    "event": {
        "id": 5,
        "enum": "LOG_RECORD_TYPE_event",
        "fields": [
            ("uint32_t", "timestamp_us"),
            ("uint16_t", "event_code"),
            ("uint16_t", "data_u16"),
        ],
        "format": "<IHH",
        "struct": struct.Struct("<IHH"),
    },
    "baro2_sample": {
        "id": 6,
        "enum": "LOG_RECORD_TYPE_baro2_sample",
        "fields": [
            ("uint32_t", "timestamp_us"),
            ("int32_t", "temp_centi"),
            ("int32_t", "pressure_centi"),
            ("uint32_t", "seq"),
        ],
        "format": "<IiiI",
        "struct": struct.Struct("<IiiI"),
    },
    "calibration": {
        "id": 9,
        "enum": "LOG_RECORD_TYPE_calibration",
        "fields": [
            ("uint32_t", "timestamp_us"),
            ("float",    "accel_bias_x"),
            ("float",    "accel_bias_y"),
            ("float",    "accel_bias_z"),
            ("float",    "gyro_bias_x"),
            ("float",    "gyro_bias_y"),
            ("float",    "gyro_bias_z"),
            ("uint16_t", "calibration_samples"),
        ],
        "format": "<IffffffH",
        "struct": struct.Struct("<IffffffH"),
    },
    "control_output": {
        "id": 8,
        "enum": "LOG_RECORD_TYPE_control_output",
        "fields": [
            ("uint32_t", "timestamp_us"),
            ("float",    "T_cmd"),
            ("float",    "theta_x_cmd"),
            ("float",    "theta_y_cmd"),
            ("float",    "tau_gim_x"),
            ("float",    "tau_gim_y"),
            ("float",    "tau_gim_z"),
            ("float",    "tau_thrust"),
        ],
        "format": "<Ifffffff",
        "struct": struct.Struct("<Ifffffff"),
    },
    "gps_fix": {
        "id": 7,
        "enum": "LOG_RECORD_TYPE_gps_fix",
        "fields": [
            ("uint32_t", "timestamp_us"),
            ("double",   "latitude"),
            ("double",   "longitude"),
            ("float",    "altitude_msl"),
            ("float",    "ground_speed"),
            ("float",    "course"),
            ("float",    "hdop"),
            ("uint32_t", "time_of_week_ms"),
            ("uint8_t",  "fix_quality"),
            ("uint8_t",  "num_satellites"),
            ("uint16_t", "reserved"),
        ],
        "format": "<IddffffIBBH",
        "struct": struct.Struct("<IddffffIBBH"),
    },
}

MAX_RECORD_SIZE = max(rec['struct'].size for rec in RECORDS.values())
