# SD Card Log Tools

Tools for decoding and visualizing flight log data from the SD card.

## Prerequisites

Install Python dependencies from the `tools/` directory:

```bash
cd embedded-software/firmware/tools
uv sync
```

## Decoding Binary SD Dumps

Extract the raw binary dump from the SD card, then decode it to JSONL:

```bash
# Decode a binary dump (auto-detect latest flight)
python sd-log/decode_log.py /path/to/dump.bin --latest-flight --output sd-log/logs/flight.jsonl

# Decode a specific flight by magic number
python sd-log/decode_log.py /path/to/dump.bin --flight-magic 0x5A5A5A5E --output sd-log/logs/flight.jsonl
```

The decoder validates CRC-16 checksums and outputs one JSON record per line.

## Viewing Flight Logs

Launch the interactive viewer on a decoded JSONL file:

```bash
cd embedded-software/firmware/tools/sd-log

# From a JSONL file path
uv run python -m viewer logs/flight.jsonl

# Or without arguments to get a file picker dialog
uv run python -m viewer
```

### Viewer Controls

- **Vertical scroll** (right side) — scroll through all graph rows
- **Pan** — drag left/right on any graph, or drag the highlighted region on the bottom scrubber
- **Zoom** — mouse wheel on any graph, or use the bottom zoom slider
- **Zoom slider** — logarithmic scale from microseconds to full flight duration
- **Time scrubber** — shows global time range and current viewport; drag edges to resize window

All graphs share the same time axis and stay synchronized.

### Graph Rows

The viewer displays separate rows for:

- Accelerometer (X, Y, Z)
- Gyroscope (X, Y, Z)
- Barometer 1 & 2 (pressure, temperature)
- State estimate: quaternion, position (NED), velocity (NED), angular rates
- Control output: commands, torques, attitude error, altitude
- GPS: position, quality metrics
- Events: arm/disarm, flight state changes, E-STOP (shown as vertical markers)

## Log Format

Each JSONL line contains:

```json
{
  "record_name": "accel_sample",
  "timestamp_us": 1482535,
  "payload": { "timestamp_us": 1482535, "ax_mps2": -1.64, "ay_mps2": -0.76, "az_mps2": 9.51 }
}
```

Record types are defined in `log_schema.py` (auto-generated from `log_records.h`).
