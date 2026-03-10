# Tools

Host-side utilities for development and post-flight analysis.

## Setup

Dependencies are managed with [uv](https://docs.astral.sh/uv/). Install uv, then:

```bash
cd firmware/tools
uv sync
```

## sd-log

Python scripts for decoding binary SD card logs from the flight controller.

```bash
# Decode a log file
uv run sd-log/decode_log.py path/to/log_dump.bin -o output.jsonl

# Regenerate log schema from firmware header
uv run sd-log/generate_log_schema.py
```

- `decode_log.py` - Decode binary log files into human-readable NDJSON
- `generate_log_schema.py` - Generate `log_schema.py` from firmware's `log_records.h`
- `log_schema.py` - Auto-generated log record format definitions

## serial

```bash
# Start serial debug logger
uv run serial/debug_logger.py -p /dev/tty.usbmodem102
```

- `debug_logger.py` - Serial port debug output logger (requires pyserial)
