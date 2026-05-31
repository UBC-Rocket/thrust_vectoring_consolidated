#!/usr/bin/env python3
"""
Decode raw SD-card log dumps into JSON/NDJSON using the shared schema.

Example:
    python tools/SD/decode_log.py
    python tools/SD/decode_log.py /tmp/log_dump.bin --output /tmp/flight.jsonl
"""

from __future__ import annotations

import argparse
import json
import stat
import sys
import struct
import time
from pathlib import Path
from typing import BinaryIO, Dict, Iterator, Optional, Tuple

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from log_schema import (  # type: ignore  # pylint: disable=wrong-import-position
    LOG_SCHEMA_VERSION,
    MAX_RECORD_SIZE,
    RECORDS,
)

INPUT_DIR = SCRIPT_DIR / "bin"
OUTPUT_DIR = SCRIPT_DIR / "logs"
DEFAULT_INPUT = INPUT_DIR / "log_dump.bin"
DEFAULT_OUTPUT = OUTPUT_DIR / "flight.jsonl"

FRAME_STRUCT = struct.Struct("<BBHHH")
FRAME_SIZE = FRAME_STRUCT.size
LOG_RECORD_MAGIC = 0xA5
CHUNK_SIZE = 8192

# Firmware erases/writes the first 128 MB of the SD card.
LOG_ERASE_BYTES = 262144 * 512  # 128 MB

# Max payload size: a full trace sector = 512 - 8 (frame header) = 504 bytes.
# MAX_RECORD_SIZE from log_schema only covers fixed-size records, so we use
# the larger of the two.
MAX_PAYLOAD_SIZE = max(MAX_RECORD_SIZE, 504)

RECORDS_BY_ID: Dict[int, Dict[str, object]] = {
    meta["id"]: {**meta, "name": name}
    for name, meta in RECORDS.items()
}
FLIGHT_HEADER_ID: Optional[int] = next(
    (meta["id"] for name, meta in RECORDS.items() if name == "flight_header"),
    None,
)


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) & 0xFFFF) ^ 0x1021
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


def _is_block_device(path: Path) -> bool:
    """Check if path is a block/character device (raw SD card)."""
    try:
        mode = path.stat().st_mode
        return stat.S_ISBLK(mode) or stat.S_ISCHR(mode)
    except OSError:
        return False


def _fmt_bytes(n: int) -> str:
    if n >= 1_048_576:
        return f"{n / 1_048_576:.1f} MB"
    if n >= 1024:
        return f"{n / 1024:.1f} KB"
    return f"{n} B"


def iter_frames(
    handle: BinaryIO,
    max_bytes: int = 0,
    progress_label: str = "",
) -> Iterator[Tuple[int, bytes, bytes]]:
    """
    Yield (offset, header_bytes, payload_bytes) tuples for each framed record.

    max_bytes: stop reading after this many bytes (0 = unlimited).
    progress_label: if set, print progress to stderr periodically.
    """
    buffer = bytearray()
    file_offset = 0
    bytes_read = 0
    eof = False
    last_progress = 0.0

    while not eof or len(buffer) >= FRAME_SIZE:
        if not eof:
            read_size = CHUNK_SIZE
            if max_bytes > 0:
                remaining = max_bytes - bytes_read
                if remaining <= 0:
                    eof = True
                    read_size = 0
                else:
                    read_size = min(read_size, remaining)

            if read_size > 0:
                chunk = handle.read(read_size)
                if chunk:
                    buffer.extend(chunk)
                    bytes_read += len(chunk)
                else:
                    eof = True

            if progress_label and max_bytes > 0:
                now = time.monotonic()
                if now - last_progress >= 0.5 or eof:
                    pct = bytes_read * 100 / max_bytes
                    print(
                        f"\r  {progress_label}: {_fmt_bytes(bytes_read)}"
                        f" / {_fmt_bytes(max_bytes)} ({pct:.0f}%)",
                        end="", file=sys.stderr, flush=True,
                    )
                    last_progress = now

        while True:
            if len(buffer) < FRAME_SIZE:
                break

            if buffer[0] != LOG_RECORD_MAGIC:
                buffer.pop(0)
                file_offset += 1
                continue

            header_bytes = bytes(buffer[:FRAME_SIZE])
            _, _, payload_len, _, _ = FRAME_STRUCT.unpack(header_bytes)
            if payload_len > MAX_PAYLOAD_SIZE:
                buffer.pop(0)
                file_offset += 1
                continue

            total_len = FRAME_SIZE + payload_len
            if len(buffer) < total_len:
                break

            payload = bytes(buffer[FRAME_SIZE:total_len])
            yield file_offset, header_bytes, payload
            del buffer[:total_len]
            file_offset += total_len

        if eof and len(buffer) < FRAME_SIZE:
            break

    if progress_label:
        print(file=sys.stderr)


TRACE_BATCH_ID: Optional[int] = next(
    (meta["id"] for name, meta in RECORDS.items() if name == "trace_batch"),
    None,
)
TRACE_EVENT_STRUCT = struct.Struct("<IHBBI")  # timestamp_us, task_number, event_type, reserved, aux


def decode_payload(record_id: int, payload: bytes) -> Dict[str, object] | None:
    meta = RECORDS_BY_ID.get(record_id)
    if not meta:
        return None

    struct_obj = meta["struct"]
    assert hasattr(struct_obj, "size")

    if len(payload) < struct_obj.size:  # type: ignore[attr-defined]
        return None

    # Decode the fixed header fields
    values = struct_obj.unpack(payload[:struct_obj.size])  # type: ignore[call-arg]
    field_names = [name for _, name in meta["fields"]]  # type: ignore[index]
    result = dict(zip(field_names, values))

    # For trace_batch, decode the trailing event array
    if record_id == TRACE_BATCH_ID and len(payload) > struct_obj.size:
        event_data = payload[struct_obj.size:]
        events = []
        evt_size = TRACE_EVENT_STRUCT.size
        for i in range(0, len(event_data) - evt_size + 1, evt_size):
            ts, task_num, evt_type, _, aux = TRACE_EVENT_STRUCT.unpack(
                event_data[i:i + evt_size])
            events.append({
                "timestamp_us": ts,
                "task_number": task_num,
                "event_type": evt_type,
                "aux": aux,
            })
        result["events"] = events

    return result


def emit_json(obj: Dict[str, object], writer, pretty: bool) -> None:
    if pretty:
        json.dump(obj, writer, indent=2)
        writer.write("\n")
    else:
        json.dump(obj, writer, separators=(",", ":"))
        writer.write("\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Decode raw SD log (sector dump) into JSON records."
    )
    parser.add_argument(
        "input",
        nargs="?",
        default=DEFAULT_INPUT,
        type=Path,
        help=f"Path to raw log binary (default: {DEFAULT_INPUT})",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"Destination file for JSON (default: {DEFAULT_OUTPUT})",
    )
    parser.add_argument(
        "--pretty",
        action="store_true",
        help="Pretty-print JSON instead of newline-delimited JSON.",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=None,
        help="Stop after decoding this many valid records.",
    )
    parser.add_argument(
        "--include-bad-crc",
        action="store_true",
        help="Emit JSON objects for records with CRC mismatches (flagged in output).",
    )
    parser.add_argument(
        "--flight-magic",
        type=lambda value: int(value, 0),
        help="Only emit records for the flight matching this magic value (hex or int).",
    )
    parser.add_argument(
        "--latest-flight",
        action="store_true",
        help="Auto-detect the newest flight header and emit only that flight.",
    )
    parser.add_argument(
        "--read-bytes",
        type=lambda v: int(v, 0) if v.startswith("0") else int(v),
        default=None,
        help=(
            "Maximum bytes to read from input. Defaults to 64 MB for block "
            "devices (matching LOG_ERASE_BLOCK_COUNT), unlimited for files."
        ),
    )
    return parser.parse_args()


def find_latest_flight_magic(path: Path, max_bytes: int = 0) -> Optional[int]:
    if FLIGHT_HEADER_ID is None:
        return None

    print("Scanning for flight headers...", file=sys.stderr)
    latest_magic: Optional[int] = None
    header_count = 0
    with path.open("rb") as handle:
        for _, header_bytes, payload in iter_frames(
            handle, max_bytes=max_bytes, progress_label="Scanning"
        ):
            _, record_type, payload_len, stored_crc, _ = FRAME_STRUCT.unpack(
                header_bytes
            )
            if record_type != FLIGHT_HEADER_ID:
                continue

            header_for_crc = bytearray(header_bytes)
            header_for_crc[4:6] = b"\x00\x00"
            computed_crc = crc16_ccitt(header_for_crc + payload)
            if computed_crc != stored_crc or payload_len != len(payload):
                continue

            payload_dict = decode_payload(record_type, payload)
            if payload_dict is None:
                continue

            magic_value = payload_dict.get("flight_magic")
            if isinstance(magic_value, int):
                latest_magic = magic_value
                header_count += 1
                print(
                    f"  Found flight header #{header_count}: "
                    f"magic=0x{magic_value:08X}",
                    file=sys.stderr,
                )

    print(
        f"Scan complete: {header_count} flight header(s) found.",
        file=sys.stderr,
    )
    return latest_magic


def main() -> None:
    args = parse_args()

    if args.latest_flight and args.flight_magic is not None:
        print("--latest-flight cannot be combined with --flight-magic", file=sys.stderr)
        sys.exit(2)

    # Determine read limit
    if args.read_bytes is not None:
        max_bytes = args.read_bytes
    elif _is_block_device(args.input):
        max_bytes = LOG_ERASE_BYTES
        print(
            f"Block device detected — limiting read to {_fmt_bytes(max_bytes)} "
            f"(firmware erase region). Override with --read-bytes.",
            file=sys.stderr,
        )
    else:
        max_bytes = 0

    print(f"Input: {args.input}", file=sys.stderr)

    target_magic: Optional[int] = args.flight_magic
    if args.latest_flight:
        target_magic = find_latest_flight_magic(args.input, max_bytes=max_bytes)
        if target_magic is None:
            print("Unable to locate a flight_header in the log.", file=sys.stderr)
            sys.exit(1)
    if target_magic is not None:
        print(
            f"Filtering for flight magic 0x{target_magic:08X}",
            file=sys.stderr,
        )

    output_path = args.output
    output_to_stdout = str(output_path) == "-" if output_path else True
    if not output_to_stdout:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        writer = output_path.open("w", encoding="utf-8")
    else:
        writer = sys.stdout

    stats = {
        "decoded": 0,
        "crc_fail": 0,
        "unknown_type": 0,
        "length_mismatch": 0,
    }

    emitting_target = target_magic is None
    target_seen = False

    try:
        with args.input.open("rb") as handle:
            for offset, header_bytes, payload in iter_frames(
                handle, max_bytes=max_bytes, progress_label="Decoding"
            ):
                magic, record_type, payload_len, stored_crc, _ = FRAME_STRUCT.unpack(
                    header_bytes
                )
                assert magic == LOG_RECORD_MAGIC

                header_for_crc = bytearray(header_bytes)
                header_for_crc[4:6] = b"\x00\x00"
                computed_crc = crc16_ccitt(header_for_crc + payload)
                if computed_crc != stored_crc:
                    stats["crc_fail"] += 1
                    if not args.include_bad_crc:
                        continue

                payload_dict = decode_payload(record_type, payload)
                if payload_dict is None:
                    if record_type not in RECORDS_BY_ID:
                        stats["unknown_type"] += 1
                    else:
                        stats["length_mismatch"] += 1

                record_obj = {
                    "offset": offset,
                    "record_id": record_type,
                    "schema_version": LOG_SCHEMA_VERSION,
                    "crc_ok": computed_crc == stored_crc,
                    "payload_length": payload_len,
                }

                meta = RECORDS_BY_ID.get(record_type)
                if meta:
                    record_obj["record_name"] = meta["name"]
                    record_obj["record_enum"] = meta["enum"]

                is_flight_header = (
                    FLIGHT_HEADER_ID is not None
                    and record_type == FLIGHT_HEADER_ID
                    and payload_dict is not None
                )

                if is_flight_header:
                    magic_value = payload_dict.get("flight_magic")
                    if isinstance(magic_value, int):
                        record_obj["flight_magic"] = magic_value
                        if target_magic is None:
                            emitting_target = True
                        else:
                            if magic_value == target_magic:
                                emitting_target = True
                                target_seen = True
                            elif target_seen:
                                break
                            else:
                                emitting_target = False

                if payload_dict is not None:
                    record_obj["payload"] = payload_dict
                    timestamp = payload_dict.get("timestamp_us")
                    if timestamp is None:
                        timestamp = payload_dict.get("base_timestamp_us")
                    if isinstance(timestamp, (int, float)):
                        record_obj["timestamp_us"] = timestamp
                else:
                    record_obj["payload_raw"] = payload.hex()

                if target_magic is not None and not emitting_target:
                    continue

                emit_json(record_obj, writer, args.pretty)
                stats["decoded"] += 1

                if args.limit and stats["decoded"] >= args.limit:
                    break
    finally:
        if writer is not sys.stdout:
            writer.close()

    summary = (
        "Decoded {decoded} records "
        "(CRC fail: {crc_fail}, unknown type: {unknown_type}, "
        "length mismatch: {length_mismatch})"
    ).format(**stats)
    print(summary, file=sys.stderr)


if __name__ == "__main__":
    main()
