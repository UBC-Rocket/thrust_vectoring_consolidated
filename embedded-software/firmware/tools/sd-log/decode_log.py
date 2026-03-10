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
import sys
import struct
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


def iter_frames(handle: BinaryIO) -> Iterator[Tuple[int, bytes, bytes]]:
    """
    Yield (offset, header_bytes, payload_bytes) tuples for each framed record.
    """
    buffer = bytearray()
    file_offset = 0
    eof = False

    while not eof or len(buffer) >= FRAME_SIZE:
        if not eof:
            chunk = handle.read(CHUNK_SIZE)
            if chunk:
                buffer.extend(chunk)
            else:
                eof = True

        while True:
            if len(buffer) < FRAME_SIZE:
                break

            if buffer[0] != LOG_RECORD_MAGIC:
                buffer.pop(0)
                file_offset += 1
                continue

            header_bytes = bytes(buffer[:FRAME_SIZE])
            _, _, payload_len, _, _ = FRAME_STRUCT.unpack(header_bytes)
            if payload_len > MAX_RECORD_SIZE:
                # Length is clearly invalid; drop this magic byte and keep scanning.
                buffer.pop(0)
                file_offset += 1
                continue

            total_len = FRAME_SIZE + payload_len
            if len(buffer) < total_len:
                # Need more data.
                break

            payload = bytes(buffer[FRAME_SIZE:total_len])
            yield file_offset, header_bytes, payload
            del buffer[:total_len]
            file_offset += total_len

        if eof and len(buffer) < FRAME_SIZE:
            break


def decode_payload(record_id: int, payload: bytes) -> Dict[str, object] | None:
    meta = RECORDS_BY_ID.get(record_id)
    if not meta:
        return None

    struct_obj = meta["struct"]
    assert hasattr(struct_obj, "size")

    if len(payload) != struct_obj.size:  # type: ignore[attr-defined]
        return None

    values = struct_obj.unpack(payload)  # type: ignore[call-arg]
    field_names = [name for _, name in meta["fields"]]  # type: ignore[index]
    return dict(zip(field_names, values))


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
    return parser.parse_args()


def find_latest_flight_magic(path: Path) -> Optional[int]:
    if FLIGHT_HEADER_ID is None:
        return None

    latest_magic: Optional[int] = None
    with path.open("rb") as handle:
        for _, header_bytes, payload in iter_frames(handle):
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

    return latest_magic


def main() -> None:
    args = parse_args()

    if args.latest_flight and args.flight_magic is not None:
        print("--latest-flight cannot be combined with --flight-magic", file=sys.stderr)
        sys.exit(2)

    target_magic: Optional[int] = args.flight_magic
    if args.latest_flight:
        target_magic = find_latest_flight_magic(args.input)
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
            for offset, header_bytes, payload in iter_frames(handle):
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
