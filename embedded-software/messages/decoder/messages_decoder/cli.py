"""Thin CLI: dump an SD log to stdout as JSONL or CSV.

Exit codes:
  0  clean read (any number of records, including zero).
  1  registry mismatch (fail-closed; future: triggered by header CRC).
  2  file-level corruption past the threshold (>= 64 consecutive DecodeError
     records, or > 50% of all records are DecodeError once total > 64).

Use:
  python -m messages_decoder <log_file> [--format jsonl|csv]
                                        [--registry PATH]
                                        [--msg <module>.<name>]   # required for csv
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import os
import sys
from enum import IntEnum
from typing import Any, Iterable, Iterator, List

from .registry import load_registry
from .types import build_decoder
from .wire import (
    DecodeError,
    Record,
    UnknownMsgRecord,
    UnsupportedClassBRecord,
)

# Tuned per the spec: a run of >= this many consecutive DecodeError records
# is treated as catastrophic corruption.
CORRUPTION_RUN_THRESHOLD = 64
# Or: once total records > threshold and >50% are DecodeError, bail.
CORRUPTION_RATIO_MIN_TOTAL = 64
CORRUPTION_RATIO = 0.5


def _default_registry_path() -> str:
    """Best-guess: sibling of the decoder package itself."""

    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    candidate = os.path.normpath(os.path.join(here, "..", "registry.json"))
    return candidate


def _to_jsonable(obj: Any) -> Any:
    """Recursively render a decoded record as JSON-safe primitives."""

    if isinstance(obj, IntEnum):
        return {"code": int(obj), "name": obj.name}
    if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
        out = {"__type__": type(obj).__name__}
        for f in dataclasses.fields(obj):
            out[f.name] = _to_jsonable(getattr(obj, f.name))
        return out
    if isinstance(obj, (list, tuple)):
        return [_to_jsonable(v) for v in obj]
    if isinstance(obj, (bytes, bytearray)):
        return obj.hex()
    if isinstance(obj, float):
        # NaN/inf aren't valid JSON; emit as strings the consumer can parse.
        if obj != obj:
            return "NaN"
        if obj in (float("inf"), float("-inf")):
            return "Infinity" if obj > 0 else "-Infinity"
        return obj
    return obj


def _flatten_for_csv(prefix: str, obj: Any, out: "dict[str, Any]") -> None:
    """Flatten a nested dataclass into dotted column names for CSV."""

    if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
        for f in dataclasses.fields(obj):
            child = getattr(obj, f.name)
            key = f"{prefix}.{f.name}" if prefix else f.name
            _flatten_for_csv(key, child, out)
        return
    if isinstance(obj, IntEnum):
        out[prefix] = int(obj)
        out[f"{prefix}_name"] = obj.name
        return
    if isinstance(obj, (list, tuple)):
        for i, v in enumerate(obj):
            _flatten_for_csv(f"{prefix}[{i}]", v, out)
        return
    if isinstance(obj, (bytes, bytearray)):
        out[prefix] = obj.hex()
        return
    out[prefix] = obj


def _csv_columns_for_spec(spec) -> List[str]:
    """Compute the static CSV header for a message spec."""

    cols: List[str] = ["t_us_publish"]

    def walk(prefix: str, fspec) -> None:
        from .types import _FieldSpec  # local to avoid cycles
        name = f"{prefix}.{fspec.py_name}" if prefix else fspec.py_name
        if fspec.kind == "prim":
            if fspec.repeat == 1:
                cols.append(name)
            else:
                for i in range(fspec.repeat):
                    cols.append(f"{name}[{i}]")
        elif fspec.kind == "enum":
            if fspec.repeat == 1:
                cols.append(name)
                cols.append(f"{name}_name")
            else:
                for i in range(fspec.repeat):
                    cols.append(f"{name}[{i}]")
                    cols.append(f"{name}[{i}]_name")
        elif fspec.kind == "type":
            sub = fspec.py_type  # _CompositeSpec
            if fspec.repeat == 1:
                for sf in sub.fields:
                    walk(name, sf)
            else:
                for i in range(fspec.repeat):
                    for sf in sub.fields:
                        walk(f"{name}[{i}]", sf)

    for fspec in spec.fields:
        walk("", fspec)
    return cols


def main(argv: "list[str] | None" = None) -> int:
    p = argparse.ArgumentParser(prog="messages-decoder",
                                description="Decode UBC Rocket SD log records.")
    p.add_argument("log_file", help="Path to the SD log file to decode.")
    p.add_argument("--format", choices=["jsonl", "csv"], default="jsonl",
                   help="Output format (default: jsonl).")
    p.add_argument("--registry", default=None,
                   help="Path to registry.json (default: alongside the package).")
    p.add_argument("--msg", default=None,
                   help="Required for --format csv: filter to a single "
                        "message, given as '<module>.<name>' (e.g. "
                        "'state_estimation.state_estimate').")
    args = p.parse_args(argv)

    registry_path = args.registry or _default_registry_path()
    try:
        registry = load_registry(registry_path)
    except FileNotFoundError:
        print(f"error: registry not found at {registry_path}", file=sys.stderr)
        return 1

    decoder = build_decoder(registry)

    total = 0
    consecutive_errors = 0
    error_count = 0

    if args.format == "csv":
        if not args.msg:
            print("error: --format csv requires --msg <module>.<name>", file=sys.stderr)
            return 1
        try:
            module_name, msg_name = args.msg.split(".", 1)
            spec = decoder.get_spec(module_name, msg_name)
        except (ValueError, KeyError):
            print(f"error: unknown message {args.msg!r}", file=sys.stderr)
            return 1
        columns = _csv_columns_for_spec(spec)
        writer = csv.DictWriter(sys.stdout, fieldnames=columns,
                                extrasaction="ignore")
        writer.writeheader()
        with open(args.log_file, "rb") as f:
            for rec in decoder.iter_records(f):
                total += 1
                if isinstance(rec, DecodeError):
                    error_count += 1
                    consecutive_errors += 1
                    print(f"# decode-error @ offset {rec.offset}: {rec.reason}",
                          file=sys.stderr)
                else:
                    consecutive_errors = 0
                if consecutive_errors >= CORRUPTION_RUN_THRESHOLD:
                    print(f"error: {consecutive_errors} consecutive decode errors; aborting",
                          file=sys.stderr)
                    return 2
                if (total > CORRUPTION_RATIO_MIN_TOTAL
                        and error_count / total > CORRUPTION_RATIO):
                    print(f"error: {error_count}/{total} records corrupt; aborting",
                          file=sys.stderr)
                    return 2
                if isinstance(rec, (DecodeError, UnsupportedClassBRecord, UnknownMsgRecord)):
                    continue
                if not isinstance(rec, spec.py_type):
                    continue
                row: dict[str, Any] = {"t_us_publish": rec.t_us_publish}
                for fspec in spec.fields:
                    _flatten_for_csv(fspec.py_name, getattr(rec, fspec.py_name), row)
                writer.writerow(row)
        return 0

    # jsonl
    with open(args.log_file, "rb") as f:
        for rec in decoder.iter_records(f):
            total += 1
            if isinstance(rec, DecodeError):
                error_count += 1
                consecutive_errors += 1
                print(json.dumps({
                    "__type__": "DecodeError",
                    "offset": rec.offset,
                    "reason": rec.reason,
                }))
            else:
                consecutive_errors = 0
                print(json.dumps(_to_jsonable(rec)))
            if consecutive_errors >= CORRUPTION_RUN_THRESHOLD:
                print(f"error: {consecutive_errors} consecutive decode errors; aborting",
                      file=sys.stderr)
                return 2
            if (total > CORRUPTION_RATIO_MIN_TOTAL
                    and error_count / total > CORRUPTION_RATIO):
                print(f"error: {error_count}/{total} records corrupt; aborting",
                      file=sys.stderr)
                return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
