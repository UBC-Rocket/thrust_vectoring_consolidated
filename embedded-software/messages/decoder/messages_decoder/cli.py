"""Thin CLI: dump an SD log to stdout as JSONL or CSV.

Exit codes:
  0  clean read (any number of records, including zero).
  1  registry mismatch (fail-closed; future: triggered by header CRC).
  2  file-level corruption past the threshold (>= 64 consecutive DecodeError
     records, or > 50% of all records are DecodeError once total > 64).

Use:
  python -m messages_decoder <log_file> [--format jsonl|csv|mf4]
                                        [--registry PATH]
                                        [--msg <module>.<name>]   # required for csv
                                        [--out <file.mf4>]        # required for mf4

  # ASAM MDF4 (viewable in asammdf/CANape, loads into pandas). Needs asammdf:
  uv run --with asammdf python -m messages_decoder flight.log \\
      --format mf4 --out flight.mf4 --registry embedded-software/messages/registry.json
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


def _export_mf4(decoder, registry, log_file: str, out_path: str) -> int:
    """Decode the SD log and write an ASAM MDF4 (.mf4) file.

    One channel group per message type, t_us_publish (→ seconds) as the time
    base, one channel per flattened field. Class-A records export; sporadic
    class-B log events are skipped (not time-series). Units come from the
    registry (flattened components inherit their parent field's unit)."""

    try:
        import numpy as np
        from asammdf import MDF, Signal
    except ImportError:
        print("error: mf4 export needs asammdf + numpy. Run e.g.:\n"
              "  uv run --with asammdf python -m messages_decoder "
              f"{log_file} --format mf4 --out {out_path}", file=sys.stderr)
        return 1

    # Map each decoded record's python type -> ("<module>.<msg>", spec), and
    # collect per-message field units keyed by the top-level field name.
    specs: dict = {}
    units_by_label: dict = {}
    for mod_name, mod in registry.modules.items():
        for msg_name, msg in (mod.get("messages") or {}).items():
            try:
                spec = decoder.get_spec(mod_name, msg_name)
            except (KeyError, ValueError):
                continue
            label = f"{mod_name}.{msg_name}"
            specs[spec.py_type] = (label, spec)
            units_by_label[label] = {
                f["name"]: f["units"] for f in msg.get("fields", []) if "units" in f
            }

    # Bucket rows by message type.
    buckets: dict = {}   # label -> {"t": [...], "cols": {name: [...]}, "n": int}
    total = errors = 0
    with open(log_file, "rb") as f:
        for rec in decoder.iter_records(f):
            total += 1
            if isinstance(rec, DecodeError):
                errors += 1
                continue
            entry = specs.get(type(rec))
            if entry is None:
                continue   # unknown / class-B / unsupported
            label, spec = entry
            b = buckets.setdefault(label, {"t": [], "cols": {}, "n": 0})
            row: dict = {}
            for fspec in spec.fields:
                _flatten_for_csv(fspec.py_name, getattr(rec, fspec.py_name), row)
            b["t"].append(rec.t_us_publish)
            for k, v in row.items():
                b["cols"].setdefault(k, []).append(v)
            b["n"] += 1

    if not buckets:
        print(f"error: no decodable records in {log_file} "
              f"({total} records, {errors} errors)", file=sys.stderr)
        return 1

    mdf = MDF()
    groups = 0
    for label, b in buckets.items():
        n = b["n"]
        t = np.asarray(b["t"], dtype=np.float64) / 1e6   # microseconds -> seconds
        units = units_by_label.get(label, {})
        sigs = []
        for col, vals in b["cols"].items():
            if len(vals) != n:
                continue                      # field wasn't present on every record
            try:
                samples = np.asarray(vals, dtype=np.float64)
            except (ValueError, TypeError):
                continue                      # non-numeric (e.g. enum *_name) — skip
            base = col.split(".", 1)[0].split("[", 1)[0]
            sigs.append(Signal(samples=samples, timestamps=t,
                               name=col, unit=units.get(base, "")))
        if sigs:
            mdf.append(sigs, comment=label, common_timebase=True)
            groups += 1

    mdf.save(out_path, overwrite=True)
    print(f"wrote {out_path}: {groups} channel group(s) from {total} records "
          f"({errors} decode errors)", file=sys.stderr)
    return 0


def main(argv: "list[str] | None" = None) -> int:
    p = argparse.ArgumentParser(prog="messages-decoder",
                                description="Decode UBC Rocket SD log records.")
    p.add_argument("log_file", help="Path to the SD log file to decode.")
    p.add_argument("--format", choices=["jsonl", "csv", "mf4"], default="jsonl",
                   help="Output format (default: jsonl). mf4 = ASAM MDF4.")
    p.add_argument("--registry", default=None,
                   help="Path to registry.json (default: alongside the package).")
    p.add_argument("--msg", default=None,
                   help="Required for --format csv: filter to a single "
                        "message, given as '<module>.<name>' (e.g. "
                        "'state_estimation.state_estimate').")
    p.add_argument("--out", default=None,
                   help="Output file path. Required for --format mf4 "
                        "(e.g. flight.mf4); jsonl/csv go to stdout.")
    args = p.parse_args(argv)

    registry_path = args.registry or _default_registry_path()
    try:
        registry = load_registry(registry_path)
    except FileNotFoundError:
        print(f"error: registry not found at {registry_path}", file=sys.stderr)
        return 1

    decoder = build_decoder(registry)

    if args.format == "mf4":
        if not args.out:
            print("error: --format mf4 requires --out <file.mf4>", file=sys.stderr)
            return 1
        return _export_mf4(decoder, registry, args.log_file, args.out)

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
