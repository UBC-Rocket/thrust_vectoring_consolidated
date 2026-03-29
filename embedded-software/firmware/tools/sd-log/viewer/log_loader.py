"""Loads a JSONL flight log into numpy arrays grouped by record type."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class GroupData:
    """Timestamps and field arrays for one record type."""
    timestamps: np.ndarray  # float64 seconds
    fields: dict[str, np.ndarray]  # field_name -> values


@dataclass
class EventRecord:
    """A single discrete event."""
    time_s: float
    event_code: int
    data: int
    label: str


@dataclass
class FlightData:
    """All parsed data from a JSONL flight log."""
    groups: dict[str, GroupData]  # record_name -> GroupData
    events: list[EventRecord]
    t_min: float
    t_max: float
    flight_magic: int = 0
    flight_counter: int = 0


def _split_flights(records: list[dict]) -> list[list[dict]]:
    """Split a flat list of parsed JSONL records into per-flight segments.

    A new flight starts at each flight_header record, or when timestamps
    jump backward by more than 1 second (stale data from a previous flight
    on the SD card).
    """
    flights: list[list[dict]] = []
    current: list[dict] = []
    prev_ts = 0

    for rec in records:
        ts = rec.get("timestamp_us", 0)
        is_header = rec.get("record_name") == "flight_header"

        # Detect flight boundary: header or large backward timestamp jump
        if is_header and current:
            flights.append(current)
            current = []
            prev_ts = 0
        elif not is_header and ts < prev_ts - 1_000_000:
            # Backward jump > 1 second = stale data from old flight
            flights.append(current)
            current = []
            prev_ts = 0

        current.append(rec)
        if not is_header:
            prev_ts = ts

    if current:
        flights.append(current)

    return flights


EVENT_CODES = {
    0x0001: "E-STOP",
    0x0002: "FLIGHT_STATE",
    0x0004: "ARM_STATE",
}


def load_jsonl(path: Path) -> FlightData:
    """Parse a JSONL flight log, extracting only the first flight (with header)."""

    # Read all records
    all_records: list[dict] = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            all_records.append(json.loads(line))

    # Split into flights and use the first one (the one written from
    # the base of the SD card with a valid flight_header)
    flights = _split_flights(all_records)
    if not flights:
        return FlightData(
            groups={}, events=[], t_min=0.0, t_max=0.0
        )

    records = flights[0]

    # Parse the selected flight
    raw: dict[str, dict[str, list]] = {}
    raw_events: list[tuple[float, int, int]] = []
    flight_magic = 0
    flight_counter = 0

    for record in records:
        record_name = record.get("record_name", "")
        payload = record.get("payload", {})
        ts_us = record.get("timestamp_us", 0)
        ts_s = ts_us / 1e6

        if record_name == "flight_header":
            flight_magic = payload.get("flight_magic", 0)
            flight_counter = payload.get("flight_counter", 0)
            continue

        if record_name == "event":
            code = payload.get("event_code", 0)
            data = payload.get("data_u16", 0)
            raw_events.append((ts_s, code, data))
            continue

        if record_name == "calibration":
            continue

        # Accumulate time-series data
        if record_name not in raw:
            raw[record_name] = {"_ts": []}
            for key in payload:
                if key != "timestamp_us":
                    raw[record_name][key] = []

        raw[record_name]["_ts"].append(ts_s)
        for key, val in payload.items():
            if key != "timestamp_us" and key in raw[record_name]:
                raw[record_name][key].append(val)

    # Convert to numpy arrays
    groups: dict[str, GroupData] = {}
    global_min = float("inf")
    global_max = float("-inf")

    for name, data in raw.items():
        ts = np.array(data.pop("_ts"), dtype=np.float64)
        fields = {k: np.array(v, dtype=np.float64) for k, v in data.items()}
        groups[name] = GroupData(timestamps=ts, fields=fields)
        if len(ts) > 0:
            global_min = min(global_min, ts[0])
            global_max = max(global_max, ts[-1])

    # Build event list
    events = []
    for ts_s, code, data in raw_events:
        label = EVENT_CODES.get(code, f"0x{code:04X}")
        events.append(EventRecord(
            time_s=ts_s, event_code=code, data=data, label=label
        ))
        global_min = min(global_min, ts_s)
        global_max = max(global_max, ts_s)

    if global_min == float("inf"):
        global_min = 0.0
        global_max = 0.0

    return FlightData(
        groups=groups,
        events=events,
        t_min=global_min,
        t_max=global_max,
        flight_magic=flight_magic,
        flight_counter=flight_counter,
    )
