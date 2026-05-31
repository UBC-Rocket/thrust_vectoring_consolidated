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

    A new flight starts at each flight_header record.  Backward timestamp
    jumps within a flight are normal (different sensors log at different
    rates and may interleave out of order).
    """
    flights: list[list[dict]] = []
    current: list[dict] = []

    for rec in records:
        is_header = rec.get("record_name") == "flight_header"

        if is_header and current:
            flights.append(current)
            current = []

        current.append(rec)

    if current:
        flights.append(current)

    return flights


EVENT_CODES = {
    0x0001: "E-STOP",
    0x0002: "FLIGHT_STATE",
    0x0004: "ARM_STATE",
}

TASK_NAMES = {
    0: "Idle",
    1: "MissionMgr",
    2: "Controls",
    3: "StateEst",
    4: "DebugLog",
    5: "SdFlush",
}

TRACE_EVT_SWITCH_IN = 0
TRACE_EVT_SWITCH_OUT = 1


def _compute_trace_groups(
    records: list[dict],
    t_min_s: float,
    t_max_s: float,
    window_ms: float = 100.0,
) -> dict[str, "GroupData"]:
    """Compute CPU utilization and controls jitter from trace_batch records."""
    # Extract all trace events
    events: list[tuple[float, int, int]] = []  # (time_s, task_number, event_type)
    for rec in records:
        if rec.get("record_name") != "trace_batch":
            continue
        payload = rec.get("payload", {})
        for evt in payload.get("events", []):
            events.append((
                evt["timestamp_us"] / 1e6,
                evt["task_number"],
                evt["event_type"],
            ))

    if not events:
        return {}

    events.sort(key=lambda e: e[0])

    # ── CPU utilization per task in sliding windows ──
    window_s = window_ms / 1000.0
    window_start = t_min_s
    window_times: list[float] = []

    # Track switch-in time per task
    task_switch_in: dict[int, float] = {}
    # Accumulate CPU time per task per window
    task_ids = sorted({e[1] for e in events})
    task_cpu: dict[int, list[float]] = {tid: [] for tid in task_ids}
    task_accum: dict[int, float] = {tid: 0.0 for tid in task_ids}

    evt_idx = 0
    while window_start + window_s <= t_max_s:
        window_end = window_start + window_s

        # Process events in this window
        while evt_idx < len(events) and events[evt_idx][0] < window_end:
            t, tid, etype = events[evt_idx]
            if etype == TRACE_EVT_SWITCH_IN:
                task_switch_in[tid] = t
            elif etype == TRACE_EVT_SWITCH_OUT:
                if tid in task_switch_in:
                    run_time = t - task_switch_in[tid]
                    if run_time > 0:
                        task_accum[tid] = task_accum.get(tid, 0.0) + run_time
                    del task_switch_in[tid]
            evt_idx += 1

        # Record utilization as percentage
        window_times.append(window_start + window_s / 2)
        for tid in task_ids:
            pct = min(task_accum.get(tid, 0.0) / window_s * 100.0, 100.0)
            task_cpu[tid].append(pct)
            task_accum[tid] = 0.0

        window_start += window_s

    groups: dict[str, GroupData] = {}

    if window_times:
        ts_arr = np.array(window_times, dtype=np.float64)
        fields = {}
        for tid in task_ids:
            name = TASK_NAMES.get(tid, f"Task{tid}")
            fields[name] = np.array(task_cpu[tid], dtype=np.float64)
        groups["cpu_utilization"] = GroupData(timestamps=ts_arr, fields=fields)

    # ── Controls task period jitter ──
    controls_task_id = 2  # Controls task number
    controls_switch_ins = [
        e[0] for e in events
        if e[1] == controls_task_id and e[2] == TRACE_EVT_SWITCH_IN
    ]

    if len(controls_switch_ins) > 2:
        switch_in_arr = np.array(controls_switch_ins, dtype=np.float64)
        periods_us = np.diff(switch_in_arr) * 1e6  # convert to microseconds
        # Use median period as the "expected" period
        expected_us = float(np.median(periods_us))
        jitter_us = periods_us - expected_us
        jitter_ts = (switch_in_arr[:-1] + switch_in_arr[1:]) / 2  # midpoints

        groups["controls_jitter"] = GroupData(
            timestamps=jitter_ts,
            fields={"jitter_us": jitter_us},
        )

    return groups


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

        if record_name in ("calibration", "trace_batch", "trace_overflow"):
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

    # Compute derived trace groups (CPU utilization, jitter)
    trace_groups = _compute_trace_groups(records, global_min, global_max)
    groups.update(trace_groups)

    return FlightData(
        groups=groups,
        events=events,
        t_min=global_min,
        t_max=global_max,
        flight_magic=flight_magic,
        flight_counter=flight_counter,
    )
