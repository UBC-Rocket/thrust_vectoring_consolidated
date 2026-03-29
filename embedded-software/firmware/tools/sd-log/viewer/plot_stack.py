"""Scrollable vertical stack of PlotStrip widgets, auto-generated from log_schema."""

from __future__ import annotations

import importlib.util
from pathlib import Path

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QScrollArea, QVBoxLayout, QWidget

from .event_strip import EventStrip
from .log_loader import FlightData
from .plot_strip import PlotStrip
from .time_axis_controller import TimeAxisController

# ── Load log_schema.py (sibling of the viewer/ package) ──
_schema_path = Path(__file__).resolve().parent.parent / "log_schema.py"
_spec = importlib.util.spec_from_file_location("log_schema", _schema_path)
_log_schema = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_log_schema)
RECORDS = _log_schema.RECORDS

# ── Configuration ──
SKIP_RECORDS = {"flight_header", "trace_batch", "trace_overflow", "event"}
SKIP_FIELDS = {"timestamp_us", "reserved", "reserved1", "reserved2", "seq"}

COLOR_PALETTE = [
    "#e6194b", "#3cb44b", "#4363d8", "#f58231",
    "#911eb4", "#42d4f4", "#f032e6", "#ffffff",
]

# Suffix → Y-axis label (checked in order, first match wins)
UNIT_SUFFIXES = [
    ("_mps2", "m/s\u00b2"),
    ("_rad_s", "rad/s"),
    ("_mps", "m/s"),
    ("_msl", "m"),
    ("_m", "m"),
    ("_centi", "centi-units"),
]


def _detect_y_label(field_names: list[str]) -> str:
    """Guess Y-axis label from the first field's suffix."""
    for name in field_names:
        for suffix, label in UNIT_SUFFIXES:
            if name.endswith(suffix):
                return label
    return ""


def _humanize(name: str) -> str:
    """accel_sample → Accel Sample"""
    return name.replace("_", " ").title()


class PlotStack(QScrollArea):
    """A vertically scrollable area containing all plot strips."""

    def __init__(
        self,
        flight_data: FlightData,
        controller: TimeAxisController,
        parent=None,
    ):
        super().__init__(parent)
        self._strips: list[PlotStrip] = []

        self.setWidgetResizable(True)
        self.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )
        self.setVerticalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOn
        )

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setSpacing(2)
        layout.setContentsMargins(0, 0, 0, 0)

        for record_name, schema in RECORDS.items():
            if record_name in SKIP_RECORDS:
                continue
            if record_name not in flight_data.groups:
                continue

            group_data = flight_data.groups[record_name]
            fields = [
                f for _, f in schema["fields"]
                if f not in SKIP_FIELDS and f in group_data.fields
            ]
            if not fields:
                continue

            colors = [COLOR_PALETTE[i % len(COLOR_PALETTE)] for i in range(len(fields))]

            strip = PlotStrip(
                title=_humanize(record_name),
                group_data=group_data,
                field_names=fields,
                colors=colors,
                y_label=_detect_y_label(fields),
                controller=controller,
                events=flight_data.events,
            )
            layout.addWidget(strip)
            self._strips.append(strip)

        # Dedicated event timeline strip
        if flight_data.events:
            event_strip = EventStrip(flight_data.events, controller)
            layout.addWidget(event_strip)

        layout.addStretch()
        self.setWidget(container)

    @property
    def strips(self) -> list[PlotStrip]:
        return self._strips
