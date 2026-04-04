"""A single graph row showing one group of time-series data."""

from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import QEvent
from PyQt6.QtWidgets import QApplication, QScrollArea, QVBoxLayout, QWidget

from .downsampler import downsample_minmax
from .log_loader import EventRecord, GroupData
from .time_axis_controller import TimeAxisController


class PlotStrip(QWidget):
    """One row in the vertically stacked plot view."""

    MIN_HEIGHT = 200

    def __init__(
        self,
        title: str,
        group_data: GroupData,
        field_names: list[str],
        colors: list[str],
        y_label: str,
        controller: TimeAxisController,
        events: list[EventRecord] | None = None,
        parent=None,
    ):
        super().__init__(parent)
        self._controller = controller
        self._group_data = group_data
        self._field_names = field_names
        self._updating = False

        self.setMinimumHeight(self.MIN_HEIGHT)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self._plot_widget = pg.PlotWidget(title=title)
        self._plot_widget.setLabel("left", y_label)
        self._plot_widget.setLabel("bottom", "Time", units="s")
        self._plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self._plot_widget.setMouseEnabled(x=False, y=False)
        self._plot_widget.getViewBox().setMenuEnabled(False)
        # Forward wheel events to parent scroll area instead of pyqtgraph consuming them
        self._plot_widget.viewport().installEventFilter(self)
        layout.addWidget(self._plot_widget)

        # Legend must be added before curves so it auto-captures names
        self._plot_widget.addLegend(offset=(10, 10))

        # Create curves
        self._curves: list[pg.PlotDataItem] = []
        for i, field_name in enumerate(field_names):
            color = colors[i % len(colors)]
            curve = self._plot_widget.plot(
                pen=pg.mkPen(color, width=1.5), name=field_name
            )
            self._curves.append(curve)

        # Add event markers as vertical lines
        self._event_lines: list[pg.InfiniteLine] = []
        for ev in (events or []):
            line = pg.InfiniteLine(
                pos=ev.time_s,
                angle=90,
                pen=pg.mkPen("#ffff00", width=1, style=pg.QtCore.Qt.PenStyle.DashLine),
                label=ev.label,
                labelOpts={"position": 0.95, "color": "#ffff00", "fill": "#333333"},
            )
            self._plot_widget.addItem(line)
            self._event_lines.append(line)

        # Connect to controller
        controller.viewChanged.connect(self._on_view_changed)

        # Initial data + range
        self._update_data(controller.view_min, controller.view_max)

    def eventFilter(self, obj, event):
        """Intercept wheel events from pyqtgraph and forward to scroll area."""
        if event.type() == QEvent.Type.Wheel:
            parent = self.parent()
            while parent and not isinstance(parent, QScrollArea):
                parent = parent.parent()
            if parent:
                QApplication.sendEvent(parent.verticalScrollBar(), event)
            return True
        return super().eventFilter(obj, event)

    def _on_view_changed(self, vmin: float, vmax: float):
        """Controller says the view changed — update our X range and data."""
        if self._updating:
            return
        self._updating = True
        self._plot_widget.setXRange(vmin, vmax, padding=0)
        self._update_data(vmin, vmax)
        self._updating = False

    def _update_data(self, vmin: float, vmax: float):
        """Downsample and set data for the visible range."""
        n_pixels = max(self.width(), 400)
        ts = self._group_data.timestamps

        for i, field_name in enumerate(self._field_names):
            if field_name not in self._group_data.fields:
                continue
            y = self._group_data.fields[field_name]
            t_ds, y_ds = downsample_minmax(ts, y, vmin, vmax, n_pixels)
            self._curves[i].setData(t_ds, y_ds)

        # Auto-range Y to visible data
        self._auto_range_y(vmin, vmax)

    def _auto_range_y(self, vmin: float, vmax: float):
        """Set Y range to fit the visible data with some padding."""
        ts = self._group_data.timestamps
        i0 = int(np.searchsorted(ts, vmin, side="left"))
        i1 = int(np.searchsorted(ts, vmax, side="right"))
        if i0 >= i1:
            return

        y_min = float("inf")
        y_max = float("-inf")
        for field_name in self._field_names:
            if field_name not in self._group_data.fields:
                continue
            y_slice = self._group_data.fields[field_name][i0:i1]
            if len(y_slice) > 0:
                y_min = min(y_min, float(np.min(y_slice)))
                y_max = max(y_max, float(np.max(y_slice)))

        if y_min < y_max:
            margin = (y_max - y_min) * 0.05
            self._plot_widget.setYRange(y_min - margin, y_max + margin, padding=0)
