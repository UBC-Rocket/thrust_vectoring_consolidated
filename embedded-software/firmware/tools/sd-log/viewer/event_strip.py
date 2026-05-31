"""Dedicated event timeline strip showing discrete events as labeled markers."""

from __future__ import annotations

import pyqtgraph as pg
from PyQt6.QtCore import QEvent
from PyQt6.QtWidgets import QApplication, QScrollArea, QVBoxLayout, QWidget

from .log_loader import EventRecord
from .time_axis_controller import TimeAxisController

EVENT_COLORS = {
    "E-STOP": "#ff4444",
    "FLIGHT_STATE": "#44ff44",
    "ARM_STATE": "#4488ff",
}


class EventStrip(QWidget):
    """A plot row showing events as vertical markers on a timeline."""

    MIN_HEIGHT = 120

    def __init__(
        self,
        events: list[EventRecord],
        controller: TimeAxisController,
        parent=None,
    ):
        super().__init__(parent)
        self._controller = controller
        self._updating = False

        self.setMinimumHeight(self.MIN_HEIGHT)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self._plot_widget = pg.PlotWidget(title="Events")
        self._plot_widget.setLabel("bottom", "Time", units="s")
        self._plot_widget.hideAxis("left")
        self._plot_widget.setMouseEnabled(x=False, y=False)
        self._plot_widget.getViewBox().setMenuEnabled(False)
        self._plot_widget.viewport().installEventFilter(self)
        self._plot_widget.setYRange(0, 1, padding=0)
        layout.addWidget(self._plot_widget)

        # Add event markers
        for ev in events:
            color = EVENT_COLORS.get(ev.label, "#ffff00")
            label_text = ev.label
            if ev.data:
                label_text += f" ({ev.data})"
            line = pg.InfiniteLine(
                pos=ev.time_s,
                angle=90,
                pen=pg.mkPen(color, width=2),
                label=label_text,
                labelOpts={"position": 0.5, "color": color, "fill": "#222222"},
            )
            self._plot_widget.addItem(line)

        # Connect to controller
        controller.viewChanged.connect(self._on_view_changed)

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Type.Wheel:
            parent = self.parent()
            while parent and not isinstance(parent, QScrollArea):
                parent = parent.parent()
            if parent:
                QApplication.sendEvent(parent.verticalScrollBar(), event)
            return True
        return super().eventFilter(obj, event)

    def _on_view_changed(self, vmin: float, vmax: float):
        if self._updating:
            return
        self._updating = True
        self._plot_widget.setXRange(vmin, vmax, padding=0)
        self._updating = False
