"""Bottom navigation bar with position slider and zoom slider."""

from __future__ import annotations

import math

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QGridLayout, QLabel, QSlider, QWidget

from .time_axis_controller import TimeAxisController


class NavigationBar(QWidget):
    """Two-slider control: position (start point) + zoom (window width)."""

    SLIDER_STEPS = 2000

    def __init__(self, controller: TimeAxisController, parent=None):
        super().__init__(parent)
        self._controller = controller
        self._updating = False

        self._min_width = 100e-6  # 100 microseconds
        self._max_width = max(controller.global_width, 1e-3)

        self.setFixedHeight(70)

        layout = QGridLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setVerticalSpacing(2)

        # Row 0: Position slider
        self._pos_label = QLabel("Position:")
        layout.addWidget(self._pos_label, 0, 0)

        self._pos_slider = QSlider(Qt.Orientation.Horizontal)
        self._pos_slider.setMinimum(0)
        self._pos_slider.setMaximum(self.SLIDER_STEPS)
        self._pos_slider.setValue(0)
        self._pos_slider.valueChanged.connect(self._on_pos_changed)
        layout.addWidget(self._pos_slider, 0, 1)

        self._pos_value_label = QLabel()
        self._pos_value_label.setMinimumWidth(180)
        layout.addWidget(self._pos_value_label, 0, 2)

        # Row 1: Zoom slider
        self._zoom_label = QLabel("Zoom:")
        layout.addWidget(self._zoom_label, 1, 0)

        self._zoom_slider = QSlider(Qt.Orientation.Horizontal)
        self._zoom_slider.setMinimum(0)
        self._zoom_slider.setMaximum(self.SLIDER_STEPS)
        self._zoom_slider.setValue(self.SLIDER_STEPS)  # start fully zoomed out
        self._zoom_slider.valueChanged.connect(self._on_zoom_changed)
        layout.addWidget(self._zoom_slider, 1, 1)

        self._zoom_value_label = QLabel()
        self._zoom_value_label.setMinimumWidth(180)
        layout.addWidget(self._zoom_value_label, 1, 2)

        layout.setColumnStretch(1, 1)

        controller.viewChanged.connect(self._on_view_changed)
        self._update_labels()

    def _format_time(self, t: float) -> str:
        if t < 1.0:
            return f"{t * 1e3:.1f} ms"
        return f"{t:.3f} s"

    # ── Position slider: linear mapping ──

    def _start_from_slider(self, value: int) -> float:
        """Map slider [0, STEPS] → [global_min, global_max - view_width]."""
        c = self._controller
        max_start = c.global_max - c.view_width
        if max_start <= c.global_min:
            return c.global_min
        frac = value / self.SLIDER_STEPS
        return c.global_min + frac * (max_start - c.global_min)

    def _slider_from_start(self, start: float) -> int:
        c = self._controller
        max_start = c.global_max - c.view_width
        if max_start <= c.global_min:
            return 0
        frac = (start - c.global_min) / (max_start - c.global_min)
        return int(round(max(0, min(1, frac)) * self.SLIDER_STEPS))

    # ── Zoom slider: logarithmic mapping ──

    def _width_from_slider(self, value: int) -> float:
        if self._max_width <= self._min_width:
            return self._max_width
        frac = value / self.SLIDER_STEPS
        return self._min_width * math.exp(
            frac * math.log(self._max_width / self._min_width)
        )

    def _slider_from_width(self, width: float) -> int:
        if self._max_width <= self._min_width or width <= self._min_width:
            return 0
        frac = math.log(width / self._min_width) / math.log(
            self._max_width / self._min_width
        )
        return int(round(max(0, min(1, frac)) * self.SLIDER_STEPS))

    # ── Handlers ──

    def _on_pos_changed(self, value: int):
        if self._updating:
            return
        self._updating = True
        start = self._start_from_slider(value)
        self._controller.set_start(start)
        self._update_labels()
        self._updating = False

    def _on_zoom_changed(self, value: int):
        if self._updating:
            return
        self._updating = True
        width = self._width_from_slider(value)
        # Keep the current start, adjust the end
        start = self._controller.view_min
        self._controller.set_view(start, start + width)
        self._update_labels()
        self._updating = False

    def _on_view_changed(self, vmin: float, vmax: float):
        if self._updating:
            return
        self._updating = True
        self._pos_slider.setValue(self._slider_from_start(vmin))
        self._zoom_slider.setValue(self._slider_from_width(vmax - vmin))
        self._update_labels()
        self._updating = False

    def _update_labels(self):
        c = self._controller
        self._pos_value_label.setText(
            f"{self._format_time(c.view_min)} .. {self._format_time(c.view_max)}"
        )
        width = c.view_width
        if width < 1e-3:
            wtext = f"{width * 1e6:.0f} \u00b5s"
        elif width < 1.0:
            wtext = f"{width * 1e3:.1f} ms"
        else:
            wtext = f"{width:.3f} s"
        self._zoom_value_label.setText(f"Window: {wtext}")
