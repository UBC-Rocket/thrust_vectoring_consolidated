"""Central controller for the shared time axis across all plots."""

from PyQt6.QtCore import QObject, pyqtSignal


class TimeAxisController(QObject):
    """Single source of truth for the visible time window.

    All plots, the scrubber, and the zoom slider connect to viewChanged
    and call methods here to update the view. This prevents circular
    update cascades.
    """

    viewChanged = pyqtSignal(float, float)  # view_min, view_max

    def __init__(self, global_min: float, global_max: float, parent=None):
        super().__init__(parent)
        self.global_min = global_min
        self.global_max = global_max
        self._view_min = global_min
        self._view_max = global_max

    @property
    def view_min(self) -> float:
        return self._view_min

    @property
    def view_max(self) -> float:
        return self._view_max

    @property
    def view_width(self) -> float:
        return self._view_max - self._view_min

    @property
    def global_width(self) -> float:
        return self.global_max - self.global_min

    def set_view(self, vmin: float, vmax: float):
        """Set the visible time window, clamping to global bounds."""
        width = vmax - vmin
        if width < 1e-6:
            width = 1e-6

        # Clamp to global range
        if vmin < self.global_min:
            vmin = self.global_min
            vmax = vmin + width
        if vmax > self.global_max:
            vmax = self.global_max
            vmin = vmax - width
        if vmin < self.global_min:
            vmin = self.global_min

        if vmin != self._view_min or vmax != self._view_max:
            self._view_min = vmin
            self._view_max = vmax
            self.viewChanged.emit(vmin, vmax)

    def set_zoom_width(self, width: float):
        """Change the view width, keeping the center fixed."""
        center = (self._view_min + self._view_max) / 2
        self.set_view(center - width / 2, center + width / 2)

    def pan_to(self, center: float):
        """Pan the view to a new center, keeping width constant."""
        half = self.view_width / 2
        self.set_view(center - half, center + half)

    def set_start(self, start: float):
        """Set the view start point, keeping the current width."""
        self.set_view(start, start + self.view_width)
