"""Main application window assembling all viewer components."""

from __future__ import annotations

from PyQt6.QtWidgets import (
    QFileDialog,
    QMainWindow,
    QVBoxLayout,
    QWidget,
)

from .log_loader import FlightData, load_jsonl
from .navigation_bar import NavigationBar
from .plot_stack import PlotStack
from .time_axis_controller import TimeAxisController


class MainWindow(QMainWindow):
    def __init__(self, flight_data: FlightData, filename: str = "", parent=None):
        super().__init__(parent)
        self._flight_data = flight_data

        title = "TVR Flight Log Viewer"
        if filename:
            title += f" - {filename}"
        self.setWindowTitle(title)
        self.resize(1400, 900)

        # Central time controller
        self._controller = TimeAxisController(
            flight_data.t_min, flight_data.t_max
        )

        # Layout
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Plot stack (takes all stretch)
        self._plot_stack = PlotStack(flight_data, self._controller)
        layout.addWidget(self._plot_stack, stretch=1)

        # Bottom navigation bar
        self._nav_bar = NavigationBar(self._controller)
        layout.addWidget(self._nav_bar)

        # Menu bar
        menu = self.menuBar()
        file_menu = menu.addMenu("File")
        open_action = file_menu.addAction("Open...")
        open_action.triggered.connect(self._open_file)

    def _open_file(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Open Flight Log", "", "JSONL Files (*.jsonl);;All Files (*)"
        )
        if path:
            from pathlib import Path

            flight_data = load_jsonl(Path(path))
            new_window = MainWindow(flight_data, Path(path).name)
            new_window.show()
            # Keep reference so it's not garbage collected
            if not hasattr(self, "_child_windows"):
                self._child_windows = []
            self._child_windows.append(new_window)
