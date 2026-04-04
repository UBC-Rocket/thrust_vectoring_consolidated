"""Entry point for the flight log viewer."""

import sys
from pathlib import Path

from PyQt6.QtWidgets import QApplication, QFileDialog

from .log_loader import load_jsonl
from .main_window import MainWindow


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("TVR Flight Log Viewer")

    # Get file path from CLI arg or file dialog
    if len(sys.argv) > 1:
        path = Path(sys.argv[1])
    else:
        path_str, _ = QFileDialog.getOpenFileName(
            None, "Open Flight Log", "", "JSONL Files (*.jsonl);;All Files (*)"
        )
        if not path_str:
            sys.exit(0)
        path = Path(path_str)

    if not path.exists():
        print(f"Error: {path} does not exist", file=sys.stderr)
        sys.exit(1)

    flight_data = load_jsonl(path)
    window = MainWindow(flight_data, path.name)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
