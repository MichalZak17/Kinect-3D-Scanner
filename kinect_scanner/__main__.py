"""Entry point: python -m kinect_scanner"""

import sys

from PyQt6.QtWidgets import QApplication

from .gui.main_window import MainWindow


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("Kinect 3D Scanner")
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
