"""
scratch/capture_dashboard_logo.py
════════════════════════════════
Renders the Ground Station application offscreen and captures screenshots of:
1. Overview dashboard with header logo, sidebar logo, and background watermark
2. Rocket dashboard
3. CanSat dashboard
4. Command dashboard
"""
import os
import sys

# Ensure project root is in sys.path
sys.path.insert(0, ".")

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QApplication

from ground_station.app import GroundStationApp
from ground_station.theme import ThemeManager

def main():
    os.environ["QT_QPA_PLATFORM"] = "offscreen"
    app = QApplication.instance() or QApplication(sys.argv)
    app.setStyleSheet(ThemeManager.main_stylesheet())

    win = GroundStationApp(mode_label="SIMULATOR", simulator_mode=True)
    win.resize(1560, 960)
    win.show()

    # Process events to allow full layout and rendering
    app.processEvents()

    # Capture 1: Overview
    pix1 = win.grab()
    pix1.save("scratch/dashboard_overview_with_logo.png")
    print("Saved scratch/dashboard_overview_with_logo.png")

    # Capture 2: Header and Sidebar zoom
    crop_header = pix1.copy(0, 0, 850, 180)
    crop_header.save("scratch/crop_header_logo.png")
    print("Saved scratch/crop_header_logo.png")

    # Capture 3: Rocket page
    win._navigate(1)
    app.processEvents()
    pix2 = win.grab()
    pix2.save("scratch/dashboard_rocket_with_logo.png")
    print("Saved scratch/dashboard_rocket_with_logo.png")

    # Capture 4: Command page
    win._navigate(3)
    app.processEvents()
    pix3 = win.grab()
    pix3.save("scratch/dashboard_command_with_logo.png")
    print("Saved scratch/dashboard_command_with_logo.png")

if __name__ == "__main__":
    main()
