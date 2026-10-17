import sys
import os
sys.path.insert(0, os.path.abspath("."))
from PyQt6.QtWidgets import QApplication

from ground_station.app import GroundStationApp
from ground_station.theme import ThemeManager

app = QApplication.instance() or QApplication(sys.argv)
app.setStyleSheet(ThemeManager.main_stylesheet())

win = GroundStationApp(mode_label="SIMULATOR", simulator_mode=True)
win.resize(1560, 960)
win.show()

os.makedirs("scratch/renders", exist_ok=True)

pages = [
    ("overview", win.overview_page, 0),
    ("rocket", win.rocket_page, 1),
    ("cansat", win.cansat_page, 2),
    ("command", win.command_page, 3),
    ("experiment", win.experiment_page, 4),
    ("com", win.com_page, 5),
]

for name, page, idx in pages:
    win._navigate(idx)
    app.processEvents()
    img_path = f"scratch/renders/{name}.png"
    win.grab().save(img_path)
    print(f"Rendered {name} -> {img_path}")

print("All page renders captured successfully.")
