import sys
import os
sys.path.insert(0, os.path.abspath("."))
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt, QPoint, QPointF
from PyQt6.QtGui import QWheelEvent

from ground_station.app import GroundStationApp
from ground_station.models import VehicleID

app = QApplication.instance() or QApplication(sys.argv)
win = GroundStationApp(mode_label="TEST", simulator_mode=True)
win.resize(1560, 960)
win.show()

# Switch to Rocket page
win._navigate(1)
rocket_page = win.rocket_page
scroll = rocket_page.findChild(type(rocket_page.layout().itemAt(1).widget()))

# Check graphs
graphs = list(rocket_page.graphs.values())
print("Found rocket graphs:", len(graphs))

g = graphs[0]
print("Testing wheel event over graph:", g.axis_key)

# Test sending wheel event to g.plot.viewport()
# With default TelemetryPlotWidget that ignores wheelEvent:
class TelemetryPlotWidget(pg.PlotWidget):
    def wheelEvent(self, ev):
        ev.ignore()

class TestViewBox(TelemetryViewBox):
    def wheelEvent(self, ev, axis=None):
        ev.ignore()

print("Initial scroll value:", rocket_page.layout().itemAt(1).widget().verticalScrollBar().value())
# Test delivered event

