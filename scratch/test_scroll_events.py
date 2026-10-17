import sys
import os
sys.path.insert(0, os.path.abspath("."))
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QScrollArea, QLabel
from PyQt6.QtCore import Qt, QPoint
from PyQt6.QtGui import QWheelEvent
import pyqtgraph as pg

from ground_station.graphs.plot import TelemetryGraph, TelemetryViewBox
from ground_station.pages.vehicle import VehiclePage
from ground_station.models import VehicleID

app = QApplication.instance() or QApplication(sys.argv)

page = VehiclePage(VehicleID.ROCKET)
page.resize(1200, 800)
page.show()

# Find the scroll area in page
scroll_area = page.findChild(QScrollArea)
assert scroll_area is not None
v_bar = scroll_area.verticalScrollBar()
initial_val = v_bar.value()

# Find a TelemetryGraph
g = list(page.graphs.values())[0]
print("Graph found:", g.axis_key)

# Simulate a wheel event on the graph
from PyQt6.QtCore import Qt, QPoint, QPointF

pos = QPointF(50.0, 50.0)
global_pos = QPointF(g.mapToGlobal(QPoint(50, 50)))
wheel_event = QWheelEvent(
    pos, global_pos,
    QPoint(0, 0), QPoint(0, -120),
    Qt.MouseButton.NoButton,
    Qt.KeyboardModifier.NoModifier,
    Qt.ScrollPhase.NoScrollPhase,
    False
)

# Let's test Qt event propagation up parent chain when child ignores
def deliver_wheel_event(target, event):
    curr = target
    while curr is not None:
        if hasattr(curr, "wheelEvent"):
            curr.wheelEvent(event)
            if event.isAccepted():
                break
        curr = curr.parent()

g.plot.wheelEvent = lambda ev: ev.ignore()
g.vb.wheelEvent = lambda ev, axis=None: ev.ignore()

# Reset event
wheel_event = QWheelEvent(
    pos, global_pos,
    QPoint(0, 0), QPoint(0, -120),
    Qt.MouseButton.NoButton,
    Qt.KeyboardModifier.NoModifier,
    Qt.ScrollPhase.NoScrollPhase,
    False
)
deliver_wheel_event(g.plot, wheel_event)
print("VBar value after delivering with ignore():", v_bar.value())
print("VBar max:", v_bar.maximum())



