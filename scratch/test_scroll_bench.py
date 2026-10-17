import sys
import os
sys.path.insert(0, os.path.abspath("."))
from PyQt6.QtWidgets import QApplication, QScrollArea, QWidget, QVBoxLayout, QLabel
from PyQt6.QtCore import Qt, QPoint, QPointF
from PyQt6.QtGui import QWheelEvent
import pyqtgraph as pg

from ground_station.models import VehicleID

class GroundStationScrollArea(QScrollArea):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWidgetResizable(True)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.setFrameShape(QScrollArea.Shape.NoFrame)
        vbar = self.verticalScrollBar()
        vbar.setSingleStep(48)

    def wheelEvent(self, ev: QWheelEvent) -> None:
        p_delta = ev.pixelDelta()
        if not p_delta.isNull() and p_delta.y() != 0:
            vbar = self.verticalScrollBar()
            vbar.setValue(vbar.value() - p_delta.y())
            ev.accept()
            return

        a_delta = ev.angleDelta()
        if not a_delta.isNull() and a_delta.y() != 0:
            vbar = self.verticalScrollBar()
            steps = a_delta.y() / 120.0
            delta_px = int(steps * 48)
            vbar.setValue(vbar.value() - delta_px)
            ev.accept()
            return

        super().wheelEvent(ev)

app = QApplication.instance() or QApplication(sys.argv)

scroll = GroundStationScrollArea()
body = QWidget()
lay = QVBoxLayout(body)
for i in range(50):
    lay.addWidget(QLabel(f"Item {i} " + "x" * 50))
scroll.setWidget(body)
scroll.resize(800, 400)
scroll.show()

vbar = scroll.verticalScrollBar()
print("Initial vbar value:", vbar.value(), "max:", vbar.maximum())

# Test mouse wheel down
ev_down = QWheelEvent(
    QPointF(100.0, 100.0), QPointF(100.0, 100.0),
    QPoint(0, 0), QPoint(0, -120),
    Qt.MouseButton.NoButton,
    Qt.KeyboardModifier.NoModifier,
    Qt.ScrollPhase.NoScrollPhase,
    False
)
scroll.wheelEvent(ev_down)
print("After wheel down:", vbar.value())
assert vbar.value() == 48, f"Expected 48, got {vbar.value()}"

# Test mouse wheel up
ev_up = QWheelEvent(
    QPointF(100.0, 100.0), QPointF(100.0, 100.0),
    QPoint(0, 0), QPoint(0, 120),
    Qt.MouseButton.NoButton,
    Qt.KeyboardModifier.NoModifier,
    Qt.ScrollPhase.NoScrollPhase,
    False
)
scroll.wheelEvent(ev_up)
print("After wheel up:", vbar.value())
assert vbar.value() == 0, f"Expected 0, got {vbar.value()}"

# Test trackpad pixelDelta
ev_trackpad = QWheelEvent(
    QPointF(100.0, 100.0), QPointF(100.0, 100.0),
    QPoint(0, -35), QPoint(0, 0),
    Qt.MouseButton.NoButton,
    Qt.KeyboardModifier.NoModifier,
    Qt.ScrollPhase.NoScrollPhase,
    False
)
scroll.wheelEvent(ev_trackpad)
print("After trackpad scroll:", vbar.value())
assert vbar.value() == 35, f"Expected 35, got {vbar.value()}"

print("All GroundStationScrollArea tests passed!")
