"""
ground_station/widgets/scroll_area.py
═════════════════════════════════════
High-performance dashboard scroll container.

Provides immediate, fluid, native desktop scrolling across all ground
station pages (Overview, Rocket, CanSat, Command, CanSat Experiment, COM).

Features:
  - 1:1 hardware responsiveness for precision trackpads & laptop touchpads.
  - Smooth, consistent step scaling for mouse wheel inputs.
  - Clamped vertical range with zero overshoot/jumping.
  - Strict horizontal scroll prevention (ScrollBarAlwaysOff).
  - Unbroken propagation from inner cards, plots, and tables.
"""
from __future__ import annotations

from typing import Optional

from PyQt6.QtCore import QEvent, QObject, QPoint, Qt
from PyQt6.QtGui import QWheelEvent
from PyQt6.QtWidgets import QAbstractScrollArea, QScrollArea, QWidget


class GroundStationScrollArea(QScrollArea):
    """Primary vertical scroll container for ground station dashboards."""

    # Standard desktop wheel scroll step in pixels per 120-unit wheel notch
    WHEEL_STEP_PX = 48

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setWidgetResizable(True)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.setFrameShape(QScrollArea.Shape.NoFrame)
        self.verticalScrollBar().setSingleStep(self.WHEEL_STEP_PX)

    def wheelEvent(self, ev: QWheelEvent) -> None:
        """Handle wheel and touchpad events with zero delay."""
        # 1. Precision touchpad gestures provide pixelDelta (e.g. Windows Precision / macOS)
        p_delta = ev.pixelDelta()
        if not p_delta.isNull() and p_delta.y() != 0:
            vbar = self.verticalScrollBar()
            vbar.setValue(vbar.value() - p_delta.y())
            ev.accept()
            return

        # 2. Standard mouse wheel notches provide angleDelta (120 units per notch)
        a_delta = ev.angleDelta()
        if not a_delta.isNull() and a_delta.y() != 0:
            vbar = self.verticalScrollBar()
            steps = a_delta.y() / 120.0
            delta_px = int(steps * self.WHEEL_STEP_PX)
            vbar.setValue(vbar.value() - delta_px)
            ev.accept()
            return

        super().wheelEvent(ev)

    def attach_child_wheel_passthrough(self, child: QWidget) -> None:
        """Install an event filter on a nested scrollable or input widget so that
        wheel events that reach boundary conditions propagate to this main page scroll area.
        """
        child.installEventFilter(_ChildWheelFilter(self, child))


class _ChildWheelFilter(QObject):
    """Event filter that passes unhandled or boundary wheel events to the parent scroll area."""

    def __init__(self, parent_scroll: GroundStationScrollArea, target: QWidget):
        super().__init__(target)
        self.parent_scroll = parent_scroll
        self.target = target

    def eventFilter(self, watched: QObject, event: QEvent) -> bool:
        if event.type() == QEvent.Type.Wheel and isinstance(event, QWheelEvent):
            # If target is an abstract scroll area (e.g. QTableWidget, nested QScrollArea)
            if isinstance(self.target, QAbstractScrollArea):
                vbar = self.target.verticalScrollBar()
                dy = event.angleDelta().y() or event.pixelDelta().y()
                # If cannot scroll further up or down, pass event to parent
                at_top = vbar.value() <= vbar.minimum() and dy > 0
                at_bottom = vbar.value() >= vbar.maximum() and dy < 0
                not_scrollable = vbar.maximum() <= vbar.minimum()

                if at_top or at_bottom or not_scrollable:
                    self.parent_scroll.wheelEvent(event)
                    return True
        return super().eventFilter(watched, event)
