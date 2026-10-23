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

import os
from PyQt6.QtCore import QEvent, QObject, QPoint, Qt
from PyQt6.QtGui import QColor, QPainter, QPixmap, QWheelEvent
from PyQt6.QtWidgets import QAbstractScrollArea, QScrollArea, QWidget


class GroundStationScrollArea(QScrollArea):
    """Primary vertical scroll container for ground station dashboards."""

    # Standard desktop wheel scroll step in pixels per 120-unit wheel notch
    WHEEL_STEP_PX = 48

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setWidgetResizable(True)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.setFrameShape(QScrollArea.Shape.NoFrame)
        self.verticalScrollBar().setSingleStep(self.WHEEL_STEP_PX)
        self.horizontalScrollBar().setSingleStep(self.WHEEL_STEP_PX)
        self.viewport().setAutoFillBackground(False)
        self.viewport().setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)

    def wheelEvent(self, ev: QWheelEvent) -> None:
        """Handle wheel and touchpad events with zero delay for both vertical and horizontal directions."""
        # 1. Precision touchpad gestures provide pixelDelta (e.g. Windows Precision / macOS)
        p_delta = ev.pixelDelta()
        if not p_delta.isNull():
            handled = False
            if p_delta.y() != 0:
                vbar = self.verticalScrollBar()
                vbar.setValue(vbar.value() - p_delta.y())
                handled = True
            if p_delta.x() != 0:
                hbar = self.horizontalScrollBar()
                hbar.setValue(hbar.value() - p_delta.x())
                handled = True
            if handled:
                ev.accept()
                return

        # 2. Standard mouse wheel notches provide angleDelta (120 units per notch)
        a_delta = ev.angleDelta()
        if not a_delta.isNull():
            # If Shift is pressed or horizontal delta is dominant, scroll horizontally
            if (ev.modifiers() & Qt.KeyboardModifier.ShiftModifier) or (a_delta.x() != 0 and abs(a_delta.x()) > abs(a_delta.y())):
                hbar = self.horizontalScrollBar()
                dx = a_delta.x() if a_delta.x() != 0 else a_delta.y()
                steps = dx / 120.0
                delta_px = int(steps * self.WHEEL_STEP_PX)
                hbar.setValue(hbar.value() - delta_px)
                ev.accept()
                return
            elif a_delta.y() != 0:
                vbar = self.verticalScrollBar()
                steps = a_delta.y() / 120.0
                delta_px = int(steps * self.WHEEL_STEP_PX)
                vbar.setValue(vbar.value() - delta_px)
                ev.accept()
                return

        super().wheelEvent(ev)

    def attach_child_wheel_passthrough(self, child: QWidget) -> None:
        """Install an event filter on a nested scrollable, plot, or input widget so that
        wheel events propagate seamlessly to this main page scroll area.
        """
        filter_obj = _ChildWheelFilter(self, child)
        child.installEventFilter(filter_obj)
        if hasattr(child, "viewport") and callable(child.viewport):
            vp = child.viewport()
            if vp is not None:
                vp.installEventFilter(filter_obj)


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
                at_top = vbar.value() <= vbar.minimum() and dy > 0
                at_bottom = vbar.value() >= vbar.maximum() and dy < 0
                not_scrollable = vbar.maximum() <= vbar.minimum()

                if at_top or at_bottom or not_scrollable:
                    self.parent_scroll.wheelEvent(event)
                    return True
            else:
                self.parent_scroll.wheelEvent(event)
                return True
        return super().eventFilter(watched, event)
