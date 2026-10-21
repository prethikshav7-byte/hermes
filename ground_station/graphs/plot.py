"""
ground_station/graphs/plot.py
═════════════════════════════
TelemetryGraph — replaces the old telemetry_plot.py entirely.

The old widget's model was wall-clock watchdog driven: a STALE state
after N seconds of silence, a "held" dashed segment, a dotted reconnect
bridge on resume. §7.5 replaces that with a packet-count-driven three-tier
model, so the two could not be reconciled and this is a rewrite rather
than a patch.

The other structural change: the old widget *derived* its own stale state
from a QTimer. This one does not derive anything. Tiering happens in
parser.py (one computation, §7.5.2), so the health strip and the graph
can never disagree about whether an outage occurred.

WHAT §7 REQUIRES, AND WHERE IT IS
─────────────────────────────────
  §7.1  selection      _set_selected, outline drawn outside the plot area
  §7.2  zoom           trackpad pinch only; wheel explicitly swallowed
  §7.3  x-axis growth  follow mode, broken by manual interaction
  §7.4  axis titles    no legends anywhere; inline end-of-line trace labels
  §7.5  loss           add_point(tier=...) drives the OutageTracker
  §8    y clamps       axes.clamp_range on every autoscale
  §13.2 decimation     min/max per pixel column at wide zoom
  §13.3 render rate    repaint on a timer, never per packet
"""
from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QColor, QFont, QPainter, QPen
from PyQt6.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QVBoxLayout, QWidget,
)

from .. import axes as axspec
from ..models import FlightState
from ..theme import ThemeManager
from .decimate import RingBuffer, minmax_decimate
from .reconcile import OutageTracker
from ..parser import LossTier


# ─────────────────────────────────────────────────────────────────────────────
#  ViewBox — §7.2 input rules
# ─────────────────────────────────────────────────────────────────────────────

class TelemetryViewBox(pg.ViewBox):
    """A ViewBox that handles direct click-and-drag panning and pinch gestures.

    Wheel events are passed through so vertical dashboard scrolling stays smooth.
    """

    pinched = pyqtSignal(float, float)      # scale factor, centre x
    sig_panned = pyqtSignal(float, float)   # delta_x, delta_y in data coordinates
    sig_clicked = pyqtSignal()

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.selected = False
        self.setMouseEnabled(x=False, y=False)
        self.setMenuEnabled(False)
        self.setCursor(Qt.CursorShape.OpenHandCursor)
        self._dragging = False

    def wheelEvent(self, ev, axis=None):
        # §7.2: Wheel never zooms/pans graph. Ignore so it bubbles to parent QScrollArea.
        ev.ignore()

    def mouseDragEvent(self, ev, axis=None):
        ev.accept()
        if ev.button() == Qt.MouseButton.LeftButton:
            if ev.isStart():
                self._dragging = True
                self.setCursor(Qt.CursorShape.ClosedHandCursor)

            p1 = self.mapToView(ev.lastPos())
            p2 = self.mapToView(ev.pos())
            delta_x = -(p2.x() - p1.x())
            delta_y = -(p2.y() - p1.y())
            self.sig_panned.emit(delta_x, delta_y)

            if ev.isFinish():
                self._dragging = False
                self.setCursor(Qt.CursorShape.OpenHandCursor)

    def mouseClickEvent(self, ev):
        if ev.button() == Qt.MouseButton.LeftButton:
            ev.accept()
            self.sig_clicked.emit()


class TelemetryPlotWidget(pg.PlotWidget):
    """PlotWidget that allows wheel events to bubble to parent scroll container."""

    def wheelEvent(self, ev):
        ev.ignore()




# ─────────────────────────────────────────────────────────────────────────────
#  NaN-aware decimation helper
# ─────────────────────────────────────────────────────────────────────────────

def _decimate_with_nans(xs: List[float], ys: List[float],
                        columns: int) -> Tuple[List[float], List[float]]:
    """Decimate a NaN-gapped series while preserving gap boundaries.

    minmax_decimate assumes a contiguous numeric array. Feeding it an array
    that already contains NaN sentinels (inserted by _split_on_breaks) causes
    the decimator to misplace min/max representatives and corrupt the
    connect="finite" breaks. This wrapper splits on the sentinels first,
    decimates each clean segment independently, then rejoins them with NaN
    separators so pyqtgraph's connect="finite" still sees the right positions.
    """
    if not xs:
        return [], []

    out_x: List[float] = []
    out_y: List[float] = []
    seg_x: List[float] = []
    seg_y: List[float] = []

    for x, y in zip(xs, ys):
        if y != y:          # NaN check (NaN != NaN is always True)
            if seg_x:
                dx, dy = minmax_decimate(seg_x, seg_y, columns)
                if out_x:   # insert separator between segments
                    out_x.append(float("nan"))
                    out_y.append(float("nan"))
                out_x.extend(dx)
                out_y.extend(dy)
                seg_x, seg_y = [], []
        else:
            seg_x.append(x)
            seg_y.append(y)

    if seg_x:
        dx, dy = minmax_decimate(seg_x, seg_y, columns)
        if out_x:
            out_x.append(float("nan"))
            out_y.append(float("nan"))
        out_x.extend(dx)
        out_y.extend(dy)

    return out_x, out_y


# ─────────────────────────────────────────────────────────────────────────────
#  The graph
# ─────────────────────────────────────────────────────────────────────────────
#  Telemetry Channel Colors and Icons
# ─────────────────────────────────────────────────────────────────────────────

CHANNEL_COLORS: Dict[str, str] = {
    "altitude": "#38bdf8",
    "velocity": "#22c55e",
    "descent_rate": "#22c55e",
    "temperature": "#f97316",
    "acceleration": "#a855f7",
    "pressure": "#38bdf8",
    "orientation": "#f43f5e",
}

CHANNEL_ICONS: Dict[str, str] = {
    "altitude": "▲",
    "velocity": "►",
    "descent_rate": "↓",
    "temperature": "◇",
    "acceleration": "∿",
    "pressure": "⊕",
    "orientation": "↻",
}


# ─────────────────────────────────────────────────────────────────────────────
#  The graph
# ─────────────────────────────────────────────────────────────────────────────

class TelemetryGraph(QWidget):
    """One live time-series channel."""

    selection_changed = pyqtSignal(object, bool)     # self, selected

    #: §13.3 — repaint at a fixed rate rather than once per packet.
    #: Telemetry arrives at up to 20 lines/s across both vehicles; this
    #: batches whatever arrived between frames.
    REPAINT_MS = 40                                   # 25 fps

    #: §7.1 — outline thickness, drawn outside the plot area and inside
    #: the widget padding so it never overlaps axis labels or the trace.
    SELECT_BORDER_PX = 2
    SELECT_PADDING_PX = 2

    def __init__(self, axis_key: str, spec: Optional[axspec.AxisSpec] = None,
                 traces: Optional[Sequence[Tuple[str, str]]] = None,
                 parent=None):
        """
        axis_key  key into axes.BY_KEY / axes.ZOOM
        spec      override, e.g. DESCENT_RATE instead of VELOCITY (§6.3)
        traces    [(name, colour)] for multi-trace channels (§7.4).
                  Single-trace graphs pass None.
        """
        super().__init__(parent)
        c = ThemeManager.C()

        self.axis_key = axis_key
        self.spec = spec or axspec.BY_KEY[axis_key]
        self.zoom_spec = axspec.zoom_for(axis_key)

        self._channel_color = CHANNEL_COLORS.get(axis_key, c.CYAN)
        self._traces = list(traces) if traces else [("", self._channel_color)]
        self._multi = bool(traces) and len(self._traces) > 1

        self._buffers: Dict[str, RingBuffer] = {
            name: RingBuffer() for name, _ in self._traces
        }
        self._curves: Dict[str, pg.PlotDataItem] = {}
        self._labels: Dict[str, pg.TextItem] = {}

        # §7.5.9 — only the altitude family reconciles.
        self._outage = OutageTracker(
            reconcilable=axis_key in axspec.RECONCILABLE_KEYS
        )
        self._placeholder_item: Optional[pg.PlotDataItem] = None
        self._resolved_items: List[pg.PlotDataItem] = []

        self._selected = False
        self._follow = True                  # §7.3
        self._manual_y = False
        self._view_x_min = 0.0
        self._view_x_max = self.zoom_spec.max_zoom_span_s
        self._x_span = self._view_x_max - self._view_x_min
        default_y_lo, default_y_hi = axspec.clamp_range(self.spec, None, None)
        self._view_y_min = default_y_lo
        self._view_y_max = default_y_hi
        self._latest_x = 0.0
        self._dirty = False
        self._state_now: Optional[FlightState] = None
        self._v_entry: Optional[float] = None

        self._build_ui()

        self._repaint = QTimer(self)
        self._repaint.setInterval(self.REPAINT_MS)
        self._repaint.timeout.connect(self._maybe_redraw)
        self._repaint.start()

    # ── construction ─────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        c = ThemeManager.C()
        layout = QVBoxLayout(self)
        pad = self.SELECT_BORDER_PX + self.SELECT_PADDING_PX
        layout.setContentsMargins(pad, pad, pad, pad)
        layout.setSpacing(0)

        # Telemetry Card Frame
        self.card = QFrame(self)
        self.card.setObjectName("TelemetryCard")
        card_layout = QVBoxLayout(self.card)
        card_layout.setContentsMargins(8, 6, 8, 6)
        card_layout.setSpacing(4)

        # Header Row: Icon, Title, Numerical Value, and Controls
        hdr = QHBoxLayout()
        hdr.setContentsMargins(4, 2, 4, 2)
        hdr.setSpacing(6)

        is_descent = (self.axis_key == "descent_rate" or
                      self.spec is axspec.DESCENT_RATE or
                      getattr(self.spec, "name", "") == "Descent Rate")

        display_title = self.spec.axis_title()
        if is_descent:
            display_title = "Descent Rate (m/s)"
        elif self.axis_key == "altitude":
            display_title = "Altitude (m/AGL)"
        elif self.axis_key == "orientation":
            display_title = "Gyro Spin Rate (deg/s)"
        elif self.axis_key == "velocity":
            display_title = "Velocity (m/s)"
        elif self.axis_key == "pressure":
            display_title = "Pressure (Pa)"
        elif self.axis_key == "temperature":
            display_title = "Temperature (°C)"
        elif self.axis_key == "acceleration":
            display_title = "Acceleration (m/s²)"

        self.lbl_title = QLabel(display_title)
        self.lbl_title.setFont(QFont("Segoe UI", 9, QFont.Weight.DemiBold))
        self.lbl_title.setStyleSheet(f"color: {c.TEXT_DIM};")
        hdr.addWidget(self.lbl_title)

        hdr.addStretch(1)

        self.lbl_value = QLabel("—")
        self.lbl_value.setFont(QFont("Segoe UI", 10, QFont.Weight.Bold))
        self.lbl_value.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.lbl_value.setMinimumWidth(120)
        self.lbl_value.setStyleSheet(f"color: {self._channel_color}; margin-right: 8px;")
        hdr.addWidget(self.lbl_value)

        btn_style = ThemeManager.graph_button_stylesheet()

        self.btn_reset = QPushButton("⟲")
        self.btn_reset.setToolTip("Reset view (auto-follow)")
        self.btn_reset.setStyleSheet(btn_style)
        self.btn_reset.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_reset.clicked.connect(lambda: self.reset_view())

        self.btn_zoom_out = QPushButton("−")
        self.btn_zoom_out.setToolTip("Zoom out (-)")
        self.btn_zoom_out.setStyleSheet(btn_style)
        self.btn_zoom_out.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_zoom_out.clicked.connect(lambda: self.zoom_out())

        self.btn_zoom_in = QPushButton("+")
        self.btn_zoom_in.setToolTip("Zoom in (+)")
        self.btn_zoom_in.setStyleSheet(btn_style)
        self.btn_zoom_in.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_zoom_in.clicked.connect(lambda: self.zoom_in())

        hdr.addWidget(self.btn_reset)
        hdr.addWidget(self.btn_zoom_out)
        hdr.addWidget(self.btn_zoom_in)
        card_layout.addLayout(hdr)

        self.vb = TelemetryViewBox()
        self.vb.sig_panned.connect(self.pan_by)
        self.vb.sig_clicked.connect(self._on_vb_clicked)
        self.plot = TelemetryPlotWidget(viewBox=self.vb, background=c.BG_CARD)
        self.plot.setAntialiasing(False)

        # §7.4 — axis titles outside the plot area, no legend anywhere.
        label_style = {"color": c.TEXT_DIM, "font-size": "8pt"}
        self.plot.setLabel("bottom", "Time (s)", **label_style)
        self.plot.setLabel("left", self.spec.axis_title(), **label_style)
        self.plot.getAxis("left").setWidth(64)
        self.plot.getAxis("bottom").setHeight(36)
        for side in ("left", "bottom"):
            ax = self.plot.getAxis(side)
            ax.enableAutoSIPrefix(False)
            ax.setPen(pg.mkPen(c.BORDER))
            ax.setTextPen(pg.mkPen(c.TEXT_DIM))
            ax.setStyle(tickFont=QFont("monospace", 8))

        self.plot.showGrid(x=True, y=True, alpha=0.10)

        for name, colour in self._traces:
            qc = QColor(colour)
            pen = pg.mkPen(qc, width=2)
            self._curves[name] = self.plot.plot([], [], pen=pen)

            if self._multi:
                lbl = pg.TextItem(name, color=colour, anchor=(0, 0.5))
                lbl.setFont(QFont("monospace", 8))
                self.plot.addItem(lbl)
                self._labels[name] = lbl

        card_layout.addWidget(self.plot)
        layout.addWidget(self.card)

        self.setSizePolicy(QSizePolicy.Policy.Expanding,
                           QSizePolicy.Policy.Expanding)
        self.setMinimumHeight(230)
        self.grabGesture(Qt.GestureType.PinchGesture)

    # ── §7.1 selection ───────────────────────────────────────────────────

    @property
    def selected(self) -> bool:
        return self._selected

    def set_selected(self, on: bool) -> None:
        if on == self._selected:
            return
        self._selected = on
        self.vb.selected = on
        self.update()
        self.selection_changed.emit(self, on)

    def mousePressEvent(self, ev) -> None:
        # §7.1 — tap toggles. One selected graph per dashboard is enforced
        # by the page, which listens to selection_changed.
        self.set_selected(not self._selected)
        ev.accept()

    def paintEvent(self, ev) -> None:
        super().paintEvent(ev)
        if not self._selected:
            return
        c = ThemeManager.C()
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setPen(QPen(QColor(c.CYAN), self.SELECT_BORDER_PX))
        p.setBrush(Qt.BrushStyle.NoBrush)
        inset = self.SELECT_BORDER_PX // 2 + 1
        p.drawRoundedRect(self.rect().adjusted(inset, inset, -inset - 1, -inset - 1), 8, 8)
        p.end()

    # ── §7.2 zoom ────────────────────────────────────────────────────────

    def event(self, ev):
        # Trackpad pinch arrives as a native gesture on the platforms in
        # scope (desktop, trackpad). Handled here rather than in
        # wheelEvent precisely because §7.2 requires the two to behave
        # differently — pinch zooms, scroll must not.
        if ev.type() == ev.Type.NativeGesture:
            if ev.gestureType() == Qt.NativeGestureType.ZoomNativeGesture:
                if self._selected:
                    val = ev.value()
                    factor = 1.0 - val
                    if factor > 0:
                        self._apply_zoom(factor)
                        self._redraw()
                ev.accept()
                return True
        return super().event(ev)

    def _apply_zoom(self, factor: float) -> None:
        """Apply a multiplicative zoom factor around the center of the visible viewport.

        factor < 1.0 zooms in (reduces span).
        factor > 1.0 zooms out (expands span).
        Clamps to [0, max(latest_x, max_zoom_span_s)] with a safe min-span floor of 0.05s.
        """
        data_min = 0.0
        data_max = max(self._latest_x, self.zoom_spec.max_zoom_span_s)
        max_possible_span = max(0.05, data_max - data_min)

        if self._follow:
            cur_min = data_min
            cur_max = data_max
        else:
            cur_min = self._view_x_min
            cur_max = self._view_x_max
            if cur_max <= cur_min or not (cur_min == cur_min and cur_max == cur_max):
                cur_min, cur_max = data_min, data_max

        cur_span = max(0.001, cur_max - cur_min)
        center = 0.5 * (cur_min + cur_max)

        MIN_SPAN_FLOOR = 0.05  # 50ms floor prevents numerical collapse
        safe_factor = max(0.01, min(100.0, factor))
        new_span = cur_span * safe_factor
        new_span = max(MIN_SPAN_FLOOR, min(max_possible_span, new_span))

        if new_span >= max_possible_span:
            new_min = data_min
            new_max = data_max
        else:
            new_min = center - 0.5 * new_span
            new_max = center + 0.5 * new_span
            if new_min < data_min:
                new_min = data_min
                new_max = min(data_max, data_min + new_span)
            elif new_max > data_max:
                new_max = data_max
                new_min = max(data_min, data_max - new_span)

        if new_max - new_min < MIN_SPAN_FLOOR and max_possible_span >= MIN_SPAN_FLOOR:
            if new_min <= data_min:
                new_min = data_min
                new_max = data_min + MIN_SPAN_FLOOR
            elif new_max >= data_max:
                new_max = data_max
                new_min = data_max - MIN_SPAN_FLOOR

        self._view_x_min = new_min
        self._view_x_max = new_max
        self._x_span = max(MIN_SPAN_FLOOR if max_possible_span >= MIN_SPAN_FLOOR else 0.001, new_max - new_min)
        self._follow = False
        self._dirty = True

    def _on_vb_clicked(self) -> None:
        self.set_selected(not self._selected)

    def pan_by(self, delta_x: float, delta_y: float) -> None:
        """Pan the viewport by delta_x and delta_y in data units with boundary clamping."""
        data_min = 0.0
        data_max = max(self._latest_x, self.zoom_spec.max_zoom_span_s)

        # ── X-axis Pan ───────────────────────────────────────────────────
        x_span = max(0.05, self._view_x_max - self._view_x_min)
        max_possible_x_span = max(0.05, data_max - data_min)

        if abs(delta_x) > 1e-9:
            new_x_min = self._view_x_min + delta_x
            new_x_max = new_x_min + x_span
            if new_x_min < data_min:
                new_x_min = data_min
                new_x_max = min(data_max, data_min + x_span)
            elif new_x_max > data_max:
                new_x_max = data_max
                new_x_min = max(data_min, data_max - x_span)

            self._view_x_min = new_x_min
            self._view_x_max = new_x_max
            self._x_span = new_x_max - new_x_min
            self._follow = False

        # ── Y-axis Pan ───────────────────────────────────────────────────
        if abs(delta_y) > 1e-9:
            y_span = max(0.001, self._view_y_max - self._view_y_min)
            extent_lo = self.spec.extent_lo if axspec._finite(self.spec.extent_lo) else -1e6
            extent_hi = self.spec.extent_hi if axspec._finite(self.spec.extent_hi) else 1e6
            if self.spec.hard_floor:
                extent_lo = max(0.0, extent_lo)

            new_y_min = self._view_y_min + delta_y
            new_y_max = new_y_min + y_span

            # Clamp to extent boundaries if defined
            if new_y_min < extent_lo:
                new_y_min = extent_lo
                new_y_max = min(extent_hi, extent_lo + y_span)
            elif new_y_max > extent_hi:
                new_y_max = extent_hi
                new_y_min = max(extent_lo, extent_hi - y_span)

            self._view_y_min = new_y_min
            self._view_y_max = new_y_max
            self._manual_y = True

        self._dirty = True
        self._redraw()

    def zoom_in(self, factor: Optional[float] = None) -> None:
        """Zoom in on the visible time viewport."""
        f = 0.8 if factor is None or isinstance(factor, bool) else factor
        self._apply_zoom(f)
        self._redraw()

    def zoom_out(self, factor: Optional[float] = None) -> None:
        """Zoom out on the visible time viewport."""
        f = 1.25 if factor is None or isinstance(factor, bool) else factor
        self._apply_zoom(f)
        self._redraw()

    def reset_view(self) -> None:
        """Reset view to auto-follow full mission time and auto-scale Y."""
        data_min = 0.0
        data_max = max(self._latest_x, self.zoom_spec.max_zoom_span_s)
        self._view_x_min = data_min
        self._view_x_max = data_max
        self._x_span = data_max - data_min
        self._follow = True
        self._manual_y = False
        self._dirty = True
        self._redraw()

    # ── data in ──────────────────────────────────────────────────────────

    def add_point(self, x: float, y: float, trace: str = "",
                  tier: LossTier = LossTier.NONE,
                  state: Optional[FlightState] = None,
                  v_entry: Optional[float] = None) -> None:
        """Add one sample.

        `tier` comes from the parser (§7.5.2) rather than being derived
        here — see the module docstring. `state` is the vehicle's reported
        state, needed by §7.5.6's state-driven reconciliation.
        """
        name = trace or self._traces[0][0]
        buf = self._buffers.get(name)
        if buf is None:
            return

        if state is not None:
            self._state_now = state
        if v_entry is not None:
            self._v_entry = v_entry

        # ── close an open outage, if any (§7.5.4) ────────────────────────
        if self._outage.is_open:
            resolved = self._outage.close_outage(x, y, self._state_now)
            self._clear_placeholder()
            if resolved is not None:
                self._draw_resolved(resolved)

        # ── open a new one, or note a break (§7.5.2) ─────────────────────
        if buf.xs:
            last_x, last_y = buf.xs[-1], buf.ys[-1]
            if tier is LossTier.OUTAGE:
                self._outage.open_outage(last_x, last_y, self._state_now,
                                         v_entry=self._v_entry)
            elif tier in (LossTier.GAP, LossTier.DROPOUT):
                # §7.5.2 — break the line, no interpolation, marked
                # subtly. Recorded so the renderer can split the curve.
                self._outage.note_break(last_x, x)

        buf.append(x, y)
        self._latest_x = max(self._latest_x, x)
        if self._follow:
            self._view_x_min = 0.0
            self._view_x_max = max(self._latest_x, self.zoom_spec.max_zoom_span_s)
            self._x_span = self._view_x_max - self._view_x_min
        self._dirty = True

        if hasattr(self, "lbl_value"):
            if not self._multi:
                if self.axis_key == "pressure":
                    self.lbl_value.setText(f"{y:.1f} Pa")
                elif self.axis_key == "orientation":
                    self.lbl_value.setText(f"{y:.2f} deg/s")
                elif self.axis_key == "altitude":
                    self.lbl_value.setText(f"{y:.2f} m")
                elif self.axis_key in ("velocity", "descent_rate"):
                    self.lbl_value.setText(f"{y:.2f} m/s")
                elif self.axis_key == "temperature":
                    self.lbl_value.setText(f"{y:.2f} °C")
                elif self.axis_key == "acceleration":
                    self.lbl_value.setText(f"{y:.2f} m/s²")
                else:
                    unit_str = f" {self.spec.unit}" if self.spec.unit else ""
                    self.lbl_value.setText(f"{y:.2f}{unit_str}")
            else:
                self.lbl_value.setText(f"{y:.2f}")

    def mark_annotated_outage(self, x: float, y: float, cause: str) -> None:
        """§7.5.10 — a refresh (§10.3) or source drop (§3.9) dropout is
        annotated as such rather than reconciled blindly."""
        self._outage.open_outage(x, y, self._state_now,
                                 v_entry=self._v_entry, cause=cause)

    def clear(self) -> None:
        for b in self._buffers.values():
            b.clear()
        self._outage.reset()
        self._clear_placeholder()
        for item in self._resolved_items:
            self.plot.removeItem(item)
        self._resolved_items.clear()
        self._latest_x = 0.0
        if hasattr(self, "lbl_value"):
            self.lbl_value.setText("—")
        self.reset_view()

    # ── rendering ────────────────────────────────────────────────────────
    def showEvent(self, ev) -> None:
        super().showEvent(ev)
        if self._dirty:
            self._maybe_redraw()

    def _maybe_redraw(self) -> None:
        if not self._dirty or not self.isVisible():
            return
        self._dirty = False
        self._redraw()

    def _redraw(self) -> None:
        # ── x window (§7.3) ──────────────────────────────────────────────
        if self._follow:
            data_min = 0.0
            data_max = max(self._latest_x, self.zoom_spec.max_zoom_span_s)
            self._view_x_min = data_min
            self._view_x_max = data_max
            self._x_span = data_max - data_min
            x_lo = self._view_x_min
            x_hi = self._view_x_max
        else:
            x_lo = self._view_x_min
            x_hi = self._view_x_max
        self.vb.setXRange(x_lo, x_hi, padding=0)

        # ── traces, decimated per pixel column (§13.2) ───────────────────
        columns = max(64, int(self.plot.width()))
        lo_all: Optional[float] = None
        hi_all: Optional[float] = None

        # §13.2 / Fix 2 — trim break records that are older than the
        # buffer's oldest retained sample; they can never be drawn and
        # their accumulation makes _split_on_breaks() O(N) on every frame.
        oldest_xs = [buf.xs[0] for buf in self._buffers.values() if buf.xs]
        if oldest_xs:
            self._outage.trim_breaks(min(oldest_xs))

        for name, _ in self._traces:
            buf = self._buffers[name]
            xs, ys = buf.window(x_lo, x_hi)
            if not xs:
                continue

            xs, ys = self._split_on_breaks(xs, ys)
            # §13.2 — decimate each NaN-separated segment independently so
            # that gap boundaries (NaN sentinels) survive the decimation
            # step. Passing the NaN-marked arrays straight to minmax_decimate
            # corrupts them: the decimator treats NaN as a numeric value and
            # the connect="finite" breaks no longer land where they should.
            dx, dy = _decimate_with_nans(xs, ys, columns)
            self._curves[name].setData(dx, dy, connect="finite")

            valid_ys = [v for v in ys if v == v]
            if valid_ys:
                b_lo, b_hi = min(valid_ys), max(valid_ys)
                lo_all = b_lo if lo_all is None else min(lo_all, b_lo)
                hi_all = b_hi if hi_all is None else max(hi_all, b_hi)

            if name in self._labels and xs:
                # Walk back to the last real sample — ys[-1] can be a NaN
                # sentinel inserted by _split_on_breaks().  Positioning a
                # TextItem at NaN silently drops it off-screen.
                label_x, label_y = xs[-1], ys[-1]
                for lx, ly in zip(reversed(xs), reversed(ys)):
                    if ly == ly:   # NaN != NaN is always True
                        label_x, label_y = lx, ly
                        break
                self._labels[name].setPos(label_x, label_y)

        # ── y range ──────────────────────────────────────────────────────
        if not self._manual_y:
            y_lo, y_hi = axspec.clamp_range(self.spec, lo_all, hi_all)
            self._view_y_min = y_lo
            self._view_y_max = y_hi
        else:
            y_lo = self._view_y_min
            y_hi = self._view_y_max
        self.vb.setYRange(y_lo, y_hi, padding=0)

        # ── §7.5.3 placeholder ───────────────────────────────────────────
        ph = self._outage.placeholder(self._latest_x)
        if ph is not None:
            self._draw_placeholder(ph)

    def _split_on_breaks(self, xs: List[float], ys: List[float]
                         ) -> Tuple[List[float], List[float]]:
        """Insert NaN at GAP boundaries so the curve breaks.

        NaN with connect="finite" is how pyqtgraph draws a discontinuity.
        The alternative — one PlotDataItem per fragment — would allocate a
        new item per gap and leak them over a lossy flight.
        """
        if not self._outage.breaks or not xs:
            return xs, ys
        breaks = sorted([b for b in self._outage.breaks if xs[0] <= b[1] and b[0] <= xs[-1]], key=lambda b: b[0])
        if not breaks:
            return xs, ys
        out_x, out_y = [], []
        bi = 0
        for i in range(len(xs)):
            x, y = xs[i], ys[i]
            if i > 0:
                prev_x = xs[i - 1]
                while bi < len(breaks) and breaks[bi][1] <= prev_x:
                    bi += 1
                if bi < len(breaks) and (
                    (breaks[bi][0] <= prev_x and breaks[bi][1] >= x) or
                    (prev_x <= breaks[bi][0] and breaks[bi][1] <= x) or
                    (breaks[bi][0] < x < breaks[bi][1])
                ):
                    out_x.append(0.5 * (prev_x + x))
                    out_y.append(float("nan"))
            out_x.append(x)
            out_y.append(y)
        return out_x, out_y

    def _draw_placeholder(self, pts: List[Tuple[float, float]]) -> None:
        c = ThemeManager.C()
        if self._placeholder_item is None:
            # §7.5.3 — dashed and greyed. Not a solid flat line, which
            # reads as "altitude held constant" and is misleading
            # telemetry.
            pen = pg.mkPen(QColor(c.TEXT_MUTED), width=2,
                           style=Qt.PenStyle.DashLine)
            self._placeholder_item = self.plot.plot([], [], pen=pen)
        self._placeholder_item.setData([p[0] for p in pts], [p[1] for p in pts])

    def _clear_placeholder(self) -> None:
        # §7.5.4 — delete the placeholder, do not leave it under the
        # reconciliation segment.
        if self._placeholder_item is not None:
            self.plot.removeItem(self._placeholder_item)
            self._placeholder_item = None

    def _draw_resolved(self, resolved) -> None:
        c = ThemeManager.C()
        # §7.5.8 — reconciled segments render dotted. They are display
        # aids, never data, and are never written to the raw log (§12.3).
        colour = c.AMBER if resolved.annotated else c.TEXT_DIM
        pen = pg.mkPen(QColor(colour), width=2, style=Qt.PenStyle.DotLine)
        item = self.plot.plot([p[0] for p in resolved.arc.points],
                              [p[1] for p in resolved.arc.points], pen=pen)
        self._resolved_items.append(item)

    # ── introspection, used by the page and the health strip ─────────────

    @property
    def outage_count(self) -> int:
        return len(self._outage.resolved)

    def latest(self, trace: str = "") -> Optional[Tuple[float, float]]:
        buf = self._buffers[trace or self._traces[0][0]]
        if not buf.xs:
            return None
        return buf.xs[-1], buf.ys[-1]
