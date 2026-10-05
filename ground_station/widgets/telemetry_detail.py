"""
ground_station/widgets/telemetry_detail.py
══════════════════════════════════════════
Avionics Telemetry Detail widget matching Images 2 & 4.
  - Image 2: Rocket Telemetry Detail (5 gauges, Rocket Attitude, Rocket Data, Rocket Phase bar)
  - Image 4: CanSat Telemetry Detail (4 gauges, CanSat Attitude, CanSat Data, CanSat Phase bar)
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from PyQt6.QtCore import QPointF, QRectF, Qt
from PyQt6.QtGui import (
    QBrush, QColor, QFont, QFontMetrics, QPainter, QPainterPath, QPen, QPolygonF,
)
from PyQt6.QtWidgets import (
    QFrame, QGridLayout, QHBoxLayout, QLabel, QSizePolicy, QVBoxLayout, QWidget,
)

from ..models import FlightState, TelemetryPacket, VehicleID


FONT_TITLE = "Segoe UI"
FONT_MONO = "Consolas"


# ─────────────────────────────────────────────────────────────────────────────
#  1. Circular Gauge Widget (Matching Gauges in Images 2 & 4)
# ─────────────────────────────────────────────────────────────────────────────

class CircularGauge(QWidget):
    """Circular gauge dial with tick marks, warning zones, green needle and digital readout."""

    START_ANGLE = 220.0  # degrees from 3 o'clock (east)
    SWEEP = 260.0        # total sweep clockwise

    def __init__(
        self,
        title: str,
        min_val: float,
        max_val: float,
        major_step: float,
        unit: str,
        decimals: int = 1,
        warning_zones: Optional[List[Tuple[float, float, str]]] = None,
        parent=None,
    ):
        super().__init__(parent)
        self.title = title
        self.min_val = min_val
        self.max_val = max_val
        self.major_step = major_step
        self.unit = unit
        self.decimals = decimals
        self.warning_zones = warning_zones or []

        self._value: Optional[float] = None
        self.setMinimumSize(140, 140)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_value(self, val: Optional[float]) -> None:
        if val != self._value:
            self._value = val
            self.update()

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setRenderHint(QPainter.RenderHint.TextAntialiasing)

        w = float(self.width())
        h = float(self.height())

        # ── Title at the top ──────────────────────────────────────────────────
        title_font_size = max(8, int(min(w, h) * 0.052))
        title_font = QFont(FONT_TITLE, title_font_size, QFont.Weight.Bold)
        p.setFont(title_font)
        p.setPen(QPen(QColor("#00bcd4")))
        title_rect = QRectF(0, 6, w, float(title_font_size + 10))
        p.drawText(title_rect, Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop, self.title)

        # ── Dial geometry ─────────────────────────────────────────────────────
        top_offset = float(title_font_size + 14)
        available_h = h - top_offset - 8.0
        side = min(w - 16.0, available_h)
        side = max(side, 60.0)
        cx = w / 2.0
        cy = top_offset + (available_h / 2.0)
        radius = side / 2.0

        # Background circular track
        track_w = max(3.5, side * 0.024)
        track_pen = QPen(QColor("#182538"), track_w)
        p.setPen(track_pen)
        p.setBrush(Qt.BrushStyle.NoBrush)
        qt_start = int(self.START_ANGLE * 16)
        qt_sweep = int(-self.SWEEP * 16)
        margin = max(4.0, side * 0.028)
        dial_rect = QRectF(cx - radius + margin, cy - radius + margin, 2 * (radius - margin), 2 * (radius - margin))
        p.drawArc(dial_rect, qt_start, qt_sweep)

        # ── Warning zones / colored arcs ──────────────────────────────────────
        span = self.max_val - self.min_val
        if span > 0:
            for z_min, z_max, color_hex in self.warning_zones:
                f1 = max(0.0, min(1.0, (z_min - self.min_val) / span))
                f2 = max(0.0, min(1.0, (z_max - self.min_val) / span))
                a1 = self.START_ANGLE - self.SWEEP * f1
                a2 = self.START_ANGLE - self.SWEEP * f2
                z_start = int(a1 * 16)
                z_sweep = int(-(a1 - a2) * 16)
                warn_w = max(4.5, side * 0.032)
                p.setPen(QPen(QColor(color_hex), warn_w))
                p.drawArc(dial_rect, z_start, z_sweep)

        # ── Ticks and Graduation Labels ───────────────────────────────────────
        num_major = int(round(span / self.major_step)) + 1 if self.major_step > 0 else 5
        step_mult = 1
        if num_major > 12:
            step_mult = 2
        elif num_major > 22:
            step_mult = 4

        num_ticks = max(num_major - 1, 1) * 2
        lbl_font_size = max(6, int(side * 0.040))
        lbl_font = QFont(FONT_TITLE, lbl_font_size, QFont.Weight.Bold)
        p.setFont(lbl_font)
        fm = QFontMetrics(lbl_font)

        major_len = max(6.0, side * 0.048)
        minor_len = max(3.0, side * 0.025)

        for i in range(num_ticks + 1):
            frac = i / num_ticks
            ang_deg = self.START_ANGLE - self.SWEEP * frac
            ang_rad = math.radians(ang_deg)

            is_major = (i % 2 == 0)
            major_idx = i // 2
            tick_len = major_len if is_major else minor_len
            p.setPen(QPen(QColor("#546e7a" if is_major else "#263238"), max(1.2, side * 0.009) if is_major else 1.0))

            outer_r = radius - margin
            inner_r = outer_r - tick_len
            p.drawLine(
                int(cx + inner_r * math.cos(ang_rad)),
                int(cy - inner_r * math.sin(ang_rad)),
                int(cx + outer_r * math.cos(ang_rad)),
                int(cy - outer_r * math.sin(ang_rad)),
            )

            # Major tick label
            if is_major and (major_idx % step_mult == 0):
                val = self.min_val + frac * span
                if abs(span) >= 1000:
                    lbl = f"{val:.0f}"
                elif abs(span) >= 10:
                    lbl = f"{val:.0f}"
                else:
                    lbl = f"{val:.1f}"

                p.setPen(QPen(QColor("#90a4ae")))
                tw = fm.horizontalAdvance(lbl)
                th = fm.height()

                label_r = inner_r - max(6.0, side * 0.045)
                lx = cx + label_r * math.cos(ang_rad)
                ly = cy - label_r * math.sin(ang_rad)
                p.drawText(int(lx - tw / 2), int(ly + th / 3), lbl)

        # ── Needle (Neon Green) ───────────────────────────────────────────────
        v = self._value
        val_to_draw = v if v is not None else self.min_val
        frac = 0.0
        if span > 0:
            frac = max(0.0, min(1.0, (val_to_draw - self.min_val) / span))
        needle_ang_deg = self.START_ANGLE - self.SWEEP * frac
        needle_ang_rad = math.radians(needle_ang_deg)

        needle_len = radius - max(14.0, side * 0.09)
        nx = cx + needle_len * math.cos(needle_ang_rad)
        ny = cy - needle_len * math.sin(needle_ang_rad)

        needle_color = QColor("#00e676")
        needle_w = max(2.5, side * 0.017)
        p.setPen(QPen(needle_color, needle_w, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        p.drawLine(int(cx), int(cy), int(nx), int(ny))

        # Pivot circle
        p.setBrush(QBrush(needle_color))
        p.setPen(Qt.PenStyle.NoPen)
        pivot_r = max(3.5, side * 0.024)
        p.drawEllipse(QPointF(cx, cy), pivot_r, pivot_r)

        # ── Digital Readout & Unit (Neon Green) ──────────────────────────────
        val_str = f"{v:,.{self.decimals}f}" if v is not None else "0.0"
        val_font_size = max(10, int(side * 0.076))
        val_font = QFont(FONT_TITLE, val_font_size, QFont.Weight.Bold)
        p.setFont(val_font)
        p.setPen(QPen(needle_color))
        vfm = QFontMetrics(val_font)
        vtw = vfm.horizontalAdvance(val_str)
        readout_y = cy + (radius * 0.33)
        p.drawText(int(cx - vtw / 2), int(readout_y), val_str)

        unit_font_size = max(7, int(side * 0.050))
        unit_font = QFont(FONT_TITLE, unit_font_size, QFont.Weight.Bold)
        p.setFont(unit_font)
        p.setPen(QPen(needle_color))
        ufm = QFontMetrics(unit_font)
        utw = ufm.horizontalAdvance(self.unit)
        p.drawText(int(cx - utw / 2), int(readout_y + vfm.height() * 0.72), self.unit)

        p.end()


# ─────────────────────────────────────────────────────────────────────────────
#  2. Attitude Indicator Widget (Artificial Horizon)
# ─────────────────────────────────────────────────────────────────────────────

class AttitudeIndicator(QWidget):
    """Artificial Horizon with Pitch Ladder, Roll Rotation, and Angle Readout."""

    def __init__(self, title: str = "ROCKET ATTITUDE", parent=None):
        super().__init__(parent)
        self.title = title
        self.roll = 0.0   # deg (+ = roll right)
        self.pitch = 0.0  # deg (+ = pitch up)
        self.yaw = 0.0    # deg
        self.setMinimumSize(170, 170)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_attitude(self, roll: float, pitch: float, yaw: float) -> None:
        if roll != self.roll or pitch != self.pitch or yaw != self.yaw:
            self.roll = roll
            self.pitch = pitch
            self.yaw = yaw
            self.update()

    def set_imu_data(
        self,
        accel_x: Optional[float],
        accel_y: Optional[float],
        accel_z: Optional[float],
        gyro_spin: Optional[float] = None,
        extras: Optional[Dict[str, str]] = None,
    ) -> None:
        r, p, y = 0.0, 0.0, 0.0
        if extras:
            if "ROLL" in extras or "roll" in extras:
                try:
                    r = float(extras.get("ROLL") or extras.get("roll"))
                except ValueError:
                    pass
            if "PITCH" in extras or "pitch" in extras:
                try:
                    p = float(extras.get("PITCH") or extras.get("pitch"))
                except ValueError:
                    pass
            if "YAW" in extras or "yaw" in extras:
                try:
                    y = float(extras.get("YAW") or extras.get("yaw"))
                except ValueError:
                    pass

        if r == 0.0 and p == 0.0 and accel_x is not None and accel_y is not None and accel_z is not None:
            denom = math.sqrt(accel_y**2 + accel_z**2)
            if denom > 0.001 or abs(accel_x) > 0.001:
                r = math.degrees(math.atan2(accel_y, accel_z if abs(accel_z) > 0.001 else 0.001))
                p = math.degrees(math.atan2(-accel_x, denom))
        if y == 0.0 and gyro_spin is not None:
            y = gyro_spin

        self.set_attitude(r, p, y)

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setRenderHint(QPainter.RenderHint.TextAntialiasing)

        w = float(self.width())
        h = float(self.height())

        # ── Title at top ──────────────────────────────────────────────────────
        title_font = QFont(FONT_TITLE, 8, QFont.Weight.Bold)
        p.setFont(title_font)
        p.setPen(QPen(QColor("#00bcd4")))
        title_rect = QRectF(0, 4, w, 16)
        p.drawText(title_rect, Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop, self.title)

        # ── Circle geometry ───────────────────────────────────────────────────
        avail_h = h - 42.0
        avail_w = w - 16.0
        diameter = min(avail_w, avail_h)
        diameter = max(diameter, 70.0)
        radius = diameter / 2.0
        cx = w / 2.0
        cy = 18.0 + (avail_h / 2.0)

        # ── Clip inside sphere ────────────────────────────────────────────────
        clip_path = QPainterPath()
        clip_rect = QRectF(cx - radius, cy - radius, diameter, diameter)
        clip_path.addEllipse(clip_rect)
        p.save()
        p.setClipPath(clip_path)

        # Transform for Roll and Pitch
        p.save()
        p.translate(cx, cy)
        p.rotate(-self.roll)

        pitch_px_per_deg = radius / 40.0
        dy = self.pitch * pitch_px_per_deg
        dy = max(-radius * 1.5, min(radius * 1.5, dy))
        p.translate(0, dy)

        # Sky (dark navy)
        sky_rect = QRectF(-radius * 2, -radius * 3, radius * 4, radius * 3)
        p.fillRect(sky_rect, QColor("#0a2240"))

        # Earth (rich brown)
        earth_rect = QRectF(-radius * 2, 0, radius * 4, radius * 3)
        p.fillRect(earth_rect, QColor("#4e2a0e"))

        # Horizon Line (white)
        p.setPen(QPen(QColor(255, 255, 255), 2))
        p.drawLine(QPointF(-radius * 1.5, 0), QPointF(radius * 1.5, 0))

        # Pitch Ladder
        ladder_font = QFont(FONT_TITLE, 6, QFont.Weight.Bold)
        p.setFont(ladder_font)
        p.setPen(QPen(QColor("#d4a373"), 1.5))
        fm = QFontMetrics(ladder_font)

        for deg in (20, 10, -10, -20):
            ly = -deg * pitch_px_per_deg
            lw = radius * 0.35 if abs(deg) == 20 else radius * 0.20
            p.drawLine(QPointF(-lw, ly), QPointF(lw, ly))

            lbl = f"{deg:+d}" if deg > 0 else f"{deg}"
            p.drawText(int(lw + 4), int(ly + fm.height() / 3), lbl)

        p.restore()  # restore translation
        p.restore()  # restore clipping

        # ── Center Airplane Symbol (Golden / Yellow) ──────────────────────────
        gold_pen = QPen(QColor("#f59e0b"), 2.5)
        p.setPen(gold_pen)
        wing_span = radius * 0.35
        wing_gap = radius * 0.08
        p.drawLine(QPointF(cx - wing_span, cy), QPointF(cx - wing_gap, cy))
        p.drawLine(QPointF(cx + wing_gap, cy), QPointF(cx + wing_span, cy))
        p.drawLine(QPointF(cx, cy - 7), QPointF(cx, cy + 5))

        # ── Top Index Marker (Yellow Triangle) ────────────────────────────────
        tri_h = 7.0
        tri_w = 6.0
        tri = QPolygonF([
            QPointF(cx, cy - radius + 2),
            QPointF(cx - tri_w, cy - radius + 2 + tri_h),
            QPointF(cx + tri_w, cy - radius + 2 + tri_h),
        ])
        p.setBrush(QBrush(QColor("#f59e0b")))
        p.drawPolygon(tri)

        # ── Outer Bezel ───────────────────────────────────────────────────────
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(QColor("#1e2d42"), 2.5))
        p.drawEllipse(clip_rect)

        # ── R/P/Y readout at bottom ───────────────────────────────────────────
        r_str = f"R {self.roll:+0.1f}°  P {self.pitch:+0.1f}°  Y {self.yaw:+0.1f}°"
        rpy_font = QFont(FONT_TITLE, 7, QFont.Weight.Bold)
        p.setFont(rpy_font)
        p.setPen(QPen(QColor("#38bdf8")))
        bottom_rect = QRectF(0, h - 18, w, 16)
        p.drawText(bottom_rect, Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter, r_str)

        p.end()


# ─────────────────────────────────────────────────────────────────────────────
#  3. Telemetry Data Panel (Right side slot rows)
# ─────────────────────────────────────────────────────────────────────────────

class TelemetryDataPanel(QWidget):
    """Dual-capsule slot rows for vehicle telemetry parameters."""

    def __init__(self, title: str, items: List[str], parent=None):
        super().__init__(parent)
        self.title = title
        self.item_keys = items
        self._values: Dict[str, QLabel] = {}
        self._build()

    def _build(self) -> None:
        outer = QVBoxLayout(self)
        outer.setContentsMargins(6, 6, 6, 6)
        outer.setSpacing(4)

        title_lbl = QLabel(self.title)
        title_lbl.setFont(QFont(FONT_TITLE, 8, QFont.Weight.Bold))
        title_lbl.setStyleSheet("color: #00bcd4;")
        title_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        outer.addWidget(title_lbl)

        for key in self.item_keys:
            row_widget = QWidget()
            row_lay = QHBoxLayout(row_widget)
            row_lay.setContentsMargins(0, 0, 0, 0)
            row_lay.setSpacing(4)

            # Left capsule (label)
            lbl_frame = QFrame()
            lbl_frame.setStyleSheet(
                "QFrame { background-color: #070e1b; border: 1px solid #14243a; border-radius: 4px; }"
            )
            lbl_lay = QHBoxLayout(lbl_frame)
            lbl_lay.setContentsMargins(8, 3, 8, 3)
            lbl = QLabel(key)
            lbl.setFont(QFont(FONT_TITLE, 7, QFont.Weight.Bold))
            lbl.setStyleSheet("color: #90a4ae; border: none;")
            lbl_lay.addWidget(lbl)
            row_lay.addWidget(lbl_frame, stretch=1)

            # Right capsule (value)
            val_frame = QFrame()
            val_frame.setStyleSheet(
                "QFrame { background-color: #070e1b; border: 1px solid #14243a; border-radius: 4px; }"
            )
            val_lay = QHBoxLayout(val_frame)
            val_lay.setContentsMargins(8, 3, 8, 3)
            val_lbl = QLabel("—")
            val_lbl.setFont(QFont(FONT_TITLE, 7, QFont.Weight.Bold))
            val_lbl.setStyleSheet("color: #eceff1; border: none;")
            val_lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            val_lay.addWidget(val_lbl)
            row_lay.addWidget(val_frame, stretch=2)

            self._values[key] = val_lbl
            outer.addWidget(row_widget)

        outer.addStretch()

    def set_field(self, key: str, value_text: str) -> None:
        if key in self._values:
            self._values[key].setText(value_text)

    def reset_fields(self) -> None:
        for val_lbl in self._values.values():
            val_lbl.setText("—")


# ─────────────────────────────────────────────────────────────────────────────
#  4. Phase Tracker Bar (Top Header / Flight State Bar)
# ─────────────────────────────────────────────────────────────────────────────

class PhaseTrackerBar(QWidget):
    """Header bar with Telemetry Detail Title and Flight Phase pills."""

    def __init__(self, vehicle_id: VehicleID, parent=None):
        super().__init__(parent)
        self.vehicle_id = vehicle_id

        if vehicle_id == VehicleID.ROCKET:
            self.title_text = "ROCKET TELEMETRY DETAIL"
            self.phases = ["Boot", "Pre_launch", "ascent", "Deployment", "Descent", "Impact/recovery"]
            self.active_color = "#00e676"
            self.active_text_color = "#022013"
        else:
            self.title_text = "CANSAT TELEMETRY DETAIL"
            self.phases = ["IDLE", "ARMED", "DEPLOY", "FALL", "LAND", "POST"]
            self.active_color = "#1d72d6"
            self.active_text_color = "#ffffff"

        self._active_phase: str = self.phases[0]
        self._buttons: Dict[str, QLabel] = {}
        self._build()

    def _build(self) -> None:
        lay = QHBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(6)

        title_lbl = QLabel(self.title_text)
        title_lbl.setFont(QFont(FONT_TITLE, 10, QFont.Weight.Bold))
        title_lbl.setStyleSheet("color: #00bcd4; padding-right: 12px;")
        lay.addWidget(title_lbl)

        for phase in self.phases:
            p_lbl = QLabel(phase)
            p_lbl.setFont(QFont(FONT_TITLE, 7, QFont.Weight.Bold))
            p_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            p_lbl.setFixedHeight(22)
            p_lbl.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            lay.addWidget(p_lbl)
            self._buttons[phase] = p_lbl

        self._update_styles()

    def set_phase(self, phase_name: str) -> None:
        target = self._normalize_phase(phase_name)
        if target != self._active_phase:
            self._active_phase = target
            self._update_styles()

    def _normalize_phase(self, phase_name: str) -> str:
        s = phase_name.strip().upper()
        if self.vehicle_id == VehicleID.ROCKET:
            if s in ("BOOT", "INIT"):
                return "Boot"
            elif s in ("PRE_LAUNCH", "PRELAUNCH", "PAD", "ARMED"):
                return "Pre_launch"
            elif s in ("ASCENT", "BOOST", "COAST", "POWERED"):
                return "ascent"
            elif s in ("DEPLOYMENT", "APOGEE", "DEPLOY"):
                return "Deployment"
            elif s in ("DESCENT", "FALL", "PARACHUTE"):
                return "Descent"
            elif s in ("IMPACT", "RECOVERY", "LANDING", "LAND", "IMPACT/RECOVERY"):
                return "Impact/recovery"
        else:
            if s in ("IDLE", "BOOT"):
                return "IDLE"
            elif s in ("ARMED", "PRE_LAUNCH", "BOOST", "COAST"):
                return "ARMED"
            elif s in ("DEPLOY", "DEPLOYMENT", "APOGEE"):
                return "DEPLOY"
            elif s in ("FALL", "DESCENT"):
                return "FALL"
            elif s in ("LAND", "LANDING"):
                return "LAND"
            elif s in ("POST", "RECOVERY", "IMPACT"):
                return "POST"
        return self._active_phase

    def _update_styles(self) -> None:
        for phase, lbl in self._buttons.items():
            if phase == self._active_phase:
                lbl.setStyleSheet(
                    f"QLabel {{ background-color: {self.active_color}; "
                    f"color: {self.active_text_color}; border-radius: 2px; "
                    f"padding: 2px 6px; font-weight: bold; }}"
                )
            else:
                lbl.setStyleSheet(
                    "QLabel { background-color: #0b1526; color: #4b6a8a; "
                    "border-radius: 2px; padding: 2px 6px; }"
                )


# ─────────────────────────────────────────────────────────────────────────────
#  5. Full Telemetry Detail Section (Matching Images 3 & 4)
# ─────────────────────────────────────────────────────────────────────────────

class TelemetryDetailWidget(QWidget):
    """Complete Telemetry Detail section containing full-width Circular Gauges."""

    def __init__(self, vehicle_id: VehicleID, parent=None):
        super().__init__(parent)
        self.vehicle_id = vehicle_id

        self.gauges: Dict[str, CircularGauge] = {}
        self.phase_bar = None
        self.attitude = None
        self.data_panel = None
        self._build()

    def _make_card_frame(self, widget: QWidget) -> QFrame:
        frame = QFrame()
        frame.setStyleSheet(
            "QFrame { background-color: #080f1d; border: 1px solid #13243a; border-radius: 6px; }"
        )
        frame.setMinimumHeight(190)
        frame.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        lay = QVBoxLayout(frame)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.addWidget(widget)
        return frame

    def _build(self) -> None:
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(8)

        # ── Full-Width Gauges Grid ───────────────────────────────────────────
        gauges_container = QWidget()
        gauges_lay = QGridLayout(gauges_container)
        gauges_lay.setContentsMargins(0, 0, 0, 0)
        gauges_lay.setSpacing(8)

        if self.vehicle_id == VehicleID.ROCKET:
            # 5 gauges: Alt, Vel, Accel on top row; Press, Temp on bottom row (Image 3)
            g_alt = CircularGauge("ALTITUDE", 0.0, 2000.0, 200.0, "m", decimals=1)
            g_vel = CircularGauge("VELOCITY", 0.0, 200.0, 20.0, "m/s", decimals=1)
            g_acc = CircularGauge("ACCEL", -20.0, 20.0, 4.0, "g", decimals=1)
            g_prs = CircularGauge("PRESSURE", 50000.0, 100000.0, 10000.0, "Pa", decimals=1)
            g_tmp = CircularGauge("TEMP", -40.0, 80.0, 20.0, "°C", decimals=1)

            self.gauges["altitude"] = g_alt
            self.gauges["velocity"] = g_vel
            self.gauges["accel_magnitude"] = g_acc
            self.gauges["pressure"] = g_prs
            self.gauges["temperature"] = g_tmp

            gauges_lay.addWidget(self._make_card_frame(g_alt), 0, 0)
            gauges_lay.addWidget(self._make_card_frame(g_vel), 0, 1)
            gauges_lay.addWidget(self._make_card_frame(g_acc), 0, 2)
            gauges_lay.addWidget(self._make_card_frame(g_prs), 1, 0)
            gauges_lay.addWidget(self._make_card_frame(g_tmp), 1, 1)
        else:
            # 4 gauges (CanSat 2x2): Alt, Descent on top; Press, Temp on bottom (Image 4)
            g_alt = CircularGauge("ALTITUDE", 0.0, 2000.0, 200.0, "m", decimals=1)
            g_des = CircularGauge("DESCENT", 0.0, 50.0, 5.0, "m/s", decimals=1)
            g_prs = CircularGauge("PRESSURE", 50000.0, 100000.0, 10000.0, "Pa", decimals=1)
            g_tmp = CircularGauge("TEMP", -40.0, 80.0, 20.0, "°C", decimals=1)

            self.gauges["altitude"] = g_alt
            self.gauges["velocity"] = g_des
            self.gauges["pressure"] = g_prs
            self.gauges["temperature"] = g_tmp

            gauges_lay.addWidget(self._make_card_frame(g_alt), 0, 0)
            gauges_lay.addWidget(self._make_card_frame(g_des), 0, 1)
            gauges_lay.addWidget(self._make_card_frame(g_prs), 1, 0)
            gauges_lay.addWidget(self._make_card_frame(g_tmp), 1, 1)

        outer.addWidget(gauges_container)

    def update_telemetry(
        self,
        packet: TelemetryPacket,
        derived_velocity: Optional[float] = None,
    ) -> None:
        """Update all visual elements from incoming packet."""
        v_final = packet.velocity if packet.velocity is not None else derived_velocity

        # 1. Update gauges
        if "altitude" in self.gauges:
            self.gauges["altitude"].set_value(packet.altitude)
        if "velocity" in self.gauges and v_final is not None:
            self.gauges["velocity"].set_value(v_final)
        if "pressure" in self.gauges:
            self.gauges["pressure"].set_value(packet.pressure)
        if "temperature" in self.gauges:
            self.gauges["temperature"].set_value(packet.temperature)
        if "accel_magnitude" in self.gauges:
            acc_val = packet.accel_magnitude
            if acc_val is not None:
                acc_g = acc_val / 9.80665 if acc_val > 50 else acc_val
                self.gauges["accel_magnitude"].set_value(acc_g)

        # 2. Update phase bar if present
        if self.phase_bar is not None and packet.state is not None:
            self.phase_bar.set_phase(packet.state.value)
