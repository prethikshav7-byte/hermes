"""
ground_station/app.py
═════════════════════
The application shell. Replaces the previous app.py entirely — that one
was built around RocketTelemetry / CanSatTelemetry and two unrelated
state enums, which §4.1 and §5.1 collapsed into one packet type and one
8-state machine.

Responsibilities, and nothing beyond them (§1.2): navigation, wiring the
link layer to the dashboards, and the §6.5 health strip. It computes no
telemetry values and makes no decisions — §1.4, "the app has no autonomy
and no watchdog that actuates anything", is a property of there being no
code here that could.

Five dashboards are specified (§6). Three exist after Phase 2: Overview,
Rocket and CanSat. Command (§10) and CanSat Experiment (§11) arrive in
Phase 3; their navigation entries are present and disabled, so the shape
of the app is visible and the buttons do not silently do nothing.
"""
from __future__ import annotations

from typing import Dict, List, Optional

import os

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QColor, QFont, QPainter, QPainterPath, QPen, QPixmap
from PyQt6.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QMainWindow, QProgressBar, QPushButton,
    QStackedWidget, QVBoxLayout, QWidget,
)

from .models import (
    ReceiverState, RejectedLine, SourceState, TelemetryPacket, VehicleID,
    VehicleLinkState,
)
from .parser import IngestResult
from .commands import CommandCentre
from .pages.command import CommandPage
from .pages.com import ComPage
from .pages.experiment import ExperimentPage
from .pages.overview import OverviewPage
from .pages.vehicle import VehiclePage
from .theme import ThemeManager


class NavButton(QPushButton):
    def __init__(self, text: str, parent=None):
        super().__init__(text, parent)
        self.nav_key = text
        self.setCheckable(True)
        self.setFont(QFont("Segoe UI", 9, QFont.Weight.DemiBold))
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self._restyle()

    def _restyle(self) -> None:
        c = ThemeManager.C()
        self.setStyleSheet(
            f"QPushButton {{ background-color: transparent; color: {c.TEXT_DIM}; "
            f"border: 1px solid transparent; border-radius: 6px; "
            f"padding: 10px 14px; margin: 2px 10px; text-align: left; "
            f"font-family: 'Segoe UI', Arial, sans-serif; }}"
            f"QPushButton:hover {{ background-color: {c.BG_CARD_HOVER}; "
            f"color: {c.TEXT}; border: 1px solid {c.BORDER}; }}"
            f"QPushButton:checked {{ color: {c.CYAN}; "
            f"background-color: #0e263d; border: 1px solid #0284c7; font-weight: bold; }}"
            f"QPushButton:disabled {{ color: {c.TEXT_MUTED}; }}")


class MachZoneHeader(QWidget):
    """Aerospace header banner with MachZone branding and live connection telemetry."""

    def __init__(self, mode_label: str = "", simulator_mode: bool = False, parent=None):
        super().__init__(parent)
        self.setFixedHeight(54)
        c = ThemeManager.C()

        row = QHBoxLayout(self)
        row.setContentsMargins(16, 0, 16, 0)
        row.setSpacing(12)

        # Brand Container (Left)
        brand_row = QHBoxLayout()
        brand_row.setSpacing(10)

        # Title: MACHZONE
        title = QLabel("MACHZONE")
        title.setFont(QFont("Segoe UI", 12, QFont.Weight.Black))
        title.setStyleSheet("color: #ffffff; letter-spacing: 3px;")
        brand_row.addWidget(title)

        # Divider bar
        div = QLabel("|")
        div.setFont(QFont("Segoe UI", 11, QFont.Weight.Light))
        div.setStyleSheet(f"color: {c.RED}; padding: 0 2px;")
        brand_row.addWidget(div)

        # Tagline / Sublabel
        tagline = QLabel("BEYOND MACH, BEYOND LIMITS")
        tagline.setFont(QFont("Segoe UI", 8, QFont.Weight.DemiBold))
        tagline.setStyleSheet(f"color: {c.TEXT_DIM}; letter-spacing: 2px;")
        brand_row.addWidget(tagline)

        row.addLayout(brand_row)
        row.addStretch(1)

        # Connection Pill
        init_status = "● SIMULATOR" if simulator_mode else "● CONNECTED"
        self.pill_status = QLabel(init_status)
        self.pill_status.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        self.pill_status.setStyleSheet(
            f"background-color: rgba(13, 26, 40, 0.85); color: {c.CYAN if simulator_mode else c.GREEN}; "
            f"border: 1px solid {c.BORDER}; border-radius: 4px; padding: 4px 10px; letter-spacing: 0.5px;"
        )
        row.addWidget(self.pill_status)

        # COM Port Pill
        port_text = "SIM" if simulator_mode else "COM9"
        self.pill_port = QLabel(f"PORT: {port_text}")
        self.pill_port.setFont(QFont("Segoe UI", 8, QFont.Weight.DemiBold))
        self.pill_port.setStyleSheet(
            f"background-color: rgba(13, 26, 40, 0.85); color: {c.TEXT}; "
            f"border: 1px solid {c.BORDER}; border-radius: 4px; padding: 4px 10px;"
        )
        row.addWidget(self.pill_port)

        # Baud Rate Pill
        self.pill_baud = QLabel("Baud 115200")
        self.pill_baud.setFont(QFont("Segoe UI", 8, QFont.Weight.DemiBold))
        self.pill_baud.setStyleSheet(
            f"background-color: rgba(13, 26, 40, 0.85); color: {c.TEXT_DIM}; "
            f"border: 1px solid {c.BORDER}; border-radius: 4px; padding: 4px 10px;"
        )
        row.addWidget(self.pill_baud)

    def paintEvent(self, ev) -> None:
        super().paintEvent(ev)
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        c = ThemeManager.C()

        # Solid background
        p.fillRect(self.rect(), QColor(c.BG_PANEL))

        # Aerodynamic supersonic flow curves in background
        w = self.width()
        h = self.height()
        path1 = QPainterPath()
        path1.moveTo(w * 0.40, h * 0.20)
        path1.cubicTo(w * 0.55, h * 0.15, w * 0.70, h * 0.80, w * 0.95, h * 0.30)
        p.setPen(QPen(QColor(255, 65, 30, 40), 2))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawPath(path1)

        path2 = QPainterPath()
        path2.moveTo(w * 0.45, h * 0.35)
        path2.cubicTo(w * 0.60, h * 0.30, w * 0.75, h * 0.90, w * 0.98, h * 0.45)
        p.setPen(QPen(QColor(249, 115, 22, 28), 1.5))
        p.drawPath(path2)

        # Bottom divider border
        p.setPen(QPen(QColor(c.BORDER), 1))
        p.drawLine(0, h - 1, w, h - 1)
        p.end()

    def set_source_state(self, state: SourceState) -> None:
        c = ThemeManager.C()
        colour = {
            SourceState.CONNECTED: c.GREEN,
            SourceState.CONNECTING: c.AMBER,
            SourceState.NO_DATA: c.AMBER,
            SourceState.LINK_ERROR: c.RED,
            SourceState.DISCONNECTED: c.TEXT_MUTED,
        }.get(state, c.TEXT_MUTED)
        self.pill_status.setText(f"● {state.value}")
        self.pill_status.setStyleSheet(
            f"background-color: rgba(13, 26, 40, 0.85); color: {colour}; "
            f"border: 1px solid {c.BORDER}; border-radius: 4px; padding: 4px 10px; letter-spacing: 0.5px;"
        )

    def update_supervisor_info(self, supervisor) -> None:
        port = getattr(supervisor, "port_name", "—")
        baud = getattr(supervisor, "baudrate", 115200)
        self.pill_port.setText(f"PORT: {port}")
        self.pill_baud.setText(f"Baud {baud}")


class SidebarFooter(QWidget):
    """Bottom slogan badge."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(76)
        c = ThemeManager.C()

        row = QHBoxLayout(self)
        row.setContentsMargins(16, 12, 16, 16)
        row.setSpacing(10)

        # Vertical red accent bar
        bar = QFrame()
        bar.setFixedWidth(2)
        bar.setStyleSheet(f"background-color: {c.RED}; border: none;")
        row.addWidget(bar)

        text_lay = QVBoxLayout()
        text_lay.setContentsMargins(0, 0, 0, 0)
        text_lay.setSpacing(2)

        t1 = QLabel("SMALL STEPS")
        t1.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        t1.setStyleSheet(f"color: {c.TEXT_DIM}; letter-spacing: 1.5px;")

        t2 = QLabel("BIG ALTITUDES")
        t2.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        t2.setStyleSheet(f"color: {c.TEXT_MUTED}; letter-spacing: 1.5px;")

        text_lay.addWidget(t1)
        text_lay.addWidget(t2)
        row.addLayout(text_lay)
        row.addStretch()

    def paintEvent(self, ev) -> None:
        super().paintEvent(ev)


class MainDashboardArea(QWidget):
    """Main dashboard content container."""

    def __init__(self, parent=None):
        super().__init__(parent)

    def paintEvent(self, ev) -> None:
        p = QPainter(self)
        c = ThemeManager.C()
        # Solid base background
        p.fillRect(self.rect(), QColor(c.BG))
        p.end()


class GroundStationApp(QMainWindow):
    """Main window."""

    def __init__(self, mode_label: str = "", simulator_mode: bool = False,
                 parent=None):
        super().__init__(parent)
        self.setWindowTitle(
            f"Ground Station{f' — {mode_label}' if mode_label else ''}")
        self.resize(1560, 960)
        self.simulator_mode = simulator_mode

        self.overview_page = OverviewPage()
        self.pages: Dict[VehicleID, VehiclePage] = {
            v: VehiclePage(v) for v in VehicleID
        }
        self.rocket_page = self.pages[VehicleID.ROCKET]
        self.cansat_page = self.pages[VehicleID.CANSAT]

        for page in self.pages.values():
            page.backward_transition.connect(self._on_notice)

        self.command_centre = CommandCentre(
            transmit=lambda target, command, seq: None)
        self.command_page = CommandPage(self.command_centre, simulator_mode)
        self.command_page.notice.connect(self._on_notice)
        self.experiment_page = ExperimentPage()
        self.com_page = ComPage()

        self._build(mode_label)

        # Status Bar aerospace telemetry footer
        c = ThemeManager.C()
        self._status_telemetry = QLabel("● Receiving Telemetry...")
        self._status_telemetry.setFont(QFont("Segoe UI", 8, QFont.Weight.DemiBold))
        self._status_telemetry.setStyleSheet(f"color: {c.TEXT_DIM}; padding-left: 10px;")
        self.statusBar().addWidget(self._status_telemetry)

        self._status_bar_prog = QProgressBar()
        self._status_bar_prog.setRange(0, 0)
        self._status_bar_prog.setFixedSize(65, 4)
        self._status_bar_prog.setTextVisible(False)
        self._status_bar_prog.setStyleSheet(
            f"QProgressBar {{ background-color: rgba(255,255,255,0.06); border: none; border-radius: 2px; }}"
            f"QProgressBar::chunk {{ background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 {c.RED}, stop:1 {c.ORANGE}); border-radius: 2px; }}"
        )
        self.statusBar().addWidget(self._status_bar_prog)

        self._status_right = QLabel("MACHZONE  |  GROUND STATION  ")
        self._status_right.setFont(QFont("Segoe UI", 8, QFont.Weight.DemiBold))
        self._status_right.setStyleSheet(f"color: {c.TEXT_MUTED}; letter-spacing: 1.5px;")
        self.statusBar().addPermanentWidget(self._status_right)

    # ── construction ─────────────────────────────────────────────────────

    def _build(self, mode_label: str) -> None:
        c = ThemeManager.C()
        app_style = ThemeManager.main_stylesheet()
        self.setStyleSheet(app_style)

        root = QWidget()
        row = QHBoxLayout(root)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(0)

        row.addWidget(self._sidebar(mode_label))

        main_area = MainDashboardArea()
        main_lay = QVBoxLayout(main_area)
        main_lay.setContentsMargins(0, 0, 0, 0)
        main_lay.setSpacing(0)

        self.header = MachZoneHeader(
            mode_label=mode_label, simulator_mode=self.simulator_mode
        )
        main_lay.addWidget(self.header)

        self.stack = QStackedWidget()
        self.stack.addWidget(self.overview_page)
        self.stack.addWidget(self.rocket_page)
        self.stack.addWidget(self.cansat_page)
        self.stack.addWidget(self.command_page)
        self.stack.addWidget(self.experiment_page)
        self.stack.addWidget(self.com_page)
        main_lay.addWidget(self.stack, stretch=1)

        row.addWidget(main_area, stretch=1)

        self.setCentralWidget(root)
        self._nav[0].setChecked(True)

    def _sidebar(self, mode_label: str) -> QWidget:
        c = ThemeManager.C()
        bar = QWidget()
        bar.setFixedWidth(210)
        bar.setStyleSheet(ThemeManager.sidebar_stylesheet())

        lay = QVBoxLayout(bar)
        lay.setContentsMargins(0, 8, 0, 8)
        lay.setSpacing(4)

        if mode_label:
            badge = QLabel(f"  {mode_label}")
            badge.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
            badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
            badge.setStyleSheet(f"color: {c.GREEN}; padding: 14px 12px 10px; letter-spacing: 1.5px;")
            lay.addWidget(badge)

        self._nav: List[NavButton] = []
        entries = [
            ("Overview", 0, True),
            ("Rocket", 1, True),
            ("CanSat", 2, True),
            ("Command", 3, True),
            ("CanSat Experiment", 4, True),
            ("COM", 5, True),
        ]
        for text, index, enabled in entries:
            b = NavButton(text)
            b.setEnabled(enabled)
            b.clicked.connect(lambda _checked, i=index: self._navigate(i))
            self._nav.append(b)
            lay.addWidget(b)

        lay.addStretch()

        footer = SidebarFooter()
        lay.addWidget(footer)
        return bar

    def _navigate(self, index: int) -> None:
        self.stack.setCurrentIndex(index)
        for i, b in enumerate(self._nav):
            b.setChecked(i == index)

    # ── link wiring ──────────────────────────────────────────────────────

    def connect_supervisor(self, supervisor) -> None:
        """Wire a LinkSupervisor (live) or a simulator exposing the same
        signals (§2.2 — only the data source differs)."""
        supervisor.rocket_packet.connect(self._on_rocket)
        supervisor.cansat_packet.connect(self._on_cansat)
        supervisor.source_state_changed.connect(self._on_source_state)
        supervisor.receiver_state_changed.connect(self._on_receiver_state)
        supervisor.vehicle_state_changed.connect(self._on_vehicle_state)
        supervisor.line_rejected.connect(self._on_rejected)
        supervisor.notice.connect(self._on_notice)
        self._supervisor = supervisor
        self.com_page.set_supervisor(supervisor)
        self.header.update_supervisor_info(supervisor)
        # §1.4 — the only place a transmit path is installed, installed
        # once. Nothing in the app calls send() on a timer; every call
        # originates in a widget's clicked handler.
        #
        # The sequence flows OUT of CommandCentre, not into it:
        # CommandCentre owns the §10.7 ack lifecycle, so the number it
        # assigns must be the number on the wire. Letting the link layer
        # assign its own would leave every command permanently
        # unacknowledged while the vehicle replied correctly.
        self.command_centre._transmit = (
            lambda target, command, sequence:
            supervisor.send_command(target, command, sequence))

    _supervisor = None
    _telemetry_connected: bool = False

    def _on_rocket(self, packet: TelemetryPacket, result: IngestResult) -> None:
        self._dispatch(VehicleID.ROCKET, packet, result)

    def _on_cansat(self, packet: TelemetryPacket, result: IngestResult) -> None:
        self._dispatch(VehicleID.CANSAT, packet, result)

    def _dispatch(self, vid: VehicleID, packet: TelemetryPacket,
                  result: IngestResult) -> None:
        if not self._telemetry_connected:
            self._telemetry_connected = True
            self.statusBar().showMessage("PHOENIX telemetry connected", 8000)

        self.command_page.set_vehicle_state(vid, packet.state)
        page = self.pages[vid]
        # The vehicle page owns the one derived-velocity estimator for
        # its vehicle (§13.5 — independent per-vehicle state) and returns
        # the value, so the Overview reuses it rather than running a
        # second estimator that could hold a different window and show a
        # different number for the same instant.
        v = page.receive(packet, result)
        self.overview_page.receive(packet, result, v)
        self.com_page.receive(packet, result, v)

        if self._supervisor is not None:
            stream = self._supervisor.demux.streams[vid]
            self.overview_page.health.set_vehicle(
                vid, self._supervisor.vehicle_link_state(vid),
                counters=stream.counters, rssi=packet.rssi,
                success=stream.packet_success)

    def _on_source_state(self, state: SourceState) -> None:
        if state is SourceState.DISCONNECTED or state is SourceState.LINK_ERROR:
            self._telemetry_connected = False
        self.overview_page.health.set_source(state)
        self.com_page.set_source_state(state)
        self.header.set_source_state(state)

    def _on_receiver_state(self, state: ReceiverState) -> None:
        self.overview_page.health.set_receiver(state)

    def _on_vehicle_state(self, vid: VehicleID, state: VehicleLinkState) -> None:
        if state is VehicleLinkState.LOST:
            self._telemetry_connected = False
        self.pages[vid].set_link_state(state)
        # §10.8 — controls for a vehicle in LOST state are disabled rather
        # than queued to fire on reconnect.
        self.command_page.set_link_state(vid, state)
        self.overview_page.health.set_vehicle(vid, state)
        if self._supervisor is not None:
            self.overview_page.health.set_ground_side_warning(
                self._supervisor.demux.all_vehicles_silent())

    def _on_rejected(self, rejected: RejectedLine) -> None:
        # §4.5 — rejected packets are counted and visible, never
        # displayed as data. The count reaches the health strip via the
        # per-vehicle counters on the next accepted packet.
        self.com_page.on_rejected_line(rejected.reason.value, rejected.detail, rejected.raw)

    def _on_notice(self, text: str) -> None:
        self.statusBar().showMessage(text, 8000)

    # ── operator actions ─────────────────────────────────────────────────

    def reset_all_graphs(self) -> None:
        self.overview_page.clear_graphs()
        for p in self.pages.values():
            p.clear_graphs()
