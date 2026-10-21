"""
ground_station/pages/command.py
═══════════════════════════════
§10 — the Command dashboard. All actuation lives here and nowhere else
(§9.7 removed it from the vehicle dashboards entirely, including the
layout space it occupied).

This file is UI only. Every rule it enforces lives in commands.py and
actuators.py, which are Qt-free and unit-tested — the widgets ask those
modules whether something is allowed and render the answer. Nothing here
decides anything on its own, which is what makes §1.4 ("no command is
ever sent automatically") checkable: there is no timer in this file that
calls send().

Four §10 requirements are visible in the layout rather than buried:

  §10.4  sampling and transmission are two independent indicators, and
         HIBERNATE + ENABLED raises a persistent banner across the top.
  §10.1  every actuator shows commanded AND actual as separate values.
  §10.6  destructive controls are arm → fire, with the armed state and
         its countdown shown.
  §10.7  the command log shows sent → acknowledged → executed, with
         unacknowledged entries visibly different and a Resend button.
"""
from __future__ import annotations

import time
from typing import Dict, List, Optional

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QColor, QFont
from PyQt6.QtWidgets import (
    QFrame, QGridLayout, QHBoxLayout, QLabel, QMessageBox, QPushButton,
    QScrollArea, QSizePolicy, QVBoxLayout, QWidget,
)

try:
    from ..actuators import (
        ACTUATORS, ARM_TIMEOUT_S, Actuator, ActuatorSpec, ArmingCentre,
        CALIBRATIONS, CalibrationOutcome, CalibrationRun,
    )
    from ..backend_client import GroundStationBackendClient
    from ..commands import (
        BENCH_COMMANDS, COMMANDS, CommandCentre, CommandRecord, CommandStatus, Guard,
        ModeValue, VehicleModes, check_guard,
    )
    from ..models import (
        FlightState, GROUND_STATES, RocketSafetyState, VehicleID, VehicleLinkState,
    )
    from ..theme import ThemeManager
    from ..widgets.scroll_area import GroundStationScrollArea
except (ImportError, ValueError):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
    from ground_station.actuators import (
        ACTUATORS, ARM_TIMEOUT_S, Actuator, ActuatorSpec, ArmingCentre,
        CALIBRATIONS, CalibrationOutcome, CalibrationRun,
    )
    from ground_station.backend_client import GroundStationBackendClient
    from ground_station.commands import (
        BENCH_COMMANDS, COMMANDS, CommandCentre, CommandRecord, CommandStatus, Guard,
        ModeValue, VehicleModes, check_guard,
    )
    from ground_station.models import (
        FlightState, GROUND_STATES, RocketSafetyState, VehicleID, VehicleLinkState,
    )
    from ground_station.theme import ThemeManager
    from ground_station.widgets.scroll_area import GroundStationScrollArea


def _btn_css(accent: str) -> str:
    c = ThemeManager.C()
    dis_color = accent if accent == c.RED else "#8b949e"
    dis_border = "#a82020" if accent == c.RED else "#2b3644"
    return (f"QPushButton {{ background-color: {c.BG_INPUT}; color: {accent}; "
            f"border: 1px solid {accent}; border-radius: 4px; "
            f"padding: 6px 12px; font-weight: bold; }}"
            f"QPushButton:hover {{ background-color: {c.BG_CARD_HOVER}; }}"
            f"QPushButton:disabled {{ background-color: #121822; color: {dis_color}; "
            f"border: 1px solid {dis_border}; font-weight: bold; }}")


def _armed_btn_css() -> str:
    """§10.6 — visibly indicates the armed state with a filled red background.

    A red border alone reads almost identically to the normal border-accent
    style used by every safe button.  A filled background makes armed vs
    unarmed unmistakable at a glance, which matters for ACT_SEPARATE.
    """
    c = ThemeManager.C()
    return (f"QPushButton {{ background-color: {c.RED}; color: #ffffff; "
            f"border: 2px solid {c.RED}; border-radius: 4px; "
            f"padding: 6px 12px; font-weight: bold; }}"
            f"QPushButton:hover {{ background-color: {c.RED}; }}"
            f"QPushButton:disabled {{ background-color: {c.BG_INPUT}; "
            f"color: {c.TEXT_MUTED}; border-color: {c.BORDER}; }}")


def _card() -> QWidget:
    c = ThemeManager.C()
    w = QWidget()
    w.setStyleSheet(
        f"background-color: {c.BG_CARD}; border: 1px solid {c.BORDER}; "
        f"border-radius: 6px;")
    return w


def _heading(text: str) -> QLabel:
    c = ThemeManager.C()
    lbl = QLabel(text)
    lbl.setFont(QFont("monospace", 9, QFont.Weight.Bold))
    lbl.setStyleSheet(f"color: {c.CYAN};")
    return lbl


# ─────────────────────────────────────────────────────────────────────────────
#  §10.1 — one actuator
# ─────────────────────────────────────────────────────────────────────────────

class ActuatorPanel(QWidget):
    """Commanded vs actual, travel, and the move controls."""

    move_requested = pyqtSignal(str, float)        # actuator id, delta
    send_command_requested = pyqtSignal(str, str) # actuator id, command
    lock_requested = pyqtSignal(str)              # actuator id
    unlock_requested = pyqtSignal(str)            # actuator id
    angle_requested = pyqtSignal(str, int)        # actuator id, angle

    def __init__(self, spec: ActuatorSpec, actuator: Actuator,
                 arming: ArmingCentre, parent=None):
        super().__init__(parent)
        self.spec = spec
        self.actuator = actuator
        self.arming = arming
        self._enabled = False
        self._last_command_sent: Optional[str] = None
        self._angle_buttons: Dict[int, QPushButton] = {}
        self._build()

    def _build(self) -> None:
        c = ThemeManager.C()
        self.setStyleSheet(
            f"background-color: {c.BG_CARD}; border: 1px solid {c.BORDER}; "
            f"border-radius: 6px;")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 10, 12, 10)
        lay.setSpacing(6)

        top = QHBoxLayout()
        name = QLabel(self.spec.mechanism)
        name.setFont(QFont("monospace", 10, QFont.Weight.Bold))
        name.setStyleSheet(f"color: {c.TEXT};")
        top.addWidget(name)
        top.addStretch()
        chan = QLabel(f"{self.spec.driver} ch {self.spec.channel}")
        chan.setFont(QFont("monospace", 8))
        chan.setStyleSheet(f"color: {c.TEXT_DIM};")
        top.addWidget(chan)
        lay.addLayout(top)

        # §10.1 — commanded and actual are separate rows. Never one row
        # showing "position", which is how commanded ends up passing for
        # actual the moment a servo stalls.
        grid = QGridLayout()
        grid.setHorizontalSpacing(14)
        grid.setVerticalSpacing(2)
        self._commanded = self._value_row(grid, 0, "Commanded")
        self._actual = self._value_row(grid, 1, "Actual (from vehicle)")
        self._travel = self._value_row(grid, 2, "Remaining travel")
        self._last = self._value_row(grid, 3, "Last command")
        lay.addLayout(grid)

        self._mismatch = QLabel("")
        self._mismatch.setFont(QFont("monospace", 8, QFont.Weight.Bold))
        self._mismatch.setStyleSheet(f"color: {c.RED};")
        self._mismatch.setVisible(False)
        lay.addWidget(self._mismatch)

        row = QHBoxLayout()
        row.setSpacing(4)
        self.buttons: Dict[str, QPushButton] = {}
        ch = self.spec.channel

        # [ Lock ] and [ Unlock ]
        self._lock_btn = QPushButton("Lock")
        self._lock_btn.setFont(QFont("monospace", 8, QFont.Weight.Bold))
        self._lock_btn.setStyleSheet(_btn_css(c.AMBER))
        lock_angles = {"ACT_DOOR": 50, "ACT_DEPLOY": 80, "ACT_SEPARATE": 80}
        lock_deg = lock_angles.get(self.spec.id, 50)
        self._lock_btn.setToolTip(f"Lock {self.spec.mechanism.lower()} ({lock_deg}° safe position)")
        self._lock_btn.clicked.connect(self._on_lock_clicked)
        self.buttons[f"LOCK,{ch}"] = self._lock_btn
        row.addWidget(self._lock_btn)

        self._unlock_btn = QPushButton("Unlock")
        self._unlock_btn.setFont(QFont("monospace", 8, QFont.Weight.Bold))
        self._unlock_btn.setStyleSheet(_btn_css(c.CYAN))
        self._unlock_btn.setToolTip(f"Unlock {self.spec.mechanism.lower()} (0° safe position)")
        self._unlock_btn.clicked.connect(self._on_unlock_clicked)
        self.buttons[f"UNLOCK,{ch}"] = self._unlock_btn
        row.addWidget(self._unlock_btn)

        # [ 30° ] [ 60° ] [ 90° ]
        self._angle_buttons = {}
        for angle in (30, 60, 90):
            b = QPushButton(f"{angle}°")
            b.setFont(QFont("monospace", 8, QFont.Weight.Bold))
            b.setStyleSheet(_btn_css(c.TEXT))
            b.setToolTip(f"Command {self.spec.mechanism.lower()} to {angle}°")
            b.clicked.connect(lambda _c, a=angle: self._on_angle_clicked(a))
            self._angle_buttons[angle] = b
            self.buttons[f"SV,{ch},{angle}"] = b
            row.addWidget(b)

        row.addStretch()

        cmd_map = {"ACT_DOOR": "TEST1", "ACT_DEPLOY": "TEST2", "ACT_SEPARATE": "TEST3"}
        cmd_name = cmd_map.get(self.spec.id, f"TEST{ch+1}")

        self._send_btn = QPushButton("Send")
        self._send_btn.setFont(QFont("monospace", 8, QFont.Weight.Bold))
        self._send_btn.setStyleSheet(_btn_css(c.CYAN))
        self._send_btn.setToolTip(f"Transmit {cmd_name} over serial uplink")
        self._send_btn.clicked.connect(self._on_send_clicked)
        self.buttons[cmd_name] = self._send_btn
        row.addWidget(self._send_btn)

        lay.addLayout(row)

        self._note = QLabel("")
        self._note.setFont(QFont("monospace", 8))
        self._note.setStyleSheet(f"color: {c.AMBER};")
        self._note.setWordWrap(True)
        lay.addWidget(self._note)

    def _value_row(self, grid: QGridLayout, row: int, label: str) -> QLabel:
        c = ThemeManager.C()
        cap = QLabel(label)
        cap.setFont(QFont("monospace", 8))
        cap.setStyleSheet(f"color: {c.TEXT_DIM};")
        val = QLabel("—")
        val.setFont(QFont("monospace", 9, QFont.Weight.Bold))
        val.setStyleSheet(f"color: {c.TEXT};")
        grid.addWidget(cap, row, 0)
        grid.addWidget(val, row, 1)
        return val

    # ── actions ──────────────────────────────────────────────────────────

    def _on_lock_clicked(self) -> None:
        self.actuator.locked = True
        lock_angles = {"ACT_DOOR": 50.0, "ACT_DEPLOY": 80.0, "ACT_SEPARATE": 80.0}
        angle = lock_angles.get(self.spec.id, 0.0)
        self.actuator.note_commanded(angle)
        cmd_map = {"ACT_DOOR": "LOCK:DOOR", "ACT_DEPLOY": "LOCK:CANSAT", "ACT_SEPARATE": "LOCK:SEPARATION"}
        cmd = cmd_map.get(self.spec.id, f"LOCK,{self.spec.channel}")
        self.set_last_command(cmd)
        self.lock_requested.emit(self.spec.id)

    def _on_unlock_clicked(self) -> None:
        self.actuator.locked = False
        self.actuator.note_commanded(0.0)
        cmd_map = {"ACT_DOOR": "UNLOCK:DOOR", "ACT_DEPLOY": "UNLOCK:CANSAT", "ACT_SEPARATE": "UNLOCK:SEPARATION"}
        cmd = cmd_map.get(self.spec.id, f"UNLOCK,{self.spec.channel}")
        self.set_last_command(cmd)
        self._note.setText("Unlocked to 0° safe position.")
        self.unlock_requested.emit(self.spec.id)
        self.refresh()

    def _on_angle_clicked(self, angle: int) -> None:
        self.actuator.locked = False
        self.actuator.note_commanded(float(angle))
        cmd_prefix = {"ACT_DOOR": "DOOR", "ACT_DEPLOY": "CANSAT", "ACT_SEPARATE": "SEPARATION"}.get(self.spec.id, "DOOR")
        cmd = f"{cmd_prefix}:{angle}"
        self.set_last_command(cmd)
        self.angle_requested.emit(self.spec.id, angle)

    def _on_send_clicked(self) -> None:
        cmd_prefix = {"ACT_DOOR": "DOOR", "ACT_DEPLOY": "CANSAT", "ACT_SEPARATE": "SEPARATION"}.get(self.spec.id, "DOOR")
        if self.actuator.commanded is not None and int(self.actuator.commanded) in (30, 60, 90):
            cmd = f"{cmd_prefix}:{int(self.actuator.commanded)}"
        elif self._last_command_sent and ":" in self._last_command_sent:
            cmd = self._last_command_sent
        else:
            cmd_map = {"ACT_DOOR": "TEST1", "ACT_DEPLOY": "TEST2", "ACT_SEPARATE": "TEST3"}
            cmd = cmd_map.get(self.spec.id, "TEST1")
        self.set_last_command(cmd)
        self.send_command_requested.emit(self.spec.id, cmd)

    def set_last_command(self, command: str) -> None:
        self._last_command_sent = command
        self.actuator.last_command_at = time.monotonic()
        self.refresh()

    # ── refresh ──────────────────────────────────────────────────────────

    def set_enabled_by_guard(self, enabled: bool, reason: str = "", rocket_selected: bool = True) -> None:
        self._enabled = enabled
        self._rocket_selected = rocket_selected
        if not enabled and reason:
            self._note.setText(reason)
        self.refresh()

    def refresh(self) -> None:
        c = ThemeManager.C()
        a = self.actuator

        self._commanded.setText(
            "—" if a.commanded is None else f"{a.commanded:.0f}°" if a.commanded.is_integer() else f"{a.commanded:.1f}°")
        # §10.1 — "Never display commanded position as if it were actual."
        # An em dash here means the vehicle has not reported a position.
        # Preserve distinction between actual measurement, firmware estimate, and not reported.
        if a.position is None:
            self._actual.setText("not reported")
        else:
            deg_val = f"{a.position:.0f}°" if a.position.is_integer() else f"{a.position:.1f}°"
            if getattr(a, "is_estimate", False):
                self._actual.setText(f"{deg_val} (est)")
            else:
                self._actual.setText(deg_val)

        rem = getattr(a, "remaining_deg", None)
        if rem is not None:
            self._travel.setText(f"{rem:.0f}° to target" if rem > 0 else "0° (at target)")
        else:
            down, up = a.remaining_travel
            self._travel.setText(
                "—" if down is None else f"{down:.0f}° down   {up:.0f}° up")
        if self._last_command_sent:
            self._last.setText(self._last_command_sent)
        elif a.last_command_at is not None:
            self._last.setText(time.strftime("%H:%M:%S", time.localtime(a.last_command_at)))
        else:
            self._last.setText("—")

        agree = a.commanded_matches_actual
        self._mismatch.setVisible(agree is False)
        if agree is False:
            self._mismatch.setText(
                "COMMANDED AND ACTUAL DISAGREE — mechanism may be jammed")

        rocket_selected = getattr(self, '_rocket_selected', True)
        self._lock_btn.setEnabled(rocket_selected)
        self._unlock_btn.setEnabled(rocket_selected)
        for b in self._angle_buttons.values():
            b.setEnabled(rocket_selected)
        self._send_btn.setEnabled(rocket_selected)


# ─────────────────────────────────────────────────────────────────────────────
#  §10.4 — sampling vs transmission
# ─────────────────────────────────────────────────────────────────────────────

class ModePanel(QWidget):
    """Telemetry control panel matching user layout with 4 mode boxes."""

    command_requested = pyqtSignal(str)

    def __init__(self, modes: VehicleModes, parent=None):
        super().__init__(parent)
        self.modes = modes
        c = ThemeManager.C()
        self.setStyleSheet(
            f"background-color: {c.BG_CARD}; border: 1px solid {c.BORDER}; "
            f"border-radius: 6px;")
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(12, 10, 12, 10)
        self.layout.setSpacing(8)
        self.layout.addWidget(_heading("Telemetry"))

        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)
        self.buttons: Dict[str, QPushButton] = {}

        # Row 0: [ Live ] [ Hibernate ]
        for i, (cmd, label) in enumerate([("LIVE", "Live"), ("HIBERNATE", "Hibernate")]):
            b = QPushButton(label)
            b.setFont(QFont("monospace", 9, QFont.Weight.Bold))
            b.setMinimumHeight(30)
            b.setStyleSheet(ThemeManager.status_button_css("red"))
            b.clicked.connect(lambda _c, x=cmd: self.command_requested.emit(x))
            grid.addWidget(b, 0, i)
            self.buttons[cmd] = b

        # Row 1: [ Enable ] [ Disable ]
        for i, (cmd, label) in enumerate([("ENABLE", "Enable"), ("DISABLE", "Disable")]):
            b = QPushButton(label)
            b.setFont(QFont("monospace", 9, QFont.Weight.Bold))
            b.setMinimumHeight(30)
            b.setStyleSheet(ThemeManager.status_button_css("red"))
            b.clicked.connect(lambda _c, x=cmd: self.command_requested.emit(x))
            grid.addWidget(b, 1, i)
            self.buttons[cmd] = b

        self.layout.addLayout(grid)

    def refresh(self) -> None:
        pass
# ─────────────────────────────────────────────────────────────────────────────
#  Sequence & Bench Controls
# ─────────────────────────────────────────────────────────────────────────────

class SequenceControlPanel(QWidget):
    """Real-time bench test & sequence controls for multi-servo hardware."""

    command_requested = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._mode = "IDLE"
        self._step = 0
        self._total_steps = 3
        self._step_text = "0 / 3"
        self._last_cmd = "—"
        self._status = "Ready"
        self._buttons: List[QPushButton] = []
        self.buttons: Dict[str, QPushButton] = {}
        self._enabled = True
        self._build()

    def _build(self) -> None:
        c = ThemeManager.C()
        self.setStyleSheet(
            f"background-color: {c.BG_CARD}; border: 1px solid {c.BORDER}; "
            f"border-radius: 6px;")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 10, 12, 10)
        lay.setSpacing(8)
        lay.addWidget(_heading("Sequence Controls"))

        grid = QGridLayout()
        grid.setHorizontalSpacing(14)
        grid.setVerticalSpacing(2)

        self._mode_val = self._value_row(grid, 0, "Sequence mode")
        self._step_val = self._value_row(grid, 1, "Current step")
        self._last_val = self._value_row(grid, 2, "Last command")
        self._status_val = self._value_row(grid, 3, "Status")
        lay.addLayout(grid)

        # Row 1: LOCK ALL, RESET
        row1 = QHBoxLayout()
        row1.setSpacing(6)

        self._btn_lock_all = QPushButton("LOCK ALL")
        self._btn_lock_all.setFont(QFont("monospace", 8, QFont.Weight.Bold))
        self._btn_lock_all.setStyleSheet(ThemeManager.status_button_css("neutral"))
        self._btn_lock_all.setToolTip("Transmit LOCKALL: Lock all servos to safe position (0°)")
        self._btn_lock_all.clicked.connect(self._on_lock_all_clicked)
        self._buttons.append(self._btn_lock_all)
        self.buttons["LOCKALL"] = self._btn_lock_all
        row1.addWidget(self._btn_lock_all)

        self._btn_reset = QPushButton("RESET")
        self._btn_reset.setFont(QFont("monospace", 8, QFont.Weight.Bold))
        self._btn_reset.setStyleSheet(ThemeManager.status_button_css("neutral"))
        self._btn_reset.setToolTip("Transmit SEQRST: Reset and lock all servos to 0°")
        self._btn_reset.clicked.connect(self._on_reset_clicked)
        self._buttons.append(self._btn_reset)
        self.buttons["SEQRST"] = self._btn_reset
        row1.addWidget(self._btn_reset)

        lay.addLayout(row1)

        # Row 2: MANUAL SEQUENCE, NEXT SEQUENCE STEP, FULL SEQUENCE, SEPARATION
        row2 = QHBoxLayout()
        row2.setSpacing(6)

        self._btn_manual_seq = QPushButton("MANUAL SEQUENCE")
        self._btn_manual_seq.setFont(QFont("monospace", 8, QFont.Weight.Bold))
        self._btn_manual_seq.setStyleSheet(ThemeManager.status_button_css("neutral"))
        self._btn_manual_seq.setToolTip("Transmit MSEQ: Start manual sequence")
        self._btn_manual_seq.clicked.connect(self._on_manual_seq_clicked)
        self._buttons.append(self._btn_manual_seq)
        self.buttons["MSEQ"] = self._btn_manual_seq
        row2.addWidget(self._btn_manual_seq)

        self._btn_next_step = QPushButton("NEXT SEQUENCE STEP")
        self._btn_next_step.setFont(QFont("monospace", 8, QFont.Weight.Bold))
        self._btn_next_step.setStyleSheet(ThemeManager.status_button_css("neutral"))
        self._btn_next_step.setToolTip("Transmit NSTEP: Advance manual sequence to next step (needs Arm)")
        self._btn_next_step.clicked.connect(self._on_next_step_clicked)
        self._buttons.append(self._btn_next_step)
        self.buttons["NSTEP"] = self._btn_next_step
        row2.addWidget(self._btn_next_step)

        self._btn_full_seq = QPushButton("FULL SEQUENCE")
        self._btn_full_seq.setFont(QFont("monospace", 8, QFont.Weight.Bold))
        self._btn_full_seq.setStyleSheet(ThemeManager.status_button_css("neutral"))
        self._btn_full_seq.setToolTip("Transmit FSEQ: Run full automatic 3-servo sequence (needs Arm + confirmation)")
        self._btn_full_seq.clicked.connect(self._on_full_seq_clicked)
        self._buttons.append(self._btn_full_seq)
        self.buttons["FSEQ"] = self._btn_full_seq
        row2.addWidget(self._btn_full_seq)

        self._btn_sep = QPushButton("SEPARATION")
        self._btn_sep.setFont(QFont("monospace", 8, QFont.Weight.Bold))
        self._btn_sep.setStyleSheet(ThemeManager.status_button_css("neutral"))
        self._btn_sep.setToolTip("Transmit SEP: Fire vehicle separation (needs Arm + confirmation)")
        self._btn_sep.clicked.connect(self._on_sep_clicked)
        self._buttons.append(self._btn_sep)
        self.buttons["SEP"] = self._btn_sep
        row2.addWidget(self._btn_sep)

        lay.addLayout(row2)
        self.refresh()

    def _value_row(self, grid: QGridLayout, row: int, label: str) -> QLabel:
        c = ThemeManager.C()
        cap = QLabel(label)
        cap.setFont(QFont("monospace", 8))
        cap.setStyleSheet(f"color: {c.TEXT_DIM};")
        val = QLabel("—")
        val.setFont(QFont("monospace", 9, QFont.Weight.Bold))
        val.setStyleSheet(f"color: {c.TEXT};")
        grid.addWidget(cap, row, 0)
        grid.addWidget(val, row, 1)
        return val

    def _on_btn_clicked(self, command: str) -> None:
        self.command_requested.emit(command)

    def _on_lock_all_clicked(self) -> None:
        self.command_requested.emit("LOCKALL")

    def _on_reset_clicked(self) -> None:
        self.command_requested.emit("SEQRST")

    def _on_manual_seq_clicked(self) -> None:
        self.command_requested.emit("MSEQ")

    def _on_next_step_clicked(self) -> None:
        self.command_requested.emit("NSTEP")

    def _on_full_seq_clicked(self) -> None:
        self.command_requested.emit("FSEQ")

    def _on_sep_clicked(self) -> None:
        self.command_requested.emit("SEP")

    def note_command_sent(self, command: str) -> None:
        self._last_cmd = command
        cmd_u = command.upper()
        if cmd_u in ("TEST1",):
            self._mode = "INDIVIDUAL TEST"
            self._step_text = "Door Test"
            self._status = "Sent: TEST1 (Door Test)"
        elif cmd_u in ("TEST2",):
            self._mode = "INDIVIDUAL TEST"
            self._step_text = "CanSat Test"
            self._status = "Sent: TEST2 (CanSat Test)"
        elif cmd_u in ("TEST3",):
            self._mode = "INDIVIDUAL TEST"
            self._step_text = "Separation Test"
            self._status = "Sent: TEST3 (Separation Test)"
        elif cmd_u in ("LOCKALL", "LOCK:ALL", "TEST4"):
            self._mode = "LOCKED"
            self._step = 0
            self._step_text = "0 / 3 (All Locked)"
            self._status = "Sent: LOCKALL (Lock All)"
        elif cmd_u in ("MSEQ", "TEST5"):
            self._mode = "MANUAL"
            self._step = 1
            self._step_text = "1 / 3 (Step 1: Door)"
            self._status = "Sent: MSEQ (Manual Step 1: Door)"
        elif cmd_u in ("NSTEP", "NEXT"):
            if self._step == 1:
                self._step = 2
                self._step_text = "2 / 3 (Step 2: CanSat)"
            elif self._step == 2:
                self._step = 3
                self._step_text = "3 / 3 (Step 3: Separation)"
            elif self._step >= 3:
                self._step_text = "3 / 3 (Complete)"
            else:
                self._step = 1
                self._step_text = "1 / 3 (Step 1: Door)"
            self._status = f"Sent: NSTEP ({self._step_text})"
        elif cmd_u in ("FSEQ", "TEST6"):
            self._mode = "FULL SEQUENCE"
            self._step = 3
            self._step_text = "3 / 3 (Full Auto Run)"
            self._status = "Sent: FSEQ (Full Sequence)"
        elif cmd_u in ("SEQRST", "RESET"):
            self._mode = "RESET / IDLE"
            self._step = 0
            self._step_text = "0 / 3 (Safe / Reset)"
            self._status = "Sent: SEQRST (Sequence Reset)"
        self.refresh()

    def set_enabled(self, enabled: bool) -> None:
        self._enabled = enabled
        for b in self._buttons:
            b.setEnabled(enabled)

    def refresh(self) -> None:
        c = ThemeManager.C()
        self._mode_val.setText(self._mode)
        step_display = getattr(self, "_step_text", "0 / 3")
        self._step_val.setText(step_display)
        self._last_val.setText(self._last_cmd)
        self._status_val.setText(self._status)
        colour = c.GREEN if ("Sent" in self._status or self._status == "Ready") else c.AMBER
        self._status_val.setStyleSheet(f"color: {colour};")


# ─────────────────────────────────────────────────────────────────────────────
#  §10.2 — calibration
# ─────────────────────────────────────────────────────────────────────────────

class CalibrationPanel(QWidget):
    command_requested = pyqtSignal(str)

    def __init__(self, run: CalibrationRun, parent=None):
        super().__init__(parent)
        self.run = run
        self._rows: Dict[str, QLabel] = {}
        c = ThemeManager.C()
        self.setStyleSheet(
            f"background-color: {c.BG_CARD}; border: 1px solid {c.BORDER}; "
            f"border-radius: 6px;")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 10, 12, 10)
        lay.setSpacing(6)
        lay.addWidget(_heading("Calibration"))

        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        self._buttons: List[QPushButton] = []
        self.buttons: Dict[str, QPushButton] = {}
        active_calibrations = [s for s in CALIBRATIONS if s.key in ("ALTITUDE", "IMU")]
        key_map = {"ALTITUDE": "CALA", "IMU": "CALI"}
        for i, sensor in enumerate(active_calibrations):
            name = QLabel(sensor.label)
            name.setFont(QFont("monospace", 9))
            name.setStyleSheet(f"color: {c.TEXT};")
            name.setToolTip(sensor.target)
            grid.addWidget(name, i, 0)

            result = QLabel("not run")
            result.setFont(QFont("monospace", 8, QFont.Weight.Bold))
            result.setStyleSheet(f"color: {c.TEXT_MUTED};")
            self._rows[sensor.key] = result
            grid.addWidget(result, i, 1)

            b = QPushButton("Calibrate")
            b.setFont(QFont("monospace", 8, QFont.Weight.Bold))
            b.setStyleSheet(ThemeManager.status_button_css("neutral"))
            cmd_key = key_map.get(sensor.key, f"CALIBRATE_{sensor.key}")
            b.clicked.connect(
                lambda _c, k=cmd_key:
                self.command_requested.emit(k))
            self._buttons.append(b)
            self.buttons[cmd_key] = b
            grid.addWidget(b, i, 2)
        lay.addLayout(grid)

        self._all = QPushButton("Calibrate All")
        self._all.setFont(QFont("monospace", 9, QFont.Weight.Bold))
        self._all.setStyleSheet(ThemeManager.status_button_css("neutral"))
        self._all.clicked.connect(
            lambda: self.command_requested.emit("CAL"))
        self._buttons.append(self._all)
        self.buttons["CAL"] = self._all
        lay.addWidget(self._all)

    def set_enabled_by_guard(self, enabled: bool) -> None:
        for b in self._buttons:
            b.setEnabled(enabled)

    def refresh(self) -> None:
        c = ThemeManager.C()
        # §10.2 — per-sensor pass/fail, "not one aggregate result". An
        # aggregate hides which sensor failed, which is the only part the
        # operator can act on.
        for key, label in self._rows.items():
            outcome = self.run.results.get(key, CalibrationOutcome.NOT_RUN)
            colour = {
                CalibrationOutcome.PASS: c.GREEN,
                CalibrationOutcome.FAIL: c.RED,
                CalibrationOutcome.RUNNING: c.AMBER,
            }.get(outcome, c.TEXT_MUTED)
            label.setText(outcome.value)
            label.setStyleSheet(f"color: {colour};")
            label.setToolTip(self.run.messages.get(key, ""))


# ─────────────────────────────────────────────────────────────────────────────
#  Hardware Diagnostics & RF Control
# ─────────────────────────────────────────────────────────────────────────────

class DiagnosticsPanel(QWidget):
    """Real-time diagnostics and link control panel (PG, ST, LK, BITE, FPL, BZ, TX, GR, LR)."""

    command_requested = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        c = ThemeManager.C()
        self.setStyleSheet(
            f"background-color: {c.BG_CARD}; border: 1px solid {c.BORDER}; "
            f"border-radius: 6px;")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 10, 12, 10)
        lay.setSpacing(8)
        lay.addWidget(_heading("Diagnostics & RF Control"))

        self.buttons: Dict[str, QPushButton] = {}

        # Row 1: Diagnostics [ Ping ] [ Status ] [ Link Check ] [ BITE ] [ Force Pre-Launch ]
        row1 = QHBoxLayout()
        row1.setSpacing(6)

        diag_cmds = [
            ("PG", "Ping", "Ping rocket to establish two-way communication"),
            ("ST", "Status", "Query vehicle system status"),
            ("LK", "Link Check", "Check RF link quality and signal parameters"),
            ("BITE", "BITE", "Run Built-In Test Equipment diagnostics"),
            ("FPL", "Force Pre-Launch", "Force flight computer into PRE-LAUNCH state (needs Arm + confirm)"),
        ]
        for cmd, label, tip in diag_cmds:
            btn = QPushButton(label)
            btn.setFont(QFont("monospace", 8, QFont.Weight.Bold))
            btn.setStyleSheet(ThemeManager.status_button_css("neutral"))
            btn.setToolTip(tip)
            btn.clicked.connect(lambda _c, x=cmd: self.command_requested.emit(x))
            self.buttons[cmd] = btn
            row1.addWidget(btn)

        lay.addLayout(row1)

        # Row 2: RF & Peripheral Control [ Buzzer ON ] [ Buzzer OFF ] [ Rocket TX ON ] [ Rocket TX OFF ] [ GNSS Reset ] [ LoRa Reinit ]
        row2 = QHBoxLayout()
        row2.setSpacing(6)

        rf_cmds = [
            ("BZ,1", "Buzzer ON", "Turn rocket audio beacon / buzzer ON"),
            ("BZ,0", "Buzzer OFF", "Turn rocket audio beacon / buzzer OFF"),
            ("TX,1", "Rocket TX ON", "Enable rocket LoRa transmission"),
            ("TX,0", "Rocket TX OFF", "Silence rocket transmitter (receiver stays on)"),
            ("GR", "GNSS Reset", "Reset / reinitialise rocket GNSS module"),
            ("LR", "LoRa Reinit", "Reinitialise rocket LoRa radio transceiver"),
        ]
        for cmd, label, tip in rf_cmds:
            btn = QPushButton(label)
            btn.setFont(QFont("monospace", 8, QFont.Weight.Bold))
            btn.setStyleSheet(ThemeManager.status_button_css("neutral"))
            btn.setToolTip(tip)
            btn.clicked.connect(lambda _c, x=cmd: self.command_requested.emit(x))
            self.buttons[cmd] = btn
            row2.addWidget(btn)

        lay.addLayout(row2)

    def set_enabled_by_guard(self, enabled: bool) -> None:
        for b in self.buttons.values():
            b.setEnabled(enabled)

    def refresh(self) -> None:
        pass


# ─────────────────────────────────────────────────────────────────────────────
#  §10.7 — command log
# ─────────────────────────────────────────────────────────────────────────────

class CommandLogPanel(QWidget):
    resend_requested = pyqtSignal(int)

    def __init__(self, centre: CommandCentre, parent=None):
        super().__init__(parent)
        self.centre = centre
        c = ThemeManager.C()
        self.setStyleSheet(
            f"background-color: {c.BG_CARD}; border: 1px solid {c.BORDER}; "
            f"border-radius: 6px;")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 10, 12, 10)
        lay.setSpacing(6)
        lay.addWidget(_heading("Command Log"))

        self._scroll = QScrollArea()
        self._scroll.setWidgetResizable(True)
        self._scroll.setMinimumHeight(220)
        self._body = QWidget()
        self._body_layout = QVBoxLayout(self._body)
        self._body_layout.setContentsMargins(0, 0, 0, 0)
        self._body_layout.setSpacing(3)
        self._body_layout.addStretch()
        self._scroll.setWidget(self._body)
        lay.addWidget(self._scroll)

        self._rendered: Dict[int, QWidget] = {}

    def refresh(self) -> None:
        for record in self.centre.records:
            if record.sequence not in self._rendered:
                w = self._row(record)
                self._rendered[record.sequence] = w
                self._body_layout.insertWidget(0, w)
            self._update_row(self._rendered[record.sequence], record)

    def _row(self, record: CommandRecord) -> QWidget:
        w = QWidget()
        row = QHBoxLayout(w)
        row.setContentsMargins(4, 2, 4, 2)
        row.setSpacing(8)

        w._time = QLabel("")
        w._time.setFont(QFont("monospace", 8))
        w._text = QLabel("")
        w._text.setFont(QFont("monospace", 8))
        w._status = QLabel("")
        w._status.setFont(QFont("monospace", 8, QFont.Weight.Bold))
        w._resend = QPushButton("Resend")
        w._resend.setFont(QFont("monospace", 8))
        w._resend.setStyleSheet(_btn_css(ThemeManager.C().AMBER))
        w._resend.clicked.connect(
            lambda _c, s=record.sequence: self.resend_requested.emit(s))

        row.addWidget(w._time)
        row.addWidget(w._text, stretch=1)
        row.addWidget(w._status)
        row.addWidget(w._resend)
        return w

    def _update_row(self, w: QWidget, record: CommandRecord) -> None:
        c = ThemeManager.C()
        w._time.setText(time.strftime("%H:%M:%S",
                                      time.localtime(record.sent_at)))
        w._time.setStyleSheet(f"color: {c.TEXT_MUTED};")
        w._text.setText(record.command if record.command in BENCH_COMMANDS else f"{record.target.value}  {record.command}")
        w._text.setStyleSheet(f"color: {c.TEXT};")

        # §10.7 — "An unacknowledged command must look visibly different
        # from a successful one." Colour plus the persistent Resend
        # button, not colour alone.
        colour = {
            CommandStatus.EXECUTED: c.GREEN,
            CommandStatus.ACKNOWLEDGED: c.CYAN,
            CommandStatus.SENT: c.AMBER,
            CommandStatus.TIMED_OUT: c.RED,
            CommandStatus.REJECTED: c.RED,
        }.get(record.status, c.TEXT_DIM)
        w._status.setText(record.status.value)
        w._status.setStyleSheet(f"color: {colour};")
        # §10.7 — retry is operator-initiated only. The button is the only
        # path to a resend; nothing in this file calls resend() on a timer.
        w._resend.setVisible(record.unacknowledged)


# ─────────────────────────────────────────────────────────────────────────────
#  The page
# ─────────────────────────────────────────────────────────────────────────────

class CommandPage(QWidget):
    """§10 — the whole dashboard for one vehicle at a time."""

    notice = pyqtSignal(str)

    @staticmethod
    def _command_popup_text(command: str) -> str:
        names = {
            "DOOR:30": "Door 30° command sent",
            "DOOR:60": "Door 60° command sent",
            "DOOR:90": "Door 90° command sent",
            "LOCK:DOOR": "Door locked (0°)",
            "CANSAT:30": "CanSat 30° command sent",
            "CANSAT:60": "CanSat 60° command sent",
            "CANSAT:90": "CanSat 90° command sent",
            "LOCK:CANSAT": "CanSat locked (0°)",
            "SEPARATION:30": "Separation 30° command sent",
            "SEPARATION:60": "Separation 60° command sent",
            "SEPARATION:90": "Separation 90° command sent",
            "LOCK:SEPARATION": "Separation locked (0°)",
            "LOCK:ALL": "Lock All command sent",
            "LOCKALL": "Lock All command sent",
            "UNLOCK:DOOR": "Door unlocked (0°)",
            "UNLOCK:CANSAT": "CanSat unlocked (0°)",
            "UNLOCK:SEPARATION": "Separation unlocked (0°)",
            "TEST1": "Door test command sent",
            "TEST2": "CanSat test command sent",
            "TEST3": "Separation test command sent",
            "TEST4": "Lock All command sent",
            "RESET": "Reset command sent",
            "SEQRST": "Sequence Reset command sent",
            "TEST5": "Manual sequence command sent",
            "MSEQ": "Manual sequence command sent",
            "NEXT": "Next sequence step command sent",
            "NSTEP": "Next sequence step command sent",
            "TEST6": "Full sequence command sent",
            "FSEQ": "Full sequence command sent",
            "ENABLE_TELEMETRY": "Enable telemetry command sent",
            "DISABLE_TELEMETRY": "Disable telemetry command sent",
            "SET_MODE_LIVE": "Live mode command sent",
            "SET_MODE_HIBERNATE": "Hibernate command sent",
            "FREQ:ROCKET:ORIGINAL": "Rocket frequency set to Original",
            "FREQ:ROCKET:BACKUP": "Rocket frequency set to Backup",
            "FREQ:CANSAT:ORIGINAL": "CanSat frequency set to Original",
            "FREQ:CANSAT:BACKUP": "CanSat frequency set to Backup",
            "REFRESH_PROCESSOR": "Refresh processor command sent",
            "LOCK,0": "Door lock command sent (50°)",
            "LOCK,1": "CanSat lock command sent (80°)",
            "LOCK,2": "Separation lock command sent (80°)",
            "UNLOCK,0": "Door unlock command sent (0°)",
            "UNLOCK,1": "CanSat unlock command sent (0°)",
            "UNLOCK,2": "Separation unlock command sent (0°)",
            "SV,0,30": "Door 30° command sent",
            "SV,0,60": "Door 60° command sent",
            "SV,0,90": "Door 90° command sent",
            "SV,1,30": "CanSat 30° command sent",
            "SV,1,60": "CanSat 60° command sent",
            "SV,1,90": "CanSat 90° command sent",
            "SV,2,30": "Separation 30° command sent",
            "SV,2,60": "Separation 60° command sent",
            "SV,2,90": "Separation 90° command sent",
            "FQ,0": "Rocket Original frequency (866.50 MHz) command sent",
            "FQ,1": "Rocket Backup frequency (865.25 MHz) command sent",
            "RF": "Refresh processor (RF) command sent",
            "LIVE": "Live mode (SD) command sent",
            "HIBERNATE": "Hibernate mode (XD) command sent",
            "ENABLE": "LoRa Enable command sent",
            "DISABLE": "LoRa Disable command sent",
            "PG": "Ping (PG) command sent",
            "ST": "Status query (ST) command sent",
            "LK": "Link check (LK) command sent",
            "BITE": "BITE diagnostic command sent",
            "FPL": "Force Pre-Launch (FPL) command sent",
            "BZ,1": "Buzzer ON command sent",
            "BZ,0": "Buzzer OFF command sent",
            "TX,1": "Rocket TX ON command sent",
            "TX,0": "Rocket TX OFF command sent",
            "GR": "GNSS Reset command sent",
            "LR": "LoRa Reinit command sent",
            "CALA": "Altitude calibration command sent",
            "CALI": "IMU calibration command sent",
            "CAL": "Calibrate All sensors command sent",
            "SEP": "Separation (SEP) command sent",
        }
        return names.get(command, f"{command} command sent")

    def __init__(self, centre: CommandCentre, simulator_mode: bool = False,
                 parent=None):
        super().__init__(parent)
        self.centre = centre
        self.simulator_mode = simulator_mode
        self.vehicle = VehicleID.ROCKET

        self.arming = ArmingCentre()
        self.actuators: Dict[str, Actuator] = {
            spec.id: Actuator(spec) for spec in ACTUATORS
        }
        # §10.4 modes live on the CommandCentre, which is what receives
        # the acknowledgments. A second copy here would show a mode the
        # vehicle never confirmed — exactly the optimistic flip §10.4
        # forbids.
        self.calibration: Dict[VehicleID, CalibrationRun] = {
            v: CalibrationRun() for v in VehicleID
        }
        self._states: Dict[VehicleID, Optional[FlightState]] = {
            v: None for v in VehicleID
        }
        self._links: Dict[VehicleID, VehicleLinkState] = {
            v: VehicleLinkState.LOST for v in VehicleID
        }
        self._usb_direct = False

        self._build()

        # Refresh only — this timer never sends anything (§1.4). It exists
        # so the arming countdown and ack ages stay current on screen.
        self._tick = QTimer(self)
        self._tick.setInterval(500)
        self._tick.timeout.connect(self.refresh)
        self._tick.start()

    def _build(self) -> None:
        c = ThemeManager.C()
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        # §10.4 — persistent, unmissable banner whenever HIBERNATE +
        # ENABLED holds: packets keep arriving so the link looks healthy,
        # but the values are not live measurements.
        self._banner = QLabel("")
        self._banner.setFont(QFont("monospace", 10, QFont.Weight.Bold))
        self._banner.setStyleSheet(
            f"background-color: {c.RED}; color: #FFFFFF; padding: 8px;")
        self._banner.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._banner.setVisible(False)
        outer.addWidget(self._banner)

        scroll = GroundStationScrollArea()
        body = QWidget()
        lay = QVBoxLayout(body)
        lay.setContentsMargins(12, 12, 12, 12)
        lay.setSpacing(10)

        # Operator sequence banner (§6)
        self._op_seq_banner = QFrame()
        self._op_seq_banner.setStyleSheet(
            f"QFrame {{ background-color: {c.BG_PANEL}; border: 1px solid {c.BORDER}; border-radius: 4px; }}"
        )
        op_lay = QHBoxLayout(self._op_seq_banner)
        op_lay.setContentsMargins(10, 6, 10, 6)
        op_text = QLabel(
            "OPERATOR SEQUENCE:  1. Plug in ground Teensy → header CONNECTED (green)   "
            "2. Enable (LoRa ON)   3. Ping (wake Rocket)   4. Live (telemetry flows)"
        )
        op_text.setFont(QFont("monospace", 8, QFont.Weight.Bold))
        op_text.setStyleSheet(f"color: {c.CYAN}; border: none;")
        op_lay.addWidget(op_text)
        lay.addWidget(self._op_seq_banner)

        lay.addLayout(self._vehicle_selector())

        # Telemetry panel (containing 4 mode boxes, refresh processor, and frequency selectors)
        self.mode_panel = ModePanel(self.centre.modes[self.vehicle])
        self.mode_panel.command_requested.connect(self._send_mode_command)
        self._setup_telemetry_commands(self.mode_panel)
        lay.addWidget(self.mode_panel)

        act_head = _heading("Actuators")
        lay.addWidget(act_head)
        # Actuators note removed per spec

        self.actuator_panels: Dict[str, ActuatorPanel] = {}
        for spec in ACTUATORS:
            p = ActuatorPanel(spec, self.actuators[spec.id], self.arming)
            p.move_requested.connect(self._send_move)
            p.send_command_requested.connect(self._send_actuator_command)
            p.lock_requested.connect(self._on_actuator_lock)
            p.unlock_requested.connect(self._on_actuator_unlock)
            p.angle_requested.connect(self._on_actuator_angle)
            self.actuator_panels[spec.id] = p
            lay.addWidget(p)

        self.sequence_panel = SequenceControlPanel()
        self.sequence_panel.command_requested.connect(self._send_sequence_command)
        lay.addWidget(self.sequence_panel)

        self.calibration_panel = CalibrationPanel(self.calibration[self.vehicle])
        self.calibration_panel.command_requested.connect(self._send_guarded)
        lay.addWidget(self.calibration_panel)

        self.diagnostics_panel = DiagnosticsPanel()
        self.diagnostics_panel.command_requested.connect(self._send_diagnostics_command)
        lay.addWidget(self.diagnostics_panel)

        self.log_panel = CommandLogPanel(self.centre)
        self.log_panel.resend_requested.connect(self.centre.resend)
        lay.addWidget(self.log_panel)
        scroll.attach_child_wheel_passthrough(self.log_panel._scroll)

        lay.addStretch()
        scroll.setWidget(body)
        outer.addWidget(scroll)

    def _vehicle_selector(self) -> QHBoxLayout:
        c = ThemeManager.C()
        row = QHBoxLayout()
        row.addWidget(_heading("Target vehicle"))
        self._vehicle_buttons: Dict[VehicleID, QPushButton] = {}
        for vid in VehicleID:
            b = QPushButton(vid.value)
            b.setCheckable(True)
            b.setFont(QFont("monospace", 9, QFont.Weight.Bold))
            b.setStyleSheet(_btn_css(c.CYAN))
            b.clicked.connect(lambda _c, v=vid: self._select_vehicle(v))
            self._vehicle_buttons[vid] = b
            row.addWidget(b)
        self._vehicle_buttons[VehicleID.ROCKET].setChecked(True)

        # Rocket safety state control (ARM / DISARM)
        self._rocket_arm_btn = QPushButton("Arm")
        self._rocket_arm_btn.setFont(QFont("monospace", 9, QFont.Weight.Bold))
        self._rocket_arm_btn.setStyleSheet(_btn_css(c.RED))
        self._rocket_arm_btn.setToolTip("Set Rocket safety state: ARMED (flight-ready)")
        self._rocket_arm_btn.clicked.connect(self._toggle_rocket_arm)
        row.addWidget(self._rocket_arm_btn)

        self._rocket_safety_label = QLabel("UNARMED")
        self._rocket_safety_label.setFont(QFont("monospace", 9, QFont.Weight.Bold))
        self._rocket_safety_label.setStyleSheet(f"color: {c.GREEN}; padding: 0 4px;")
        row.addWidget(self._rocket_safety_label)

        row.addStretch()

        self._guard_note = QLabel("")
        self._guard_note.setFont(QFont("monospace", 8))
        self._guard_note.setStyleSheet(f"color: {c.AMBER};")
        row.addWidget(self._guard_note)
        return row

    @property
    def rocket_safety_state(self) -> RocketSafetyState:
        return self.centre.rocket_safety_state

    def arm_rocket(self) -> None:
        self.centre.arm_rocket()
        if self._backend_client:
            self._backend_client.set_armed(True)
        self.refresh()

    def disarm_rocket(self) -> None:
        self.centre.disarm_rocket()
        if self._backend_client:
            self._backend_client.set_armed(False)
        self.refresh()

    def _toggle_rocket_arm(self) -> None:
        if self._backend_client:
            current_armed = self._backend_client.state.get("armed", False)
            new_armed = not current_armed
            self._backend_client.set_armed(new_armed)
            if new_armed:
                self.centre.arm_rocket()
            else:
                self.centre.disarm_rocket()
            self.refresh()
        else:
            if self.centre.is_rocket_armed():
                self.disarm_rocket()
            else:
                self.arm_rocket()

    def _setup_telemetry_commands(self, panel: ModePanel) -> None:
        c = ThemeManager.C()

        # ── Frequency selector rows: Rocket & CanSat ─────────────────────────
        freq_grid = QGridLayout()
        freq_grid.setHorizontalSpacing(8)
        freq_grid.setVerticalSpacing(8)

        # Row 0: Rocket frequency [ Rocket ] [ Original ] [ Backup ]
        rkt_capsule = self._label_capsule("Rocket")
        freq_grid.addWidget(rkt_capsule, 0, 0)

        self._rocket_freq_buttons: Dict[str, QPushButton] = {}
        for i, (mode, cmd, label, freq_str) in enumerate([
            ("ORIGINAL", "FQ,0", "Original", "866.50 MHz (Main)"),
            ("BACKUP", "FQ,1", "Backup", "865.25 MHz (Backup)"),
        ]):
            btn = QPushButton(label)
            btn.setFont(QFont("monospace", 9, QFont.Weight.Bold))
            btn.setMinimumHeight(30)
            btn.setStyleSheet(ThemeManager.status_button_css("red"))
            btn.setToolTip(f"Rocket telemetry frequency: {label} ({freq_str})")
            btn.clicked.connect(lambda _c, c_cmd=cmd, m=mode: self._on_rocket_freq_clicked(c_cmd, m))
            self._rocket_freq_buttons[mode] = btn
            self._rocket_freq_buttons[cmd] = btn
            freq_grid.addWidget(btn, 0, 1 + i)

        # Row 1: CanSat frequency [ CanSat ] [ Original ] [ Backup ]
        cansat_capsule = self._label_capsule("CanSat")
        freq_grid.addWidget(cansat_capsule, 1, 0)

        self._cansat_freq_buttons: Dict[str, QPushButton] = {}
        for i, (mode, label) in enumerate([
            ("ORIGINAL", "Original"),
            ("BACKUP", "Backup"),
        ]):
            btn = QPushButton(label)
            btn.setFont(QFont("monospace", 9, QFont.Weight.Bold))
            btn.setMinimumHeight(30)
            btn.setEnabled(False)
            btn.setStyleSheet(ThemeManager.status_button_css("red"))
            btn.setToolTip("CanSat link not implemented")
            self._cansat_freq_buttons[mode] = btn
            self._cansat_freq_buttons[f"CANSAT_{mode}"] = btn
            freq_grid.addWidget(btn, 1, 1 + i)

        panel.layout.addLayout(freq_grid)

        self._command_buttons: Dict[str, QPushButton] = {}
        b_ref = QPushButton("Refresh Processor")
        b_ref.setFont(QFont("monospace", 9, QFont.Weight.Bold))
        b_ref.setMinimumHeight(32)
        b_ref.setStyleSheet(ThemeManager.status_button_css("neutral"))
        b_ref.setToolTip("Software reset (reinitialise flight computer) — confirmation required")
        b_ref.clicked.connect(self._on_refresh_processor_clicked)
        self._command_buttons["RF"] = b_ref
        self._command_buttons["REFRESH_PROCESSOR"] = b_ref
        panel.layout.addWidget(b_ref)

    def _label_capsule(self, text: str) -> QFrame:
        c = ThemeManager.C()
        frame = QFrame()
        frame.setStyleSheet(
            f"QFrame {{ background-color: {c.BG_INPUT}; border: 1px solid {c.BORDER}; border-radius: 4px; }}"
        )
        frame.setMinimumHeight(30)
        lay = QHBoxLayout(frame)
        lay.setContentsMargins(12, 4, 12, 4)
        lbl = QLabel(text)
        lbl.setFont(QFont("monospace", 8, QFont.Weight.Bold))
        lbl.setStyleSheet(f"color: {c.TEXT_DIM}; border: none;")
        lay.addWidget(lbl)
        return frame

    # ── Bridge Wiring & Button Status Handling ───────────────────────────

    _backend_client: Optional[GroundStationBackendClient] = None

    def set_backend_client(self, client: GroundStationBackendClient) -> None:
        self._backend_client = client
        client.buttons_changed.connect(self._on_backend_buttons_changed)
        client.state_changed.connect(self._on_backend_state_changed)
        if client.button_colors:
            self._on_backend_buttons_changed({
                "default": client.default_color,
                "buttons": client.button_colors,
                "tooltips": client.button_tooltips,
            })
        if client.state:
            self._on_backend_state_changed(client.state)

    def _collect_all_buttons(self) -> Dict[str, QPushButton]:
        reg: Dict[str, QPushButton] = {}
        if hasattr(self, "mode_panel"):
            reg.update(self.mode_panel.buttons)
        if hasattr(self, "_rocket_freq_buttons"):
            reg.update(self._rocket_freq_buttons)
        if hasattr(self, "_cansat_freq_buttons"):
            reg.update(self._cansat_freq_buttons)
        if hasattr(self, "_command_buttons"):
            reg.update(self._command_buttons)
        if hasattr(self, "actuator_panels"):
            for p in self.actuator_panels.values():
                reg.update(p.buttons)
        if hasattr(self, "sequence_panel"):
            reg.update(self.sequence_panel.buttons)
        if hasattr(self, "calibration_panel"):
            reg.update(self.calibration_panel.buttons)
        if hasattr(self, "diagnostics_panel"):
            reg.update(self.diagnostics_panel.buttons)
        return reg

    def _on_backend_buttons_changed(self, data: dict) -> None:
        default_color = data.get("default", "neutral")
        buttons = data.get("buttons", {})
        tooltips = data.get("tooltips", {})

        for key, btn in self._collect_all_buttons().items():
            if btn is None:
                continue
            if "CANSAT" in key:
                btn.setStyleSheet(ThemeManager.status_button_css("red"))
                btn.setToolTip("CanSat link not implemented")
                btn.setEnabled(False)
                continue

            color = buttons.get(key, default_color)
            tip = tooltips.get(key, tooltips.get("default", ""))
            btn.setStyleSheet(ThemeManager.status_button_css(color))
            if tip:
                btn.setToolTip(tip)

    def _on_backend_state_changed(self, state: dict) -> None:
        armed = state.get("armed", False)
        c = ThemeManager.C()
        self._rocket_arm_btn.setText("Disarm" if armed else "Arm")
        self._rocket_arm_btn.setStyleSheet(
            ThemeManager.status_button_css("red", armed=armed) if armed else _btn_css(c.RED)
        )
        self._rocket_safety_label.setText("ARMED" if armed else "UNARMED")
        self._rocket_safety_label.setStyleSheet(
            f"color: {c.RED if armed else c.GREEN}; padding: 0 4px; font-weight: bold;"
        )

        target = state.get("target", "ROCKET")
        for vid, b in getattr(self, "_vehicle_buttons", {}).items():
            is_sel = (vid.value == target)
            b.setChecked(is_sel)
            if is_sel:
                b.setStyleSheet(
                    f"QPushButton {{ background-color: #122840; color: {c.CYAN}; border: 1px solid {c.CYAN}; "
                    f"border-radius: 4px; padding: 6px 12px; font-weight: bold; }}"
                )
            else:
                b.setStyleSheet(_btn_css(c.TEXT_DIM))

    def _send_command_via_bridge(self, cmd: str, key: Optional[str] = None) -> None:
        key = key or cmd
        if self._backend_client and self._backend_client.state.get("serial", {}).get("state") == "DISCONNECTED":
            self.notice.emit(f"Cannot send {cmd}: Teensy disconnected")
            self.refresh()
            return

        rec = self.centre.send(self.vehicle, cmd)
        # If CommandCentre._transmit is not wired to supervisor (e.g. standalone test without app.py),
        # transmit directly via backend client with matching sequence number
        is_dummy_transmit = getattr(self.centre._transmit, "__code__", None) == (lambda _t, _c, _s: None).__code__
        if is_dummy_transmit and self._backend_client:
            self._backend_client.send_command(cmd, key, seq=rec.sequence)

        msg = self._command_popup_text(cmd)
        self.notice.emit(msg)
        self.refresh()

    def _on_rocket_freq_clicked(self, cmd: str, mode: str) -> None:
        freq_str = "Original (866.50 MHz)" if mode == "ORIGINAL" else "Backup (865.25 MHz)"
        reply = QMessageBox.question(
            self,
            "Confirm Frequency Switch",
            f"Are you sure you want to switch Rocket and Ground radio frequency to {freq_str}?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self._send_command_via_bridge(cmd, key=cmd)

    def _on_refresh_processor_clicked(self) -> None:
        reply = QMessageBox.question(
            self,
            "Confirm Processor Refresh",
            "Are you sure you want to trigger a software restart of the Rocket processor (RF)?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self._send_command_via_bridge("RF", key="RF")

    def _on_dangerous_command_clicked(self, cmd: str) -> None:
        if self._backend_client and not self._backend_client.state.get("armed", False):
            QMessageBox.warning(
                self,
                "Command Rejected",
                f"Command '{cmd}' requires the Rocket to be ARMED first.\n"
                "Please click the 'Arm' button before executing dangerous commands."
            )
            return

        confirm_text = {
            "FSEQ": "Run FULL AUTOMATIC 3-SERVO SEQUENCE (Door -> CanSat -> Separation)?",
            "SEP": "FIRE VEHICLE SEPARATION MECHANISM?",
            "FPL": "FORCE PRE-LAUNCH FLIGHT STATE?",
        }.get(cmd, f"Execute dangerous command {cmd}?")

        reply = QMessageBox.question(
            self,
            "Confirm Destructive Command",
            confirm_text,
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            if hasattr(self, "sequence_panel"):
                self.sequence_panel.note_command_sent(cmd)
            self._send_command_via_bridge(cmd, key=cmd)

    def _send_diagnostics_command(self, cmd: str) -> None:
        if cmd == "FPL":
            self._on_dangerous_command_clicked("FPL")
            return
        self._send_command_via_bridge(cmd, key=cmd)

    def _on_actuator_angle(self, actuator_id: str, angle: int) -> None:
        ch_map = {"ACT_DOOR": 0, "ACT_DEPLOY": 1, "ACT_SEPARATE": 2}
        ch = ch_map.get(actuator_id, 0)
        cmd = f"SV,{ch},{angle}"
        panel = self.actuator_panels.get(actuator_id)
        if panel is not None:
            panel.actuator.locked = False
            panel.actuator.note_commanded(float(angle))
            panel.set_last_command(cmd)
        self._send_command_via_bridge(cmd, key=cmd)

    def _on_actuator_lock(self, actuator_id: str) -> None:
        ch_map = {"ACT_DOOR": 0, "ACT_DEPLOY": 1, "ACT_SEPARATE": 2}
        ch = ch_map.get(actuator_id, 0)
        cmd = f"LOCK,{ch}"
        lock_angles = {"ACT_DOOR": 50.0, "ACT_DEPLOY": 80.0, "ACT_SEPARATE": 80.0}
        angle = lock_angles.get(actuator_id, 0.0)
        panel = self.actuator_panels.get(actuator_id)
        if panel is not None:
            panel.actuator.locked = True
            panel.actuator.note_commanded(angle)
            panel.set_last_command(cmd)
        self._send_command_via_bridge(cmd, key=cmd)

    def _on_actuator_unlock(self, actuator_id: str) -> None:
        ch_map = {"ACT_DOOR": 0, "ACT_DEPLOY": 1, "ACT_SEPARATE": 2}
        ch = ch_map.get(actuator_id, 0)
        cmd = f"UNLOCK,{ch}"
        panel = self.actuator_panels.get(actuator_id)
        if panel is not None:
            panel.actuator.locked = False
            panel.actuator.note_commanded(0.0)
            panel.set_last_command(cmd)
        self._send_command_via_bridge(cmd, key=cmd)

    def _send_actuator_command(self, actuator_id: str, command: str) -> None:
        panel = self.actuator_panels.get(actuator_id)
        if panel is not None:
            panel.set_last_command(command)
        self.sequence_panel.note_command_sent(command)
        self._send_command_via_bridge(command, key=command)

    def _send_sequence_command(self, command: str) -> None:
        cmd_u = command.upper()
        if cmd_u in ("FSEQ", "SEP"):
            self._on_dangerous_command_clicked(cmd_u)
            return

        self.sequence_panel.note_command_sent(command)
        if cmd_u in ("LOCKALL", "LOCK:ALL", "TEST4"):
            lock_angles = {"ACT_DOOR": 50.0, "ACT_DEPLOY": 80.0, "ACT_SEPARATE": 80.0}
            for sid, p in self.actuator_panels.items():
                p.actuator.locked = True
                p.actuator.note_commanded(lock_angles.get(sid, 0.0))
                p.set_last_command("LOCKALL")
        elif cmd_u in ("SEQRST", "RESET"):
            for p in self.actuator_panels.values():
                p.actuator.locked = False
                p.actuator.note_commanded(0.0)
                p.set_last_command("SEQRST")
        elif cmd_u in ("TEST1", "MSEQ", "TEST5"):
            p = self.actuator_panels.get("ACT_DOOR")
            if p:
                p.actuator.note_commanded(0.0)
                p.set_last_command(cmd_u)
        elif cmd_u == "TEST2":
            p = self.actuator_panels.get("ACT_DEPLOY")
            if p:
                p.actuator.note_commanded(0.0)
                p.set_last_command("TEST2")
        elif cmd_u == "TEST3":
            p = self.actuator_panels.get("ACT_SEPARATE")
            if p:
                p.actuator.note_commanded(0.0)
                p.set_last_command("TEST3")
        elif cmd_u in ("NSTEP", "NEXT"):
            if self.sequence_panel._step == 2:
                p = self.actuator_panels.get("ACT_DEPLOY")
                if p:
                    p.actuator.note_commanded(0.0)
                    p.set_last_command("NSTEP")
            elif self.sequence_panel._step >= 3:
                p = self.actuator_panels.get("ACT_SEPARATE")
                if p:
                    p.actuator.note_commanded(0.0)
                    p.set_last_command("NSTEP")

        self._send_command_via_bridge(command, key=command)

    def _select_vehicle(self, vid: VehicleID) -> None:
        self.vehicle = vid
        c = ThemeManager.C()
        for v, b in self._vehicle_buttons.items():
            is_sel = (v is vid)
            b.setChecked(is_sel)
            if is_sel:
                b.setStyleSheet(
                    f"QPushButton {{ background-color: #122840; color: {c.CYAN}; border: 1px solid {c.CYAN}; "
                    f"border-radius: 4px; padding: 6px 12px; font-weight: bold; }}"
                )
            else:
                b.setStyleSheet(_btn_css(c.TEXT_DIM))
        if self._backend_client:
            self._backend_client.set_target(vid.value)
        self.mode_panel.modes = self.centre.modes[vid]
        self.calibration_panel.run = self.calibration[vid]
        self.refresh()

    def _send_guarded(self, command: str) -> None:
        self._send_command_via_bridge(command, key=command)

    def _send_mode_command(self, command: str) -> None:
        self._send_command_via_bridge(command, key=command)

    def _send_move(self, actuator_id: str, delta: float) -> None:
        actuator = self.actuators[actuator_id]
        if actuator.position is None:
            # §10.1 — "Never display commanded position as if it were actual."
            # If we don't know where the servo is, substituting home (180°)
            # would silently send an absolute target based on a position the
            # vehicle never confirmed.  A 0° servo (falsy) + home fallback
            # = 180° error on an irreversible mechanism.
            # Refuse the move until the vehicle has echoed a position back.
            self.refresh()
            return
        target = actuator.position + delta
        actuator.note_commanded(target)
        self.centre.send(self.vehicle, f"{actuator_id}_MOVE {target:.1f}")
        self.refresh()

    # ── inbound ──────────────────────────────────────────────────────────

    def set_vehicle_state(self, vid: VehicleID,
                          state: Optional[FlightState]) -> None:
        self._states[vid] = state

    def set_link_state(self, vid: VehicleID, link: VehicleLinkState) -> None:
        self._links[vid] = link

    def set_usb_direct(self, on: bool) -> None:
        self._usb_direct = on

    def note_actuator_echo(self, actuator_id: str, actual: float) -> None:
        a = self.actuators.get(actuator_id)
        if a is not None:
            a.note_echo(actual)

    # ── refresh ──────────────────────────────────────────────────────────

    def refresh(self) -> None:
        # §10.6 — apply the auto-disarm timeout before reading any armed
        # state.  armed_control() is now a pure read; expire() is the only
        # place that clears _armed on timeout, so it must be called here.
        self.arming.expire()

        state = self._states[self.vehicle]
        link = self._links[self.vehicle]

        for name, button in self._command_buttons.items():
            if name == "REFRESH_PROCESSOR":
                button.setEnabled(True)
                button.setToolTip(COMMANDS[name].description)
            else:
                result = check_guard(COMMANDS[name], state, link,
                                     self.simulator_mode, self._usb_direct)
                button.setEnabled(result.allowed)
                button.setToolTip(result.reason or COMMANDS[name].description)

        # §10.2 — blocked outside BOOT and PRE_LAUNCH.
        on_ground = state in GROUND_STATES if state is not None else True
        self.calibration_panel.set_enabled_by_guard(
            on_ground and link is not VehicleLinkState.LOST)

        # §9.7 / §10.1 — actuators are rocket hardware; the panels are
        # disabled rather than hidden when CanSat is selected, so the
        # operator can see they exist and why they are unavailable.
        rocket_selected = self.vehicle is VehicleID.ROCKET
        self._rocket_arm_btn.setVisible(rocket_selected)
        self._rocket_safety_label.setVisible(rocket_selected)
        if rocket_selected:
            c = ThemeManager.C()
            armed = self.centre.is_rocket_armed()
            self._rocket_arm_btn.setText("Disarm" if armed else "Arm")
            self._rocket_arm_btn.setStyleSheet(
                _armed_btn_css() if armed else _btn_css(c.RED))
            self._rocket_arm_btn.setToolTip(
                "Disarm Rocket (return to UNARMED development state)" if armed
                else "Arm Rocket (enter ARMED flight-ready state)")
            self._rocket_safety_label.setText("ARMED" if armed else "UNARMED")
            self._rocket_safety_label.setStyleSheet(
                f"color: {c.RED if armed else c.GREEN}; padding: 0 4px;")

        for p in self.actuator_panels.values():
            p.set_enabled_by_guard(
                rocket_selected and link is not VehicleLinkState.LOST,
                "" if rocket_selected else "Actuators are on the Rocket.",
                rocket_selected=rocket_selected)

        self.sequence_panel.set_enabled(rocket_selected)
        self.sequence_panel.refresh()

        self.mode_panel.refresh()
        self.calibration_panel.refresh()
        self.log_panel.refresh()

        # §10.4 — the dangerous combination.
        modes = self.centre.modes[self.vehicle]
        self._banner.setVisible(modes.dangerous)
        if modes.dangerous:
            self._banner.setText(
                "SAMPLING HIBERNATED WHILE TRANSMITTING — packets are "
                "arriving but the values are not live measurements")

        # §10.7 — surface unacknowledged commands that have aged out
        # without ever auto-retrying them.
        self.centre.poll_timeouts()

    def handle_ground_cmd(self, cmd_id: int, status: str, info: str, raw_line: str) -> None:
        self.centre.handle_ground_cmd(cmd_id, status, info, raw_line)
        status_u = status.upper()
        if status_u == "OK":
            self.notice.emit(f"Ground confirmation: Command {cmd_id} OK")
        elif status_u == "REJECTED":
            self.notice.emit(f"Ground rejected: Command {cmd_id} ({info})")
        elif status_u == "FAILED":
            self.notice.emit(f"Ground failed: Command {cmd_id} ({info})")
        elif status_u == "BADCMD":
            self.notice.emit(f"Ground syntax error: Command {cmd_id} BADCMD")
        elif status_u == "BUSY":
            self.notice.emit(f"Ground radio busy: Command {cmd_id} ({info})")
        self.refresh()

    def handle_servo_report(self, ch: int, deg: int, est: int, rem: int, mode: int, step: int, raw_line: str) -> None:
        ch_to_id = {0: "ACT_DOOR", 1: "ACT_DEPLOY", 2: "ACT_SEPARATE"}
        act_id = ch_to_id.get(ch)
        if act_id and act_id in self.actuator_panels:
            panel = self.actuator_panels[act_id]
            a = panel.actuator
            if deg >= 0:
                a.note_commanded(float(deg))
            if est >= 0:
                a.note_echo(float(est))
                a.is_estimate = True
            else:
                a.actual = None
            if rem >= 0:
                a.remaining_deg = float(rem)
            else:
                a.remaining_deg = None
            panel.refresh()

        mode_names = {0: "INDIVIDUAL", 1: "MANUAL", 2: "FULL SEQUENCE"}
        m_str = mode_names.get(mode, "IDLE")
        self.sequence_panel._mode = m_str
        self.sequence_panel._step = step
        self.sequence_panel._step_text = f"{step} / 3"
        self.sequence_panel._status = f"Vehicle: Mode={m_str}, Step={step}/3"
        self.sequence_panel.refresh()
        self.refresh()

    def handle_command_reply(self, cmd: str, status: str, details: dict, raw_line: str) -> None:
        cmd_u = cmd.upper()
        cal = self.calibration.get(VehicleID.ROCKET)
        if status == "ACK":
            if cmd_u == "CALA":
                if cal:
                    cal.set("ALTITUDE", CalibrationOutcome.PASS,
                            f"p0={details.get('p0_Pa')} Pa, T0={details.get('T0')} °C, gnss0={details.get('gnss_ground')} m")
            elif cmd_u == "CALI":
                if cal:
                    cal.set("IMU", CalibrationOutcome.PASS,
                            f"axis={details.get('axis')}, bias={details.get('bias_mm_s2')} mm/s2, tilt={details.get('bno_tilt')}°")
            elif cmd_u == "CAL":
                if cal:
                    alt_ok = (details.get("alt_result") == "OK")
                    imu_ok = (details.get("imu_result") == "OK")
                    cal.set("ALTITUDE", CalibrationOutcome.PASS if alt_ok else CalibrationOutcome.FAIL)
                    cal.set("IMU", CalibrationOutcome.PASS if imu_ok else CalibrationOutcome.FAIL)
            self.notice.emit(f"ACK: {cmd} executed successfully")
        elif status == "NACK":
            reason = details.get("reason", "unknown")
            if "CALA" in cmd_u or "ALT" in cmd_u:
                if cal:
                    cal.set("ALTITUDE", CalibrationOutcome.FAIL, f"Refused: {reason}")
            elif "CALI" in cmd_u or "IMU" in cmd_u:
                if cal:
                    cal.set("IMU", CalibrationOutcome.FAIL, f"Refused: {reason}")
            elif "CAL" in cmd_u:
                if cal:
                    cal.set("ALTITUDE", CalibrationOutcome.FAIL, f"Refused: {reason}")
                    cal.set("IMU", CalibrationOutcome.FAIL, f"Refused: {reason}")
            self.notice.emit(f"NACK: {cmd} refused ({reason})")

        # Map firmware reply name to potential GUI command names
        alias_groups = {
            "SD": ["ENABLE_TELEMETRY", "LIVE", "SET_MODE_LIVE", "SD"],
            "XD": ["DISABLE_TELEMETRY", "HIBERNATE", "SET_MODE_HIBERNATE", "XD"],
            "CALA": ["CALIBRATE_ALTITUDE", "CALA", "CAL,1"],
            "CALI": ["CALIBRATE_IMU", "CALI", "CAL,2"],
            "CAL": ["CALIBRATE_ALL", "CAL"],
            "LOCK": ["LOCK:DOOR", "LOCK:CANSAT", "LOCK:SEPARATION", "LOCK,0", "LOCK,1", "LOCK,2", "LOCK"],
            "UNLOCK": ["UNLOCK:DOOR", "UNLOCK:CANSAT", "UNLOCK:SEPARATION", "UNLOCK,0", "UNLOCK,1", "UNLOCK,2", "UNLOCK"],
            "SV": ["DOOR:30", "DOOR:60", "DOOR:90", "CANSAT:30", "CANSAT:60", "CANSAT:90", "SEPARATION:30", "SEPARATION:60", "SEPARATION:90", "SV"],
            "LOCKALL": ["LOCK ALL", "LOCK:ALL", "LOCKALL"],
            "SEQRST": ["RESET", "SEQRST"],
            "MSEQ": ["MANUAL SEQUENCE", "TEST5", "MSEQ"],
            "NSTEP": ["NEXT", "NEXT SEQUENCE STEP", "NSTEP"],
            "FSEQ": ["FULL SEQUENCE", "TEST6", "FSEQ"],
            "TEST1": ["TEST1"],
            "TEST2": ["TEST2"],
            "TEST3": ["TEST3"],
            "RF": ["REFRESH_PROCESSOR", "RF"],
            "FQ": ["FQ,0", "FQ,1", "FREQ:ROCKET:ORIGINAL", "FREQ:ROCKET:BACKUP"],
            "TX": ["TX,0", "TX,1"],
            "PG": ["PG", "PING"],
            "BZ": ["BZ,0", "BZ,1"],
            "FPL": ["FPL", "FPL,1"],
            "SEP": ["SEP", "ACT_SEPARATE", "MANUAL_DEPLOY", "SEPARATION"],
            "LK": ["LK", "LINK_CHECK"],
            "ST": ["ST", "REQUEST_STATUS", "STATUS"],
            "GR": ["GR", "GNSS_RESET"],
            "LR": ["LR", "LORA_RESET", "LORA_REINIT"],
            "ENABLE": ["ENABLE", "LINK", "ENABLE_TELEMETRY"],
            "DISABLE": ["DISABLE", "DISABLE_TELEMETRY"],
            "GS": ["GS", "GROUND_STATUS"],
        }
        aliases = alias_groups.get(cmd_u, [cmd_u])

        # Also acknowledge in CommandCentre if sequence or command matches
        for rec in reversed(self.centre.records):
            if rec.unacknowledged:
                rec_cmd_u = rec.command.upper()
                matched = (rec_cmd_u == cmd_u or cmd_u in rec_cmd_u or any(a == rec_cmd_u or a in rec_cmd_u for a in aliases))
                if matched:
                    if status == "ACK":
                        self.centre.acknowledge(rec.sequence, raw_line)
                        self.centre.mark_executed(rec.sequence, raw_line)
                    else:
                        self.centre.reject(rec.sequence, details.get("reason", "NACK"))
                    break

        self.refresh()


if __name__ == "__main__":
    import sys
    from PyQt6.QtWidgets import QApplication
    app = QApplication(sys.argv)
    app.setStyleSheet(ThemeManager.main_stylesheet())
    centre = CommandCentre(lambda vid, cmd: print(f"Transmitted to {vid.value}: {cmd}"))
    page = CommandPage(centre)
    page.resize(1100, 750)
    page.show()
    sys.exit(app.exec())

