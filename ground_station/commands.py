"""
ground_station/commands.py
══════════════════════════
§10.5 command set, §10.4 sampling-vs-transmission state, §10.7
acknowledgment lifecycle. Qt-free so every guard is testable.

THREE RULES THIS MODULE EXISTS TO ENFORCE
─────────────────────────────────────────

1. §1.4 — "No command is ever sent automatically. Every telecommand is
   operator-initiated." There is no timer, no retry loop and no
   watchdog in this file. `CommandCentre.send()` is only ever called
   from a widget's clicked handler.

2. §10.7 — retry is operator-initiated only. "An auto-retried
   ACT_SEPARATE that was in fact received the first time fires the
   mechanism twice." `resend()` exists and is deliberately not wired to
   anything that could call it on its own.

3. §10.4 — both mode indicators show the *confirmed-active* mode from
   the vehicle's acknowledgment, never the last-requested mode. Never
   flip optimistically. This is why `VehicleModes` has three values per
   axis rather than two: the third is "we asked and do not yet know".
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Dict, List, Optional, Tuple

try:
    from .models import (
        FlightState, GROUND_STATES, RocketSafetyState, VehicleID, VehicleLinkState,
    )
except (ImportError, ValueError):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from ground_station.models import (
        FlightState, GROUND_STATES, RocketSafetyState, VehicleID, VehicleLinkState,
    )


# ─────────────────────────────────────────────────────────────────────────────
#  §10.5 — the command set and its guards
# ─────────────────────────────────────────────────────────────────────────────

class Guard(Enum):
    NONE = "none"
    CONFIRM_IN_FLIGHT = "confirm if in flight"
    GROUND_ONLY = "BOOT / PRE_LAUNCH only"
    TWO_STEP = "two-step confirm"
    DESTRUCTIVE = "two-step arm → fire"
    RECOVERY_USB_ONLY = "direct USB only; RECOVERY only"
    NOT_IN_SIMULATOR_MODE = "disabled in Simulator Mode"


@dataclass(frozen=True)
class CommandSpec:
    name: str
    description: str
    guard: Guard = Guard.NONE
    #: Rocket-only commands (§10.5 marks MANUAL_DEPLOY as such).
    rocket_only: bool = False
    #: §10.6 — irreversible in flight, needs arm → fire rather than a
    #: single click.
    destructive: bool = False


#: §10.5, verbatim. RESET_COUNTERS is absent by design — it was removed,
#: and counters are monotonic for the entire mission (§4.1 field 4).
COMMANDS: Dict[str, CommandSpec] = {
    c.name: c for c in [
        CommandSpec("ENABLE_TELEMETRY", "Start transmission"),
        CommandSpec("DISABLE_TELEMETRY", "Stop transmission"),
        CommandSpec("SET_MODE_LIVE", "Start sampling",
                    Guard.CONFIRM_IN_FLIGHT),
        CommandSpec("SET_MODE_HIBERNATE", "Stop sampling",
                    Guard.CONFIRM_IN_FLIGHT),
        CommandSpec("ENTER_SIMULATION", "Flight software simulation mode",
                    Guard.NOT_IN_SIMULATOR_MODE),
        CommandSpec("REQUEST_STATUS", "System health report"),
        CommandSpec("REFRESH_PROCESSOR", "Reinitialise the processing loop",
                    Guard.NONE),
        CommandSpec("MANUAL_DEPLOY", "Emergency manual deployment",
                    Guard.DESTRUCTIVE, rocket_only=True, destructive=True),
        CommandSpec("LIST_FILES", "List experiment files",
                    Guard.RECOVERY_USB_ONLY),
        CommandSpec("REQUEST_FILE", "Retrieve an experiment file",
                    Guard.RECOVERY_USB_ONLY),
        CommandSpec("ABORT_TRANSFER", "Abort the current transfer",
                    Guard.RECOVERY_USB_ONLY),
        # §10.2 — calibration commands.  GROUND_ONLY because zeroing the
        # barometer mid-flight corrupts the altitude datum for the rest of
        # the mission.  Each sensor is a separate command so the operator
        # can re-run one without re-running all.
        CommandSpec("CALIBRATE_PRESSURE",
                    "Zero barometer to local atmospheric pressure",
                    Guard.GROUND_ONLY),
        CommandSpec("CALIBRATE_ALTITUDE",
                    "Zero altimeter AGL datum at ground level",
                    Guard.GROUND_ONLY),
        CommandSpec("CALIBRATE_IMU",
                    "Zero all IMU axes",
                    Guard.GROUND_ONLY),
        CommandSpec("CALIBRATE_GNSS",
                    "Capture GNSS reference fix at the pad",
                    Guard.GROUND_ONLY),
        CommandSpec("CALIBRATE_TEMPERATURE",
                    "Record ambient atmospheric temperature",
                    Guard.GROUND_ONLY),
        CommandSpec("CALIBRATE_ALL",
                    "Run all calibrations in sequence",
                    Guard.GROUND_ONLY),
        # Multi-test servo bench commands
        CommandSpec("TEST1",
                    "Individual door servo test (lock 0° -> actuate 60°)",
                    Guard.NONE, rocket_only=True),
        CommandSpec("TEST2",
                    "Individual CanSat deploy servo test (lock 0° -> actuate 60°)",
                    Guard.NONE, rocket_only=True),
        CommandSpec("TEST3",
                    "Individual separation servo test (lock 0° -> actuate 60°)",
                    Guard.NONE, rocket_only=True),
        CommandSpec("TEST4",
                    "Lock all servos to safe position (0°)",
                    Guard.NONE, rocket_only=True),
        CommandSpec("TEST5",
                    "Start manual sequence (Step 1: Door)",
                    Guard.NONE, rocket_only=True),
        CommandSpec("NEXT",
                    "Advance manual sequence to next step",
                    Guard.NONE, rocket_only=True),
        CommandSpec("TEST6",
                    "Full automatic 3-servo sequence",
                    Guard.NONE, rocket_only=True),
        CommandSpec("RESET",
                    "Reset and lock all servos (0°)",
                    Guard.NONE, rocket_only=True),
        # Hardware servo angle and lock commands
        CommandSpec("DOOR:30", "Set Rocket Door angle to 30°", Guard.NONE, rocket_only=True),
        CommandSpec("DOOR:60", "Set Rocket Door angle to 60°", Guard.NONE, rocket_only=True),
        CommandSpec("DOOR:90", "Set Rocket Door angle to 90°", Guard.NONE, rocket_only=True),
        CommandSpec("LOCK:DOOR", "Lock Rocket Door (0° safe position)", Guard.NONE, rocket_only=True),
        CommandSpec("CANSAT:30", "Set CanSat deployment angle to 30°", Guard.NONE, rocket_only=True),
        CommandSpec("CANSAT:60", "Set CanSat deployment angle to 60°", Guard.NONE, rocket_only=True),
        CommandSpec("CANSAT:90", "Set CanSat deployment angle to 90°", Guard.NONE, rocket_only=True),
        CommandSpec("LOCK:CANSAT", "Lock CanSat deployment (0° safe position)", Guard.NONE, rocket_only=True),
        CommandSpec("SEPARATION:30", "Set Separation angle to 30°", Guard.NONE, rocket_only=True),
        CommandSpec("SEPARATION:60", "Set Separation angle to 60°", Guard.NONE, rocket_only=True),
        CommandSpec("SEPARATION:90", "Set Separation angle to 90°", Guard.NONE, rocket_only=True),
        CommandSpec("LOCK:SEPARATION", "Lock Separation (0° safe position)", Guard.NONE, rocket_only=True),
        CommandSpec("LOCK:ALL", "Lock all servos to safe position (0°)", Guard.NONE, rocket_only=True),
        # RF Frequency selection commands
        CommandSpec("FREQ:ROCKET:ORIGINAL", "Set Rocket frequency to Original (866.50 MHz)", Guard.NONE),
        CommandSpec("FREQ:ROCKET:BACKUP", "Set Rocket frequency to Backup (865.25 MHz)", Guard.NONE),
        CommandSpec("FREQ:CANSAT:ORIGINAL", "Set CanSat frequency to Original (866.50 MHz)", Guard.NONE),
        CommandSpec("FREQ:CANSAT:BACKUP", "Set CanSat frequency to Backup (865.25 MHz)", Guard.NONE),
        # Documented README hardware commands
        CommandSpec("CAL", "Calibrate all sensors (altitude + IMU)", Guard.GROUND_ONLY),
        CommandSpec("CALA", "Altitude calibration (pad pressure, temp, GNSS ground alt)", Guard.GROUND_ONLY),
        CommandSpec("CALI", "IMU calibration (ADXL345 axis/offset, BNO08x gyro zero/spin)", Guard.GROUND_ONLY),
        CommandSpec("FQ,0", "Set Rocket + Ground radio to Main frequency (866.50 MHz)", Guard.NONE),
        CommandSpec("FQ,1", "Set Rocket + Ground radio to Backup frequency (865.25 MHz)", Guard.NONE),
        CommandSpec("GF,0", "Set Ground Station radio to Main frequency (866.50 MHz)", Guard.NONE),
        CommandSpec("GF,1", "Set Ground Station radio to Backup frequency (865.25 MHz)", Guard.NONE),
        CommandSpec("TX,0", "Silence rocket transmitter (receiver stays on)", Guard.NONE),
        CommandSpec("TX,1", "Turn on rocket transmitter", Guard.NONE),
        CommandSpec("FPL", "Force PRE-LAUNCH state", Guard.NONE),
        CommandSpec("FPL,1", "Force PRE-LAUNCH overriding checks", Guard.NONE),
        CommandSpec("FSEQ", "Full sequence: door, +1s CanSat, +1.5s separation", Guard.NONE, rocket_only=True),
        CommandSpec("RF", "Software reset (reinitialise flight computer)", Guard.NONE),
        CommandSpec("LOCKALL", "Lock all servos: door 50°, CanSat 80°, separation 80°", Guard.NONE, rocket_only=True),
        CommandSpec("UNLOCK", "Unlock all servos to 0° safe position", Guard.NONE, rocket_only=True),
        CommandSpec("LOCK", "Lock selected servo mechanism", Guard.NONE, rocket_only=True),
        CommandSpec("MSEQ", "Manual deployment sequence start", Guard.NONE, rocket_only=True),
        CommandSpec("NSTEP", "Advance to next manual deployment step", Guard.NONE, rocket_only=True),
        CommandSpec("SEQRST", "Reset deployment sequence", Guard.NONE, rocket_only=True),
        CommandSpec("SEP", "Fire separation mechanism", Guard.DESTRUCTIVE, rocket_only=True, destructive=True),
        CommandSpec("BZ", "Trigger buzzer test", Guard.NONE, rocket_only=True),
        CommandSpec("GR", "Reconfigure / restart GNSS module", Guard.NONE),
        CommandSpec("LR", "Reinitialise LoRa radio", Guard.NONE),
        CommandSpec("BITE", "Run Built-In Test Equipment diagnostic", Guard.NONE),
        CommandSpec("ENABLE", "Enable ground station reception", Guard.NONE),
        CommandSpec("DISABLE", "Disable ground station reception", Guard.NONE),
        CommandSpec("GS", "Ground station status query", Guard.NONE),
        CommandSpec("LIVE", "Set live sampling mode", Guard.CONFIRM_IN_FLIGHT),
        CommandSpec("HIBERNATE", "Set hibernate mode", Guard.CONFIRM_IN_FLIGHT),
        # Channel-specific actuator lock/unlock and servo angles
        CommandSpec("UNLOCK:DOOR", "Unlock Rocket Door (0° safe position)", Guard.NONE, rocket_only=True),
        CommandSpec("UNLOCK:CANSAT", "Unlock CanSat deployment (0° safe position)", Guard.NONE, rocket_only=True),
        CommandSpec("UNLOCK:SEPARATION", "Unlock Separation (0° safe position)", Guard.NONE, rocket_only=True),
        CommandSpec("LOCK,0", "Lock Rocket Door (50° safe position)", Guard.NONE, rocket_only=True),
        CommandSpec("LOCK,1", "Lock CanSat deployment (80° safe position)", Guard.NONE, rocket_only=True),
        CommandSpec("LOCK,2", "Lock Separation (80° safe position)", Guard.NONE, rocket_only=True),
        CommandSpec("UNLOCK,0", "Unlock Rocket Door (0° position)", Guard.NONE, rocket_only=True),
        CommandSpec("UNLOCK,1", "Unlock CanSat deployment (0° position)", Guard.NONE, rocket_only=True),
        CommandSpec("UNLOCK,2", "Unlock Separation (0° position)", Guard.NONE, rocket_only=True),
        CommandSpec("SV,0,30", "Set Rocket Door angle to 30°", Guard.NONE, rocket_only=True),
        CommandSpec("SV,0,60", "Set Rocket Door angle to 60°", Guard.NONE, rocket_only=True),
        CommandSpec("SV,0,90", "Set Rocket Door angle to 90°", Guard.NONE, rocket_only=True),
        CommandSpec("SV,1,30", "Set CanSat deployment angle to 30°", Guard.NONE, rocket_only=True),
        CommandSpec("SV,1,60", "Set CanSat deployment angle to 60°", Guard.NONE, rocket_only=True),
        CommandSpec("SV,1,90", "Set CanSat deployment angle to 90°", Guard.NONE, rocket_only=True),
        CommandSpec("SV,2,30", "Set Separation angle to 30°", Guard.NONE, rocket_only=True),
        CommandSpec("SV,2,60", "Set Separation angle to 60°", Guard.NONE, rocket_only=True),
        CommandSpec("SV,2,90", "Set Separation angle to 90°", Guard.NONE, rocket_only=True),
    ]
}


FIRMWARE_COMMAND_MAP: Dict[str, str] = {
    # Telemetry and sampling modes
    "SET_MODE_LIVE": "LIVE",
    "LIVE": "LIVE",
    "SET_MODE_HIBERNATE": "HIBERNATE",
    "HIBERNATE": "HIBERNATE",
    "ENABLE_TELEMETRY": "ENABLE",
    "ENABLE": "ENABLE",
    "DISABLE_TELEMETRY": "DISABLE",
    "DISABLE": "DISABLE",
    "REQUEST_STATUS": "ST",
    "ST": "ST",
    "GS": "GS",
    "REFRESH_PROCESSOR": "RF",
    "RF": "RF",

    # Calibration
    "CALIBRATE_ALL": "CAL",
    "CAL": "CAL",
    "CALIBRATE_ALTITUDE": "CALA",
    "CALA": "CALA",
    "CALIBRATE_IMU": "CALI",
    "CALI": "CALI",
    "CALIBRATE_GNSS": "CALA",
    "CALIBRATE_TEMPERATURE": "CALA",
    "CALIBRATE_PRESSURE": "CALA",

    # Actuator channels: 0 = Door, 1 = Payload (CanSat), 2 = Separation
    "DOOR:30": "SV,0,30",
    "DOOR:60": "SV,0,60",
    "DOOR:90": "SV,0,90",
    "LOCK:DOOR": "LOCK,0",
    "UNLOCK:DOOR": "UNLOCK,0",

    "CANSAT:30": "SV,1,30",
    "CANSAT:60": "SV,1,60",
    "CANSAT:90": "SV,1,90",
    "LOCK:CANSAT": "LOCK,1",
    "UNLOCK:CANSAT": "UNLOCK,1",

    "SEPARATION:30": "SV,2,30",
    "SEPARATION:60": "SV,2,60",
    "SEPARATION:90": "SV,2,90",
    "LOCK:SEPARATION": "LOCK,2",
    "UNLOCK:SEPARATION": "UNLOCK,2",

    # Sequence
    "LOCK:ALL": "LOCKALL",
    "LOCKALL": "LOCKALL",
    "TEST4": "LOCKALL",
    "RESET": "SEQRST",
    "SEQRST": "SEQRST",
    "TEST5": "MSEQ",
    "MSEQ": "MSEQ",
    "NEXT": "NSTEP",
    "NSTEP": "NSTEP",
    "TEST6": "FSEQ",
    "FSEQ": "FSEQ",

    # Frequencies
    "FREQ:ROCKET:ORIGINAL": "FQ,0",
    "FREQ:ROCKET:BACKUP": "FQ,1",
    "FREQ:CANSAT:ORIGINAL": "GF,0",
    "FREQ:CANSAT:BACKUP": "GF,1",
    "FQ,0": "FQ,0",
    "FQ,1": "FQ,1",
    "GF,0": "GF,0",
    "GF,1": "GF,1",
    "TX,0": "TX,0",
    "TX,1": "TX,1",

    # Bench tests and diagnostics
    "BITE": "BITE",
    "FPL": "FPL",
    "FPL,1": "FPL,1",
    "SEP": "SEP",
    "BZ,0": "BZ,0",
    "BZ,1": "BZ,1",
    "BZ": "BZ,1",
    "PG": "PG",
    "LK": "LK",
    "GR": "GR",
    "LR": "LR",
    "TEST1": "TEST1",
    "TEST2": "TEST2",
    "TEST3": "TEST3",
}


def to_firmware_command(command: str) -> str:
    """Map any UI / internal command name to the exact firmware command string."""
    clean = command.strip()
    if clean in FIRMWARE_COMMAND_MAP:
        return FIRMWARE_COMMAND_MAP[clean]
    # Check prefixes
    if clean.startswith("DOOR:"):
        ang = clean.split(":")[1]
        return f"SV,0,{ang}"
    if clean.startswith("CANSAT:"):
        ang = clean.split(":")[1]
        return f"SV,1,{ang}"
    if clean.startswith("SEPARATION:"):
        ang = clean.split(":")[1]
        return f"SV,2,{ang}"
    if clean == "LOCK:DOOR":
        return "LOCK,0"
    if clean == "LOCK:CANSAT":
        return "LOCK,1"
    if clean == "LOCK:SEPARATION":
        return "LOCK,2"
    if clean == "UNLOCK:DOOR":
        return "UNLOCK,0"
    if clean == "UNLOCK:CANSAT":
        return "UNLOCK,1"
    if clean == "UNLOCK:SEPARATION":
        return "UNLOCK,2"
    if clean in ("LOCK:ALL", "TEST4"):
        return "LOCKALL"
    if clean == "RESET":
        return "SEQRST"
    if clean == "TEST5":
        return "MSEQ"
    if clean == "NEXT":
        return "NSTEP"
    if clean == "TEST6":
        return "FSEQ"
    return clean


@dataclass(frozen=True)
class GuardResult:
    allowed: bool
    reason: str = ""
    #: True when the operator must confirm before this proceeds.
    needs_confirm: bool = False


def check_guard(spec: CommandSpec,
                state: Optional[FlightState],
                link: VehicleLinkState,
                simulator_mode: bool,
                usb_direct: bool = False) -> GuardResult:
    """Evaluate one command against current conditions.

    Returns a reason on refusal rather than a bare False, because a
    disabled control with no explanation is a control the operator will
    eventually work around.
    """
    # §10.8 — controls for a vehicle in LOST state are disabled rather
    # than queued to fire on reconnect. A queued ACT_SEPARATE that fires
    # the instant the link returns is a command nobody is watching for.
    if link is VehicleLinkState.LOST:
        return GuardResult(False, "Vehicle link lost — command not queued")

    if spec.guard is Guard.NOT_IN_SIMULATOR_MODE:
        if simulator_mode:
            # §2.5 — the app-side Simulator Mode and the vehicle-side
            # Flight Software Simulation are different things, and
            # ENTER_SIMULATION is disabled while in the former.
            return GuardResult(
                False,
                "Disabled in Simulator Mode — this command puts the "
                "vehicle's flight software into simulation, which is a "
                "different thing")
        if state is not None and state.ordinal > FlightState.PRE_LAUNCH.ordinal:
            return GuardResult(False, "Disallowed after PRE_LAUNCH")
        return GuardResult(True)

    if spec.guard is Guard.GROUND_ONLY:
        if state is None or state not in GROUND_STATES:
            # §10.2 — "zeroing the barometer mid-flight corrupts the
            # altitude datum for the rest of the mission".
            return GuardResult(
                False,
                f"Only available in BOOT or PRE_LAUNCH "
                f"(vehicle is in {state.value if state else 'unknown'})")
        return GuardResult(True)

    if spec.guard is Guard.RECOVERY_USB_ONLY:
        if not usb_direct:
            # §11.4 — a flight-length file over LoRa is tens of minutes
            # to hours, starving telemetry. Wrong link for the job.
            return GuardResult(
                False,
                "Needs a direct USB connection to the recovered CanSat — "
                "not available over the LoRa relay")
        if state is not FlightState.RECOVERY:
            return GuardResult(
                False, "Only available in RECOVERY — never during flight")
        return GuardResult(True)

    if spec.guard is Guard.CONFIRM_IN_FLIGHT:
        in_flight = state is not None and state not in GROUND_STATES
        return GuardResult(True, needs_confirm=in_flight)

    if spec.guard in (Guard.TWO_STEP, Guard.DESTRUCTIVE):
        return GuardResult(True, needs_confirm=True)

    return GuardResult(True)


# ─────────────────────────────────────────────────────────────────────────────
#  §10.7 — acknowledgment lifecycle
# ─────────────────────────────────────────────────────────────────────────────

class CommandStatus(Enum):
    SENT = "sent"
    ACKNOWLEDGED = "acknowledged"
    EXECUTED = "executed"
    REJECTED = "rejected"
    TIMED_OUT = "timed out"


#: §10.7 — how long before an unacknowledged command is called out.
#: Aligned with Ground Teensy's 8 retries @ 2s (16s + 2s margin).
ACK_TIMEOUT_S = 18.0


@dataclass
class CommandRecord:
    """One sent command and everything known about its fate."""
    sequence: int
    target: VehicleID
    command: str
    argument: str = ""
    sent_at: float = field(default_factory=time.monotonic)
    status: CommandStatus = CommandStatus.SENT
    #: The vehicle's own response text, when one arrives.
    response: str = ""
    #: §10.7 — a receiver forwarding confirmation is displayed as a
    #: separate, lesser signal. It is NOT an acknowledgment: "the ground
    #: receiver forwarding a command successfully says nothing about
    #: whether the vehicle heard it."
    receiver_forwarded: bool = False
    #: Set when this record is a resend of an earlier attempt.
    resend_of: Optional[int] = None

    @property
    def wire_text(self) -> str:
        return f"{self.command} {self.argument}".strip()

    @property
    def unacknowledged(self) -> bool:
        return self.status in (CommandStatus.SENT, CommandStatus.TIMED_OUT)

    def age(self, now: Optional[float] = None) -> float:
        return (now or time.monotonic()) - self.sent_at


BENCH_COMMANDS = (
    "TEST1", "TEST2", "TEST3", "TEST4", "TEST5", "NEXT", "TEST6", "RESET", "START", "CONFIRM",
    "DOOR:30", "DOOR:60", "DOOR:90", "LOCK:DOOR", "UNLOCK:DOOR",
    "CANSAT:30", "CANSAT:60", "CANSAT:90", "LOCK:CANSAT", "UNLOCK:CANSAT",
    "SEPARATION:30", "SEPARATION:60", "SEPARATION:90", "LOCK:SEPARATION", "UNLOCK:SEPARATION",
    "LOCK,0", "LOCK,1", "LOCK,2", "UNLOCK,0", "UNLOCK,1", "UNLOCK,2",
    "SV,0,30", "SV,0,60", "SV,0,90", "SV,1,30", "SV,1,60", "SV,1,90", "SV,2,30", "SV,2,60", "SV,2,90",
    "LOCK:ALL",
    "CAL", "CALA", "CALI", "FQ,0", "FQ,1", "GF,0", "GF,1", "TX,0", "TX,1", "FPL", "FPL,1",
    "FSEQ", "RF", "PG", "LK", "SD", "XD", "SEP", "BITE", "ST", "SV", "PW",
    "LOCK", "UNLOCK", "LOCKALL", "MSEQ", "NSTEP", "SEQRST", "BZ", "GR", "LR",
    "ENABLE", "DISABLE", "GS", "LIVE", "HIBERNATE",
)


class CommandCentre:
    """Tracks every command sent this session (§10.7).

    Holds no Qt objects and opens no ports — `_transmit` is injected, so
    the whole lifecycle including timeouts can be driven in a test.
    """

    def __init__(self, transmit: Callable[[VehicleID, str, int], None],
                 ack_timeout: float = ACK_TIMEOUT_S):
        self._transmit = transmit
        self.ack_timeout = ack_timeout
        self.records: List[CommandRecord] = []
        self._by_sequence: Dict[int, CommandRecord] = {}
        self._sequence = 0

        #: §10.4 — per-vehicle confirmed modes.
        self.modes: Dict[VehicleID, "VehicleModes"] = {
            v: VehicleModes() for v in VehicleID
        }

        #: Vehicle safety state (Rocket defaults to UNARMED)
        self.safety_states: Dict[VehicleID, RocketSafetyState] = {
            VehicleID.ROCKET: RocketSafetyState.UNARMED
        }

    @property
    def rocket_safety_state(self) -> RocketSafetyState:
        return self.safety_states.get(VehicleID.ROCKET, RocketSafetyState.UNARMED)

    def arm_rocket(self) -> None:
        self.safety_states[VehicleID.ROCKET] = RocketSafetyState.ARMED

    def disarm_rocket(self) -> None:
        self.safety_states[VehicleID.ROCKET] = RocketSafetyState.UNARMED

    def is_rocket_armed(self) -> bool:
        return self.rocket_safety_state is RocketSafetyState.ARMED

    # ── sending ──────────────────────────────────────────────────────────

    def send(self, target: VehicleID, command: str,
             argument: str = "", resend_of: Optional[int] = None
             ) -> CommandRecord:
        """Transmit one command. Only ever called from an operator action."""
        self._sequence += 1
        rec = CommandRecord(sequence=self._sequence, target=target,
                            command=command, argument=argument,
                            resend_of=resend_of)
        self.records.append(rec)
        self._by_sequence[rec.sequence] = rec

        # §10.4 — record the *request*, but do not touch the displayed
        # mode. That only moves on an acknowledgment.
        modes = self.modes[target]
        if command in ("SET_MODE_LIVE", "SET_MODE_HIBERNATE"):
            modes.request_sampling(command, rec.sequence)
        elif command in ("ENABLE_TELEMETRY", "DISABLE_TELEMETRY"):
            modes.request_telemetry(command, rec.sequence)

        self._transmit(target, rec.wire_text, rec.sequence)
        return rec

    def resend(self, sequence: int) -> Optional[CommandRecord]:
        """§10.7 — operator-initiated resend, never automatic.

        A new sequence number is issued rather than reusing the original.
        Reusing it would make the acknowledgment ambiguous — an ack for
        the first attempt would be indistinguishable from an ack for the
        resend, which is exactly how a mechanism gets fired twice.
        """
        original = self._by_sequence.get(sequence)
        if original is None:
            return None
        return self.send(original.target, original.command,
                         original.argument, resend_of=sequence)

    # ── responses ────────────────────────────────────────────────────────

    def acknowledge(self, sequence: int, response: str = "") -> Optional[CommandRecord]:
        """A vehicle acknowledgment. Only the vehicle may call this path."""
        rec = self._by_sequence.get(sequence)
        if rec is None:
            return None
        rec.status = CommandStatus.ACKNOWLEDGED
        rec.response = response
        self.modes[rec.target].confirm(rec.sequence, rec.command)
        return rec

    def mark_executed(self, sequence: int, response: str = "") -> Optional[CommandRecord]:
        rec = self._by_sequence.get(sequence)
        if rec is None:
            return None
        rec.status = CommandStatus.EXECUTED
        if response:
            rec.response = response
        return rec

    def reject(self, sequence: int, reason: str) -> Optional[CommandRecord]:
        rec = self._by_sequence.get(sequence)
        if rec is None:
            return None
        rec.status = CommandStatus.REJECTED
        rec.response = reason
        self.modes[rec.target].abandon(rec.sequence)
        return rec

    def note_receiver_forwarded(self, sequence: int) -> None:
        """§10.7 — a lesser signal than an acknowledgment, shown
        separately and never mistaken for one."""
        rec = self._by_sequence.get(sequence)
        if rec is not None:
            rec.receiver_forwarded = True

    def handle_ground_cmd(self, sequence: int, status: str,
                          info: str = "", raw_line: str = ""
                          ) -> Optional[CommandRecord]:
        """Handle a $G,CMD,<id>,<status>[,<info>] notification from Ground Teensy."""
        rec = self._by_sequence.get(sequence)
        if rec is None:
            if sequence == 0:
                for r in reversed(self.records):
                    if r.unacknowledged:
                        rec = r
                        break
            if rec is None:
                return None

        status_u = status.upper().strip()
        if status_u == "OK":
            return self.acknowledge(rec.sequence, info or raw_line or "OK")
        elif status_u == "QUEUED":
            rec.status = CommandStatus.SENT
            rec.response = "QUEUED"
        elif status_u == "SENT":
            rec.status = CommandStatus.SENT
            rec.receiver_forwarded = True
            rec.response = f"Uplink try {info}" if info else "SENT"
        elif status_u in ("REJECTED", "BADCMD"):
            return self.reject(rec.sequence, info or status_u)
        elif status_u == "FAILED":
            rec.status = CommandStatus.TIMED_OUT
            rec.response = f"Failed: {info}" if info else "Failed: no_reply"
            self.modes[rec.target].abandon(rec.sequence)
        elif status_u == "BUSY":
            return self.reject(rec.sequence, f"BUSY: {info}" if info else "BUSY")
        return rec

    # ── timeouts ─────────────────────────────────────────────────────────

    def poll_timeouts(self, now: Optional[float] = None) -> List[CommandRecord]:
        """Move stale SENT records to TIMED_OUT and return them.

        Timing out is *not* retrying. The record is flagged so the UI can
        offer an explicit Resend; nothing here re-transmits.
        """
        now = now or time.monotonic()
        out = []
        for rec in self.records:
            if rec.status is CommandStatus.SENT and rec.age(now) > self.ack_timeout:
                rec.status = CommandStatus.TIMED_OUT
                self.modes[rec.target].abandon(rec.sequence)
                out.append(rec)
        return out

    @property
    def unacknowledged(self) -> List[CommandRecord]:
        return [r for r in self.records if r.unacknowledged]


# ─────────────────────────────────────────────────────────────────────────────
#  §10.4 — sampling and transmission are orthogonal
# ─────────────────────────────────────────────────────────────────────────────

class ModeValue(Enum):
    UNKNOWN = "UNKNOWN"
    PENDING = "PENDING"          # requested, not yet acknowledged
    UNCONFIRMED = "UNCONFIRMED"  # requested, acknowledgment never came
    LIVE = "LIVE"
    HIBERNATE = "HIBERNATE"
    ENABLED = "ENABLED"
    DISABLED = "DISABLED"


class VehicleModes:
    """Confirmed sampling and telemetry modes for one vehicle (§10.4).

    Both start UNKNOWN rather than defaulting to a plausible value. The
    app has not heard from the vehicle yet, and showing LIVE/ENABLED
    before any acknowledgment is the optimistic flip §10.4 forbids.
    """

    def __init__(self) -> None:
        self.sampling = ModeValue.UNKNOWN
        self.telemetry = ModeValue.UNKNOWN
        self._pending: Dict[int, Tuple[str, str]] = {}   # seq -> (axis, cmd)

    def request_sampling(self, command: str, sequence: int) -> None:
        self.sampling = ModeValue.PENDING
        self._pending[sequence] = ("sampling", command)

    def request_telemetry(self, command: str, sequence: int) -> None:
        self.telemetry = ModeValue.PENDING
        self._pending[sequence] = ("telemetry", command)

    def confirm(self, sequence: int, command: str) -> None:
        axis_cmd = self._pending.pop(sequence, None)
        if axis_cmd is None:
            return
        axis, cmd = axis_cmd
        if axis == "sampling":
            self.sampling = (ModeValue.LIVE if cmd == "SET_MODE_LIVE"
                             else ModeValue.HIBERNATE)
        else:
            self.telemetry = (ModeValue.ENABLED if cmd == "ENABLE_TELEMETRY"
                              else ModeValue.DISABLED)

    def abandon(self, sequence: int) -> None:
        """Request timed out or was rejected — the mode is not what was
        asked for, and it is not what it was before either."""
        axis_cmd = self._pending.pop(sequence, None)
        if axis_cmd is None:
            return
        axis, _ = axis_cmd
        if axis == "sampling":
            self.sampling = ModeValue.UNCONFIRMED
        else:
            self.telemetry = ModeValue.UNCONFIRMED

    @property
    def dangerous(self) -> bool:
        """§10.4 — HIBERNATE + ENABLED.

        "Packets keep arriving so the link looks healthy, but the values
        are not live measurements." The one combination that is actively
        misleading rather than merely idle, and the reason this needs a
        persistent unmissable banner rather than a status line.
        """
        return (self.sampling is ModeValue.HIBERNATE
                and self.telemetry is ModeValue.ENABLED)


if __name__ == "__main__":
    print(f"ground_station.commands loaded successfully.")
    print(f"Total registered commands: {len(COMMANDS)}")
    for key, val in COMMANDS.items():
        print(f"  {key:25s} -> {val.description}")

