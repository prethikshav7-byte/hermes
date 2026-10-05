"""
ground_station/models.py
════════════════════════
Core data model. Deliberately free of PyQt and pyserial imports so the
whole §4–§5 layer is importable and unit-testable with no GUI and no
hardware attached (§11.7 asks this of the experiment parser; the same
discipline is worth having here).

WHAT CHANGED FROM THE PREVIOUS BUILD
────────────────────────────────────
The old model had two dataclasses with different field sets
(RocketTelemetry / CanSatTelemetry) and two state enums (RocketState with
7 members ending in IMPACT, CanSatState with 6 unrelated members).

§4.1 gives both vehicles an identical 21-field packet and §5.1/§9.6 give
them an identical 8-state machine, so those collapse into one record type
discriminated by VEHICLE_ID (§3.2). Per-vehicle separation is now a
property of the *stream* (§4.4), not of the record type.

⚠️ FIRMWARE MISMATCH — see §15 notes in the handover:
    The rocket .ino currently ends its state machine at IMPACT and has no
    LANDING or RECOVERY. §5.1 requires both, and §10.5/§11.5 gate the file
    transfer commands on RECOVERY, so those guards can never be satisfied
    until the firmware grows the two states. IMPACT is NOT accepted here
    as an alias — silently mapping it to LANDING would be exactly the kind
    of ground-side reinterpretation §1.5 forbids. It is rejected as an
    unknown state string (§4.5) and shows up in the rejected counter,
    which is the visible signal that firmware and spec disagree.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional


# ─────────────────────────────────────────────────────────────────────────────
#  §3.2 / §4.1 field 2 — routing key
# ─────────────────────────────────────────────────────────────────────────────

class VehicleID(Enum):
    ROCKET = "ROCKET"
    CANSAT = "CANSAT"


class RocketSafetyState(Enum):
    UNARMED = "UNARMED"
    ARMED = "ARMED"


# ─────────────────────────────────────────────────────────────────────────────
#  §5.1 — eight states, same set for both vehicles
# ─────────────────────────────────────────────────────────────────────────────

class FlightState(Enum):
    BOOT = "BOOT"
    PRE_LAUNCH = "PRE_LAUNCH"
    BOOST = "BOOST"
    COAST = "COAST"
    APOGEE = "APOGEE"
    DESCENT = "DESCENT"
    LANDING = "LANDING"
    RECOVERY = "RECOVERY"

    @property
    def ordinal(self) -> int:
        """Position in the nominal forward sequence.

        Used *only* to detect and flag a backward transition for display
        (§5.2) and to spot a skipped state for the inferred-APOGEE rule
        (§5.6). The app never enforces or corrects transitions — a
        vehicle that reports a regression is reporting real information
        about its flight software, and §5.2 requires it be shown, not
        fixed.
        """
        return _STATE_ORDER[self]


_STATE_ORDER: Dict["FlightState", int] = {
    s: i for i, s in enumerate([
        FlightState.BOOT,
        FlightState.PRE_LAUNCH,
        FlightState.BOOST,
        FlightState.COAST,
        FlightState.APOGEE,
        FlightState.DESCENT,
        FlightState.LANDING,
        FlightState.RECOVERY,
    ])
}

#: States during which calibration and REFRESH_PROCESSOR are permitted
#: (§10.2, §10.3). Zeroing the barometer mid-flight corrupts the altitude
#: datum for the remainder of the mission.
GROUND_STATES = frozenset({FlightState.BOOT, FlightState.PRE_LAUNCH})


# ─────────────────────────────────────────────────────────────────────────────
#  §9 — validation states
# ─────────────────────────────────────────────────────────────────────────────

class ValidationStatus(Enum):
    VALID = "VALID"
    INVALID_FORMAT = "INVALID_FORMAT"
    INVALID_FIELD_COUNT = "INVALID_FIELD_COUNT"
    INVALID_NUMERIC_FIELD = "INVALID_NUMERIC_FIELD"
    INVALID_STATE = "INVALID_STATE"
    INVALID_VEHICLE = "INVALID_VEHICLE"
    CHECKSUM_FAILED = "CHECKSUM_FAILED"
    DUPLICATE = "DUPLICATE"
    OUT_OF_ORDER = "OUT_OF_ORDER"
    MISSING_PACKET_DETECTED = "MISSING_PACKET_DETECTED"
    RF_ERROR = "RF_ERROR"
    TIMEOUT = "TIMEOUT"


# ─────────────────────────────────────────────────────────────────────────────
#  §4.1 — the decoded record
# ─────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class TelemetryPacket:
    """One decoded telemetry packet — 20 vehicle-measured values plus the
    metadata the app attaches on receipt.

    Frozen because §13.5 requires no shared mutable state downstream of
    the demultiplexer. Two dashboards holding the same packet cannot
    perturb each other.
    """
    # 1–5 identity and timing
    team_id: Optional[Any] = None
    vehicle_id: VehicleID = VehicleID.ROCKET
    mission_time: float = 0.0    # s, monotonic
    packet_count: int = 0        # per-vehicle, never reset (§4.1 field 4)
    state: Optional[FlightState] = None

    # 6–9 barometric and power
    altitude: float = 0.0        # m AGL, zeroed at the pad
    pressure: float = 0.0        # Pa
    temperature: float = 0.0     # °C
    battery_voltage: Optional[float] = None  # V

    # 10–14 GNSS
    gnss_time: Optional[str] = None          # "hh:mm:ss.ss" UTC, kept as text
    gnss_latitude: float = 0.0   # signed decimal degrees
    gnss_longitude: float = 0.0
    gnss_altitude: float = 0.0   # m MSL — different datum from `altitude`
    gnss_satellites: int = 0

    # 15–20 IMU
    accel_x: Optional[float] = None
    accel_y: Optional[float] = None
    accel_z: Optional[float] = None
    gyro_x: Optional[float] = None
    gyro_y: Optional[float] = None
    gyro_z: Optional[float] = None
    gyro_spin_rate: Optional[float] = None
    accelerometer: Optional[float] = None

    # Live rocket telemetry fields
    flight_state_code: Optional[int] = None
    velocity: Optional[float] = None
    vehicle_code: Optional[int] = None
    raw_accelerometer: Optional[str] = None
    checksum: Optional[str] = None
    validation_status: ValidationStatus = ValidationStatus.VALID

    # ── metadata attached on the ground, not measured by the vehicle ──────
    #: Fields beyond 21, e.g. receiver-appended RSSI/SNR (§3.4, §4.7).
    extras: Dict[str, str] = field(default_factory=dict)
    #: Host-clock receive time, for the raw log (§12.1). Not mission time.
    received_at: float = 0.0
    #: The verbatim line, so the log can record exactly what arrived (§12.1).
    raw: str = ""

    # ── convenience accessors ─────────────────────────────────────────────
    @property
    def timestamp(self) -> float:
        return self.mission_time

    @property
    def flight_state(self) -> str:
        return self.state.value if self.state is not None else "N/A"

    @property
    def voltage(self) -> Optional[float]:
        return self.battery_voltage

    @property
    def vehicle(self) -> str:
        return self.vehicle_id.value if self.vehicle_id is not None else "ROCKET"

    @property
    def latitude(self) -> float:
        return self.gnss_latitude

    @property
    def longitude(self) -> float:
        return self.gnss_longitude

    @property
    def satellites(self) -> int:
        return self.gnss_satellites

    @property
    def accel_magnitude(self) -> Optional[float]:
        if self.accelerometer is not None:
            return self.accelerometer
        if self.accel_x is not None and self.accel_y is not None and self.accel_z is not None:
            return (self.accel_x**2 + self.accel_y**2 + self.accel_z**2)**0.5
        return None

    @property
    def rssi(self) -> Optional[float]:
        """Per-vehicle RSSI in dBm if the receiver appended it (§3.4)."""
        return _maybe_float(self.extras.get("RSSI") or self.extras.get("rssi"))

    @property
    def snr(self) -> Optional[float]:
        """Per-vehicle SNR in dB if the receiver appended it (§3.4)."""
        return _maybe_float(self.extras.get("SNR") or self.extras.get("snr"))

    @property
    def gnss_fix(self) -> int:
        fix_str = self.extras.get("FIX") or self.extras.get("gnss_fix")
        if fix_str is not None:
            try:
                return int(float(fix_str))
            except ValueError:
                pass
        return 1 if self.gnss_satellites >= 6 else 0

    @property
    def is_simulation(self) -> bool:
        """True if packet carries SIM=1 simulation telemetry flag."""
        sim_str = self.extras.get("SIM") or self.extras.get("sim")
        return sim_str == "1"


def _maybe_float(v: Optional[str]) -> Optional[float]:
    if v is None:
        return None
    try:
        return float(v)
    except ValueError:
        return None


# ─────────────────────────────────────────────────────────────────────────────
#  §3.4 — receiver heartbeat
# ─────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class ReceiverHeartbeat:
    """Emitted by the ground receiver at ≥1 Hz even when both vehicles are
    silent (§3.4). Its absence is what distinguishes "the receiver died"
    from "both vehicles went quiet" — a distinction that matters most at
    exactly the moment it is hardest to make (§3.5).
    """
    uptime: float                       # s since receiver boot
    counts: Dict[VehicleID, int]        # per-channel received counts
    received_at: float = 0.0
    raw: str = ""


# ─────────────────────────────────────────────────────────────────────────────
#  §4.5 / §4.6 — rejection taxonomy
# ─────────────────────────────────────────────────────────────────────────────

class RejectReason(Enum):
    """Why a line was not displayed.

    §4.5 (malformed) and §4.6 (duplicate/regression) are counted
    separately because they mean different things: malformed is a
    corruption or firmware-format problem, duplicate/regression is a
    relay or ordering problem. One rejected-packet number that mixes them
    tells the operator nothing actionable.
    """
    FIELD_COUNT = "wrong field count"
    NON_NUMERIC = "non-numeric value in numeric field"
    UNKNOWN_STATE = "unknown flight state string"
    UNKNOWN_VEHICLE = "unknown vehicle id"
    CHECKSUM = "checksum mismatch"
    DUPLICATE = "packet count not advancing"
    TIME_REGRESSION = "mission time went backwards"
    INVALID_FORMAT = "invalid packet format"
    RF_ERROR = "rf error"
    TIMEOUT = "timeout"

    @property
    def is_malformed(self) -> bool:
        """True for §4.5 reasons, False for §4.6 reasons."""
        return self not in (RejectReason.DUPLICATE,
                            RejectReason.TIME_REGRESSION)


@dataclass(frozen=True)
class RejectedLine:
    """A line that was logged and dropped. Never displayed, never
    zero-filled (§4.5) — a zeroed altitude is indistinguishable from a
    real one on a graph.
    """
    reason: RejectReason
    detail: str
    raw: str
    received_at: float = 0.0
    #: Present when the line decoded far enough to identify its vehicle.
    vehicle_id: Optional[VehicleID] = None


# ─────────────────────────────────────────────────────────────────────────────
#  §5.5 / §5.6 / §5.7 — state timeline entries
# ─────────────────────────────────────────────────────────────────────────────

class StateObservation(Enum):
    """How a timeline entry came to be known."""
    OBSERVED = "observed"
    #: §5.6 — the state was never received; its occurrence is deduced
    #: retrospectively from a jump in the observed sequence. Display only,
    #: never feeds a command (§1.3). This is the *only* place the app
    #: draws a state conclusion.
    INFERRED = "inferred, not received"
    #: §5.7 — first state seen after a REFRESH_PROCESSOR, restored from
    #: the vehicle's flash rather than reached by a live transition.
    RESTORED = "restored from flash"


@dataclass(frozen=True)
class StateEntry:
    """One row of the per-vehicle state timeline (§5.5) — the primary
    post-flight debugging artefact."""
    state: FlightState
    mission_time: float
    observation: StateObservation = StateObservation.OBSERVED
    #: §5.2 — set when this transition moved backwards through the
    #: nominal sequence. Displayed with a visible anomaly flag; the app
    #: does not correct the vehicle.
    backward: bool = False
    #: §10.3 — worst-case timestamp error in seconds for a RESTORED entry,
    #: so the operator knows the timeline's precision after a refresh.
    time_uncertainty: Optional[float] = None


# ─────────────────────────────────────────────────────────────────────────────
#  §3.6 — three independent health facts
# ─────────────────────────────────────────────────────────────────────────────

class SourceState(Enum):
    """USB source health. Says nothing about either radio link."""
    DISCONNECTED = "DISCONNECTED"
    CONNECTING = "CONNECTING"
    CONNECTED = "CONNECTED"
    NO_DATA = "NO DATA"          # port open, no line in > 3 s
    LINK_ERROR = "LINK ERROR"


class ReceiverState(Enum):
    """Ground receiver health, distinct from either vehicle's (§3.5)."""
    ALIVE = "ALIVE"
    NO_HEARTBEAT = "NO HEARTBEAT"


class VehicleLinkState(Enum):
    """Per-vehicle radio health (§3.6)."""
    RECEIVING = "RECEIVING"
    STALE = "STALE"              # no packet in > 2 s
    LOST = "LOST"                # no packet in > 10 s


#: §3.6 thresholds, seconds.
NO_DATA_AFTER_S = 3.0
VEHICLE_STALE_AFTER_S = 2.0
VEHICLE_LOST_AFTER_S = 10.0
#: §3.5 — receiver heartbeat is ≥1 Hz, so allow a little slack.
HEARTBEAT_TIMEOUT_S = 3.0
