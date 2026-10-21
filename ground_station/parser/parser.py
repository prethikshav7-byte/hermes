"""
ground_station/parser.py
════════════════════════
Demultiplexing (§3.2), per-vehicle stream state (§4.4), duplicate and
regression rejection (§4.6), loss tiering (§7.5.2), and link quality
(§3.7).

Qt-free and hardware-free on purpose — every rule in here is testable
without opening a port or a window.

DESIGN NOTE — why loss tiering lives here and not in the graph widget:
§7.5.2 classifies loss by *consecutive packets missing*, which is a
property of PACKET_COUNT continuity, not of anything a plot knows. The
graph is handed a tier and renders it (§7.5.3–7.5.9); it does not derive
one. That also means the tier counts in the health strip (§6.5) and the
tier a graph draws can never disagree, because there is one computation.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Deque, Dict, List, Optional, Tuple
from collections import deque
import time

from ..codec import (
    CodecError, CsvCodec, DecodedFrame, PacketCodec,
    PREFIX_FILE, PREFIX_HEARTBEAT, PREFIX_TELEMETRY, PREFIX_GROUND, decode_heartbeat,
)
from ..models import (
    ReceiverHeartbeat, RejectReason, RejectedLine, TelemetryPacket,
    VehicleID, VehicleLinkState, VEHICLE_LOST_AFTER_S, VEHICLE_STALE_AFTER_S,
)


# ─────────────────────────────────────────────────────────────────────────────
#  §7.5.2 — loss tiers
# ─────────────────────────────────────────────────────────────────────────────

class LossTier(Enum):
    NONE = "none"           # contiguous, nothing missing
    DROPOUT = "dropout"     # 1–4 missing: draw straight through, no annotation
    GAP = "gap"             # 5–19 missing: break the line, no interpolation
    OUTAGE = "outage"       # ≥20 missing: full reconciliation (§7.5.3+)


@dataclass
class LossThresholds:
    """§7.5.2 — "thresholds are configurable; the right values depend on
    the link quality actually achieved (§3.11)". Defaults are the spec's.

    Expressed in packets rather than seconds because packet count is the
    quantity actually measured; the spec's second figures are just the
    packet counts at the nominal 10 Hz, and would be wrong the moment
    §3.11 option 3 drops the downlink to 3–5 Hz.
    """
    dropout_max: int = 5        # 1..5 -> DROPOUT (interpolated across)
    gap_max: int = 19           # 6..19 -> GAP, >=20 -> OUTAGE

    def classify(self, missing: int) -> LossTier:
        if missing <= 0:
            return LossTier.NONE
        if missing <= self.dropout_max:
            return LossTier.DROPOUT
        if missing <= self.gap_max:
            return LossTier.GAP
        return LossTier.OUTAGE


# ─────────────────────────────────────────────────────────────────────────────
#  Counters
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class StreamCounters:
    """Everything the health strip (§6.5) and rejected-packet display
    (§4.5) need. Monotonic for the whole mission — RESET_COUNTERS was
    removed (§10.5)."""
    accepted: int = 0
    #: §4.5 reasons, kept apart from §4.6 reasons — see RejectReason.is_malformed.
    malformed: int = 0
    checksum_failed: int = 0
    duplicates: int = 0
    time_regressions: int = 0
    rf_errors: int = 0
    #: §7.5.2 tier counts.
    dropouts: int = 0
    gaps: int = 0
    outages: int = 0
    #: Total packets the vehicle says it sent but we never saw.
    missing_total: int = 0

    @property
    def rejected_total(self) -> int:
        return (self.malformed + self.checksum_failed
                + self.duplicates + self.time_regressions + self.rf_errors)

    @property
    def total_received(self) -> int:
        return self.accepted + self.rejected_total


@dataclass(frozen=True)
class IngestResult:
    """What happened to one line. `packet` is None iff it was rejected."""
    packet: Optional[TelemetryPacket]
    rejected: Optional[RejectedLine] = None
    tier: LossTier = LossTier.NONE
    missing: int = 0

    @property
    def accepted(self) -> bool:
        return self.packet is not None


# ─────────────────────────────────────────────────────────────────────────────
#  Per-vehicle stream
# ─────────────────────────────────────────────────────────────────────────────

class VehicleStream:
    """All mutable state for one vehicle (§4.4, §13.5).

    Two instances share nothing. A fault on one channel cannot disturb the
    other's display, counters, or logging (§3.8).
    """

    #: §3.7 — window over which packet-success percentage is computed.
    #: 200 packets is 20 s at 10 Hz: long enough to be stable, short
    #: enough that a link degrading during descent shows up while the
    #: operator can still act on it.
    SUCCESS_WINDOW = 200

    def __init__(self, vehicle_id: VehicleID,
                 thresholds: Optional[LossThresholds] = None):
        self.vehicle_id = vehicle_id
        self.thresholds = thresholds or LossThresholds()
        self.counters = StreamCounters()

        self.last_packet: Optional[TelemetryPacket] = None
        self._last_packet_count: Optional[int] = None
        self._last_mission_time: Optional[float] = None
        self._last_received_at: float = 0.0
        self._first_received_at: Optional[float] = None

        # Rolling window of (expected, received) for §3.7.
        self._window: Deque[Tuple[int, int]] = deque(maxlen=self.SUCCESS_WINDOW)

        # §4.7 — unknown trailing field keys already reported, so they are
        # logged once per session rather than once per packet.
        self._reported_unknown_extras: set[str] = set()

        # §10.3 — set by the Command dashboard immediately before a
        # REFRESH_PROCESSOR so the next packet can be continuity-checked
        # and marked "restored from flash" (§5.7).
        self.awaiting_refresh: bool = False
        self.pre_refresh_mission_time: Optional[float] = None

    def reset(self) -> None:
        """Reset mutable stream tracking and counters for a fresh mission/simulation session."""
        self.counters = StreamCounters()
        self.last_packet = None
        self._last_packet_count = None
        self._last_mission_time = None
        self._last_received_at = 0.0
        self._first_received_at = None
        self._window.clear()
        self._reported_unknown_extras.clear()
        self.awaiting_refresh = False
        self.pre_refresh_mission_time = None

    # ── ingest ───────────────────────────────────────────────────────────

    def ingest(self, packet: TelemetryPacket) -> IngestResult:
        """Apply §4.6 and §7.5.2 to one already-decoded packet."""
        # §4.6 — duplicates and regressions, counted separately from
        # malformed. Checked before anything else touches stream state.
        if self._last_packet_count is not None:
            if packet.packet_count <= self._last_packet_count:
                if packet.packet_count == self._last_packet_count:
                    self.counters.duplicates += 1
                    reason = RejectReason.DUPLICATE
                    detail = f"duplicate packet_count {packet.packet_count}"
                else:
                    self.counters.time_regressions += 1
                    reason = RejectReason.TIME_REGRESSION
                    detail = (f"out-of-order packet_count {packet.packet_count} < last "
                              f"accepted {self._last_packet_count}")

                return IngestResult(
                    packet=None,
                    rejected=RejectedLine(
                        reason=reason,
                        detail=detail,
                        raw=packet.raw,
                        received_at=packet.received_at,
                        vehicle_id=self.vehicle_id,
                    ),
                )

        if self._last_mission_time is not None and packet.mission_time is not None:
            if packet.mission_time < self._last_mission_time:
                self.counters.time_regressions += 1
                return IngestResult(
                    packet=None,
                    rejected=RejectedLine(
                        reason=RejectReason.TIME_REGRESSION,
                        detail=(f"mission_time {packet.mission_time:.3f} < last "
                                f"accepted {self._last_mission_time:.3f}"),
                        raw=packet.raw,
                        received_at=packet.received_at,
                        vehicle_id=self.vehicle_id,
                    ),
                )

        # ── §7.5.2 loss tiering ──────────────────────────────────────────
        if self._last_packet_count is None:
            missing = 0          # first packet of the session
        else:
            missing = packet.packet_count - self._last_packet_count - 1

        tier = self.thresholds.classify(missing)
        if tier is LossTier.DROPOUT:
            self.counters.dropouts += 1
        elif tier is LossTier.GAP:
            self.counters.gaps += 1
        elif tier is LossTier.OUTAGE:
            self.counters.outages += 1
        self.counters.missing_total += max(0, missing)

        self._window.append((max(1, missing + 1), 1))
        self._last_packet_count = packet.packet_count
        self._last_mission_time = packet.mission_time
        self._last_received_at = packet.received_at
        self.last_packet = packet
        self.counters.accepted += 1

        return IngestResult(packet=packet, tier=tier, missing=missing)

    def note_rejection(self, rejected: RejectedLine) -> None:
        """Count a line that failed before it became a packet (§4.5)."""
        if rejected.reason is RejectReason.CHECKSUM:
            self.counters.checksum_failed += 1
        elif rejected.reason is RejectReason.RF_ERROR:
            self.counters.rf_errors += 1
        elif rejected.reason.is_malformed:
            self.counters.malformed += 1

    def should_report_unknown_extra(self, key: str) -> bool:
        """§4.7 — True the first time this session, False after."""
        if key in self._reported_unknown_extras:
            return False
        self._reported_unknown_extras.add(key)
        return True

    # ── §3.7 link quality ────────────────────────────────────────────────

    @property
    def packet_success(self) -> Optional[float]:
        """Rolling success percentage from PACKET_COUNT continuity.

        None until there is enough history to mean anything — showing
        "100%" off a single packet is worse than showing nothing, because
        it is read as a healthy link.
        """
        if len(self._window) < 2:
            return None
        expected = sum(e for e, _ in self._window)
        received = sum(r for _, r in self._window)
        if expected <= 0:
            return None
        return 100.0 * received / expected

    # ── §3.6 link state ──────────────────────────────────────────────────

    def link_state(self, now: Optional[float] = None) -> VehicleLinkState:
        """RECEIVING / STALE / LOST from wall-clock silence.

        Wall clock, not mission time: mission time only advances when a
        packet arrives, so a mission-time test could never detect silence.
        """
        if self.last_packet is None:
            return VehicleLinkState.LOST
        now = time.monotonic() if now is None else now
        silent = now - self._last_received_at
        if silent > VEHICLE_LOST_AFTER_S:
            return VehicleLinkState.LOST
        if silent > VEHICLE_STALE_AFTER_S:
            return VehicleLinkState.STALE
        return VehicleLinkState.RECEIVING

    @property
    def seconds_since_packet(self) -> Optional[float]:
        if self.last_packet is None:
            return None
        return time.monotonic() - self._last_received_at


# ─────────────────────────────────────────────────────────────────────────────
#  Demultiplexer
# ─────────────────────────────────────────────────────────────────────────────

class TelemetryDemux:
    """Turns raw lines from the single USB source into per-vehicle results.

    §3.2: routing is by VEHICLE_ID and only by VEHICLE_ID. There is no
    port-based routing path here and none should be added — with one
    multiplexed source it is not merely the preferred method, it is the
    only possible one.
    """

    def __init__(self, codec: Optional[PacketCodec] = None,
                 thresholds: Optional[LossThresholds] = None):
        self.codec = codec or CsvCodec()
        self.streams: Dict[VehicleID, VehicleStream] = {
            v: VehicleStream(v, thresholds) for v in VehicleID
        }

        self.last_heartbeat: Optional[ReceiverHeartbeat] = None
        self._last_heartbeat_at: float = 0.0

        self.ignored_lines: int = 0
        self.on_file_frame: Optional[Callable[[str], None]] = None
        self.on_command_ack: Optional[Callable[[str, str, Dict[str, Any], str], None]] = None
        self.on_ground_cmd: Optional[Callable[[int, str, str, str], None]] = None
        self.on_servo_report: Optional[Callable[[int, float, float, float, int, int, str], None]] = None
        self.on_lora_status: Optional[Callable[[str, str], None]] = None
        self.notices: List[str] = []
        self._latest_rssi: Optional[float] = None
        self._latest_snr: Optional[float] = None

    def reset(self, vid: Optional[VehicleID] = None) -> None:
        """Reset demux streams and state."""
        if vid is not None:
            if vid in self.streams:
                self.streams[vid].reset()
        else:
            for stream in self.streams.values():
                stream.reset()
            self.last_heartbeat = None
            self._last_heartbeat_at = 0.0
            self.ignored_lines = 0
            self.notices.clear()

    def feed(self, line: str, received_at: Optional[float] = None
             ) -> Optional[IngestResult]:
        """Process one line. Returns an IngestResult for telemetry lines
        (accepted or rejected), None for heartbeats, file frames, and
        lines belonging to another parser."""
        received_at = time.monotonic() if received_at is None else received_at
        text = line.strip()
        if not text:
            return None

        # Rocket replies ($H,ACK / $H,NACK / $H,SV)
        if text.startswith(PREFIX_HEARTBEAT + ","):
            if text.startswith(PREFIX_HEARTBEAT + ",ACK,") or text.startswith(PREFIX_HEARTBEAT + ",NACK,"):
                self._feed_command_ack(text, received_at)
                return None
            if text.startswith(PREFIX_HEARTBEAT + ",SV,"):
                self._feed_servo_report(text, received_at)
                return None
            self._feed_heartbeat(text, received_at)
            return None

        if text.startswith(PREFIX_FILE + ","):
            if self.on_file_frame is not None:
                self.on_file_frame(text)
            return None

        # Ground station command status ($G,CMD,<id>,<status>[,<info>])
        if text.startswith(PREFIX_GROUND + ",CMD,"):
            self._feed_ground_cmd(text, received_at)
            return None

        # Ground station periodic status / config lines
        if text.startswith(PREFIX_GROUND + ",STATUS,"):
            self._last_heartbeat_at = received_at
            self.notices.append(f"HW: {text}")
            return None

        if text.startswith(PREFIX_GROUND + ",LORA,"):
            parts = text.split(",")
            lora_state = parts[2].upper() if len(parts) > 2 else ""
            if self.on_lora_status is not None:
                self.on_lora_status(lora_state, text)
            self.notices.append(f"LoRa radio: {lora_state}")
            return None

        if text.startswith(PREFIX_GROUND + ",READY,"):
            self.notices.append(f"Ground ready: {text}")
            return None

        # Parse ground station receiver diagnostic RSSI / SNR lines
        if text.upper().startswith("RSSI") and ":" in text:
            self._parse_rssi_line(text)
            return None
        if text.upper().startswith("SNR") and ":" in text:
            self._parse_snr_line(text)
            return None

        # Parse LoRa receiver error line (§4)
        if text.upper().startswith("LORA RX ERROR"):
            self._parse_lora_rx_error(text, received_at)
            return None

        # Route ground station and flight computer status/config lines ($G, GPS, LORA, BITE, $B)
        if (text.startswith(PREFIX_GROUND + ",") or text.startswith("GPS,") or
                text.startswith("LORA,") or text.startswith("BITE,") or text.startswith("$B,")):
            self.notices.append(f"HW: {text}")
            return None

        if not (text.startswith(PREFIX_TELEMETRY + ",") or text.startswith("PKT=")):
            self.ignored_lines += 1
            return None

        return self._feed_telemetry(text, received_at)

    def _feed_ground_cmd(self, text: str, received_at: float) -> None:
        parts = [p.strip() for p in text.split(",")]
        # Format: $G,CMD,<id>,<status>[,<info>]
        cmd_id = 0
        if len(parts) > 2:
            try:
                cmd_id = int(parts[2])
            except ValueError:
                cmd_id = 0
        status = parts[3].upper() if len(parts) > 3 else ""
        info = parts[4] if len(parts) > 4 else ""
        if self.on_ground_cmd is not None:
            self.on_ground_cmd(cmd_id, status, info, text)
        msg = f"Ground CMD #{cmd_id}: {status}" + (f" ({info})" if info else "")
        self.notices.append(msg)

    def _feed_servo_report(self, text: str, received_at: float) -> None:
        # $H,SV,<ch>,<deg>,<estimated_deg>,<remaining_deg>,<seq_mode>,<seq_step>
        parts = [p.strip() for p in text.split(",")]
        if len(parts) >= 8:
            try:
                ch = int(parts[2])
                deg = float(parts[3])
                est = float(parts[4])
                rem = float(parts[5])
                mode = int(parts[6])
                step = int(parts[7])
                if self.on_servo_report is not None:
                    self.on_servo_report(ch, deg, est, rem, mode, step, text)
            except (ValueError, IndexError):
                pass
        self.notices.append(f"Servo report: {text}")

    def _feed_command_ack(self, text: str, received_at: float) -> None:
        parts = [p.strip() for p in text.split(",")]
        status = parts[1].upper() if len(parts) > 1 else ""
        cmd = parts[2].upper() if len(parts) > 2 else ""
        details: Dict[str, Any] = {}
        if status == "ACK":
            if cmd == "CALA":
                # $H,ACK,CALA,<n>,<p0_Pa>,<T0_0.1C>,<gnss_ground_0.1m>
                if len(parts) >= 7:
                    details["n"] = parts[3]
                    details["p0_Pa"] = parts[4]
                    try:
                        details["T0"] = float(parts[5]) / 10.0
                    except ValueError:
                        details["T0"] = parts[5]
                    try:
                        details["gnss_ground"] = float(parts[6]) / 10.0
                    except ValueError:
                        details["gnss_ground"] = parts[6]
            elif cmd == "CALI":
                # $H,ACK,CALI,<n>,<axis>,<sign>,<bias_mm/s2>,<sin_elev_1e-4>,<spin_axis>,<spin_sign>,<bno_ok>,<bno_tilt_0.1deg>
                if len(parts) >= 12:
                    details["n"] = parts[3]
                    details["axis"] = parts[4]
                    details["sign"] = parts[5]
                    details["bias_mm_s2"] = parts[6]
                    details["sin_elev"] = parts[7]
                    details["spin_axis"] = parts[8]
                    details["spin_sign"] = parts[9]
                    details["bno_ok"] = parts[10]
                    try:
                        details["bno_tilt"] = float(parts[11]) / 10.0
                    except ValueError:
                        details["bno_tilt"] = parts[11]
            elif cmd == "CAL":
                # $H,ACK,CAL,<n>,<alt result>,<imu result>
                if len(parts) >= 6:
                    details["n"] = parts[3]
                    details["alt_result"] = parts[4]
                    details["imu_result"] = parts[5]
            elif cmd in ("LOCK", "UNLOCK", "SV", "PW"):
                # $H,ACK,LOCK,<ch>,<deg> or $H,ACK,UNLOCK,<ch>,<deg> or $H,ACK,SV,<ch>,<deg>
                if len(parts) >= 5:
                    try:
                        details["ch"] = int(parts[3])
                        details["deg"] = float(parts[4])
                    except ValueError:
                        pass
            elif cmd == "NSTEP":
                # $H,ACK,NSTEP,<step>,<ch>
                if len(parts) >= 5:
                    try:
                        details["step"] = int(parts[3])
                        details["ch"] = int(parts[4])
                    except ValueError:
                        pass
            elif cmd == "ST":
                # $H,ACK,ST,<state>,<token>,<flags>,<deg0>,<deg1>,<deg2>,<est0>,<est1>,<est2>,<mode>,<step>
                if len(parts) >= 14:
                    try:
                        details["state"] = int(parts[3])
                        details["token"] = int(parts[4])
                        details["flags"] = parts[5]
                        details["deg0"] = float(parts[6])
                        details["deg1"] = float(parts[7])
                        details["deg2"] = float(parts[8])
                        details["est0"] = float(parts[9])
                        details["est1"] = float(parts[10])
                        details["est2"] = float(parts[11])
                        details["seq_mode"] = int(parts[12])
                        details["seq_step"] = int(parts[13])
                    except (ValueError, IndexError):
                        pass
            elif cmd == "FQ":
                if len(parts) >= 4:
                    try:
                        details["ch"] = int(parts[3])
                    except ValueError:
                        pass
            elif cmd == "TX":
                if len(parts) >= 4:
                    try:
                        details["tx"] = int(parts[3])
                    except ValueError:
                        pass
            elif cmd == "PG":
                if len(parts) >= 6:
                    details["rssi"] = parts[3]
                    details["snr"] = parts[4]
                    details["ch"] = parts[5]
        elif status == "NACK":
            # $H,NACK,<cmd>,<reason>
            if len(parts) >= 4:
                details["reason"] = parts[3]

        if self.on_command_ack is not None:
            self.on_command_ack(cmd, status, details, text)
        self.notices.append(f"Hardware {status}: {text}")

    def _parse_rssi_line(self, text: str) -> None:
        try:
            val_part = text.split(":", 1)[1].strip()
            num_str = val_part.replace("dBm", "").replace("dbm", "").strip()
            self._latest_rssi = float(num_str)
        except Exception:
            pass

    def _parse_snr_line(self, text: str) -> None:
        try:
            val_part = text.split(":", 1)[1].strip()
            num_str = val_part.replace("dB", "").replace("db", "").strip()
            self._latest_snr = float(num_str)
        except Exception:
            pass

    def _parse_lora_rx_error(self, text: str, received_at: float) -> None:
        for s in self.streams.values():
            s.counters.rf_errors += 1
        self.notices.append(f"LoRa RX Error: {text}")

    # ── internals ────────────────────────────────────────────────────────

    def _feed_telemetry(self, text: str, received_at: float) -> IngestResult:
        try:
            decoded: DecodedFrame = self.codec.decode(text)
        except CodecError as e:
            # The line failed before we could trust its VEHICLE_ID, so it
            # cannot be attributed to a stream. Attempt a best-effort read
            # of field 2 purely so the counter lands on the right vehicle;
            # if that is not credible either, the rejection is unattributed
            # and counted globally.
            vehicle = self._sniff_vehicle(text)
            rejected = RejectedLine(reason=e.reason, detail=e.detail,
                                    raw=text, received_at=received_at,
                                    vehicle_id=vehicle)
            if vehicle is not None:
                self.streams[vehicle].note_rejection(rejected)
            return IngestResult(packet=None, rejected=rejected)

        vehicle: VehicleID = decoded.values["vehicle_id"]
        stream = self.streams[vehicle]

        for key in decoded.unknown_extras:
            if stream.should_report_unknown_extra(key):
                self.notices.append(
                    f"{vehicle.value}: unrecognised trailing field {key!r} "
                    f"(§4.7 — accepted and ignored, reported once per session)"
                )

        extras = dict(decoded.extras)
        if self._latest_rssi is not None:
            extras.setdefault("rssi", f"{self._latest_rssi:.1f}")
        if self._latest_snr is not None:
            extras.setdefault("snr", f"{self._latest_snr:.2f}")

        if text.startswith("PKT="):
            if stream._first_received_at is None:
                stream._first_received_at = received_at
            elapsed = round(received_at - stream._first_received_at, 3)
            decoded.values["mission_time"] = elapsed

        packet = TelemetryPacket(
            **decoded.values,
            extras=extras,
            received_at=received_at,
            raw=text,
        )
        return stream.ingest(packet)

    def _sniff_vehicle(self, text: str) -> Optional[VehicleID]:
        """Best-effort VEHICLE_ID read from a line that failed to decode."""
        if text.startswith("PKT="):
            return VehicleID.ROCKET
        parts = [p.strip() for p in text.split(",")]
        # For 18/19-field packet ($T at index 0), vehicle is at index 16.
        # For 21-field packet ($T at index 0), vehicle is at index 2.
        for idx in (16, 17, 2, 1):
            if idx < len(parts):
                token = parts[idx].upper()
                if token in ("1", "ROCKET"):
                    return VehicleID.ROCKET
                elif token in ("2", "CANSAT"):
                    return VehicleID.CANSAT
        return None

    def _feed_heartbeat(self, text: str, received_at: float) -> None:
        try:
            uptime, counts = decode_heartbeat(text)
        except CodecError:
            # A corrupted heartbeat is not evidence the receiver is alive.
            # Dropping it silently lets the §3.5 timeout fire, which is the
            # correct outcome.
            return
        self.last_heartbeat = ReceiverHeartbeat(
            uptime=uptime, counts=counts, received_at=received_at, raw=text,
        )
        self._last_heartbeat_at = received_at

    # ── §3.5 receiver health ─────────────────────────────────────────────

    def heartbeat_age(self, now: Optional[float] = None) -> Optional[float]:
        if self.last_heartbeat is None:
            return None
        now = time.monotonic() if now is None else now
        return now - self._last_heartbeat_at

    def all_vehicles_silent(self, now: Optional[float] = None) -> bool:
        """§3.5 — used to suppress the "two independent vehicle failures"
        reading. Simultaneous loss of both vehicles is far more likely to
        be one USB cable than a catastrophic flight event, and the app
        must never present it as the latter."""
        return all(s.link_state(now) is VehicleLinkState.LOST
                   for s in self.streams.values())

    def drain_notices(self) -> List[str]:
        out = self.notices[:]
        self.notices.clear()
        return out
