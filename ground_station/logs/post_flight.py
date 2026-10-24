"""
ground_station/logs/post_flight.py
══════════════════════════════════
Post-Flight Telemetry Analysis Engine (§21).

Extracts and calculates physical parameters and telemetry metrics from
recorded flight telemetry logs:
- Altitude vs time
- Pressure vs time
- Temperature vs time
- Velocity vs time
- Acceleration vs time
- Gyro spin rate vs time
- GNSS altitude vs time
- Latitude vs time
- Longitude vs time
- Satellite count vs time
- Voltage vs time
- RSSI vs time
- SNR vs time

Calculates:
- Maximum altitude
- Maximum velocity
- Maximum acceleration
- Maximum gyro spin rate
- Minimum pressure
- Maximum temperature
- Minimum voltage
- Maximum satellite count
- Minimum satellite count
- Total valid packets
- Total rejected packets
- Missing packets
- Duplicate packets
- Checksum failures
- Observed telemetry rate
"""
from __future__ import annotations

import csv
import math
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple
import time

from PyQt6.QtCore import QObject, pyqtSignal

from ..codec import CsvCodec
from ..models import (
    FlightState, ReceiverState, SourceState, TelemetryPacket, VehicleID, VehicleLinkState,
)
from ..parser import IngestResult, LossThresholds, TelemetryDemux


@dataclass
class TelemetrySeries:
    """Time-indexed telemetry curve."""
    time: List[float] = field(default_factory=list)
    values: List[float] = field(default_factory=list)

    def is_empty(self) -> bool:
        return len(self.time) == 0


@dataclass
class PostFlightStats:
    """Summary statistics for post-flight analysis (§21)."""
    max_altitude: Optional[float] = None
    max_velocity: Optional[float] = None
    max_acceleration: Optional[float] = None
    max_gyro_spin_rate: Optional[float] = None
    min_pressure: Optional[float] = None
    max_temperature: Optional[float] = None
    min_voltage: Optional[float] = None
    max_satellites: Optional[int] = None
    min_satellites: Optional[int] = None
    total_valid_packets: int = 0
    total_rejected_packets: int = 0
    missing_packets: int = 0
    duplicate_packets: int = 0
    checksum_failures: int = 0
    observed_telemetry_rate_hz: float = 0.0


@dataclass
class FlightAnalysisResult:
    """Complete post-flight dataset and metrics."""
    session_id: str = ""
    vehicle: str = ""
    series: Dict[str, TelemetrySeries] = field(default_factory=dict)
    stats: PostFlightStats = field(default_factory=PostFlightStats)
    raw_lines_count: int = 0


def _get_col_val(row: List[str], col_map: Dict[str, int], keys: List[str]) -> Optional[str]:
    for k in keys:
        idx = col_map.get(k.lower())
        if idx is not None and idx < len(row):
            val = row[idx].strip()
            if val and val.upper() not in ("N/A", "NONE", "—", "NAN", "NULL"):
                return val
    return None


def _get_col_float(row: List[str], col_map: Dict[str, int], keys: List[str]) -> Optional[float]:
    val = _get_col_val(row, col_map, keys)
    return _try_float(val)


def _get_col_int(row: List[str], col_map: Dict[str, int], keys: List[str]) -> Optional[int]:
    val = _get_col_val(row, col_map, keys)
    return _try_int(val)


def _parse_state(state_name_raw: Optional[str], state_code_raw: Optional[int]) -> Tuple[int, str, FlightState]:
    if state_name_raw:
        sn = state_name_raw.upper().replace("-", "_").strip()
        name_to_code = {
            "BOOT": 0, "PRELAUNCH": 1, "PRE_LAUNCH": 1, "BOOST": 2, "COAST": 3,
            "APOGEE": 4, "DESCENT": 5, "LANDING": 6, "RECOVERY": 7,
        }
        if sn in name_to_code:
            code = name_to_code[sn]
            fs = FlightState.PRE_LAUNCH if sn in ("PRELAUNCH", "PRE_LAUNCH") else FlightState[sn]
            return code, sn, fs

    code = state_code_raw if state_code_raw is not None else 0
    code_map = {
        0: (0, "BOOT", FlightState.BOOT),
        1: (1, "PRELAUNCH", FlightState.PRE_LAUNCH),
        2: (2, "BOOST", FlightState.BOOST),
        3: (3, "COAST", FlightState.COAST),
        4: (4, "APOGEE", FlightState.APOGEE),
        5: (5, "DESCENT", FlightState.DESCENT),
        6: (6, "LANDING", FlightState.LANDING),
        7: (7, "RECOVERY", FlightState.RECOVERY),
    }
    return code_map.get(code, (0, "BOOT", FlightState.BOOT))


class PostFlightAnalyzer:
    """Analyzes recorded telemetry CSV log files from onboard SD or wire logs."""

    @staticmethod
    def analyze_file(csv_path: str) -> FlightAnalysisResult:
        """Parse a recorded telemetry CSV file and produce full flight analysis."""
        if not os.path.exists(csv_path):
            raise FileNotFoundError(f"Telemetry log file not found: {csv_path}")

        series: Dict[str, TelemetrySeries] = {
            "altitude": TelemetrySeries(),
            "pressure": TelemetrySeries(),
            "temperature": TelemetrySeries(),
            "velocity": TelemetrySeries(),
            "acceleration": TelemetrySeries(),
            "gyro_spin_rate": TelemetrySeries(),
            "gnss_altitude": TelemetrySeries(),
            "latitude": TelemetrySeries(),
            "longitude": TelemetrySeries(),
            "satellites": TelemetrySeries(),
            "voltage": TelemetrySeries(),
            "rssi": TelemetrySeries(),
            "snr": TelemetrySeries(),
        }

        valid_count = 0
        rejected_count = 0
        missing_count = 0
        dup_count = 0
        checksum_fails = 0

        last_packet_count: Optional[int] = None
        timestamps: List[float] = []

        with open(csv_path, "r", encoding="utf-8", errors="replace") as fh:
            reader = csv.reader(fh)
            first_row = next(reader, None)
            if not first_row:
                return FlightAnalysisResult(
                    session_id=os.path.basename(csv_path),
                    series=series,
                    stats=PostFlightStats(),
                )

            # Check if header row
            first_joined = ",".join(first_row).lower()
            is_csv_header = any(k in first_joined for k in (
                "mission_ms", "altitude_m", "gs_ms", "pressure_pa", "state_name", "mission_time_s", "altitude"
            ))

            if is_csv_header:
                col_map = {col.strip().lower(): idx for idx, col in enumerate(first_row)}

                for line_no, row in enumerate(reader, start=2):
                    if not row or not any(row):
                        continue
                    if row[0].startswith("#"):
                        rejected_count += 1
                        continue

                    try:
                        t_val = _get_col_float(row, col_map, ["mission_time_s", "gs_time_s", "mission_time", "time"])
                        if t_val is None:
                            ms = _get_col_float(row, col_map, ["mission_ms", "gs_ms"])
                            if ms is not None:
                                t_val = ms / 1000.0
                        if t_val is None:
                            t_val = float(valid_count)

                        pkt_cnt = _get_col_int(row, col_map, ["packet_count", "pkt_count", "pkt"])
                        if pkt_cnt is not None:
                            if last_packet_count is not None:
                                if pkt_cnt <= last_packet_count:
                                    dup_count += 1
                                elif pkt_cnt > last_packet_count + 1:
                                    missing_count += (pkt_cnt - last_packet_count - 1)
                            last_packet_count = pkt_cnt

                        alt = _get_col_float(row, col_map, ["altitude_m", "baro_altitude_m", "altitude", "alt"])
                        if alt is not None:
                            series["altitude"].time.append(t_val)
                            series["altitude"].values.append(alt)

                        p = _get_col_float(row, col_map, ["pressure_pa", "pressure"])
                        if p is None:
                            hpa = _get_col_float(row, col_map, ["pressure_hpa"])
                            if hpa is not None:
                                p = hpa * 100.0
                        if p is not None:
                            series["pressure"].time.append(t_val)
                            series["pressure"].values.append(p)

                        temp = _get_col_float(row, col_map, ["temperature_c", "ms5611_temp_c", "ds18b20_temp_c", "temperature", "temp"])
                        if temp is not None:
                            series["temperature"].time.append(t_val)
                            series["temperature"].values.append(temp)

                        v = _get_col_float(row, col_map, ["load_v", "voltage_v", "voltage", "ina_bus_v", "battery_voltage"])
                        if v is not None:
                            series["voltage"].time.append(t_val)
                            series["voltage"].values.append(v)

                        lat = _get_col_float(row, col_map, ["latitude_deg", "latitude", "gnss_latitude", "lat"])
                        if lat is not None:
                            series["latitude"].time.append(t_val)
                            series["latitude"].values.append(lat)

                        lon = _get_col_float(row, col_map, ["longitude_deg", "longitude", "gnss_longitude", "lon"])
                        if lon is not None:
                            series["longitude"].time.append(t_val)
                            series["longitude"].values.append(lon)

                        galt = _get_col_float(row, col_map, ["gnss_altitude_m", "gnss_altitude", "gnss_agl_m", "galt"])
                        if galt is not None:
                            series["gnss_altitude"].time.append(t_val)
                            series["gnss_altitude"].values.append(galt)

                        sats = _get_col_int(row, col_map, ["satellites", "gnss_satellites", "sats"])
                        if sats is not None:
                            series["satellites"].time.append(t_val)
                            series["satellites"].values.append(float(sats))

                        acc = _get_col_float(row, col_map, ["acc_mag_ms2", "acceleration_ms2", "acc_axial_ms2", "accelerometer", "acceleration"])
                        if acc is not None:
                            series["acceleration"].time.append(t_val)
                            series["acceleration"].values.append(acc)

                        gyro = _get_col_float(row, col_map, ["spin_dps", "spin_rate_dps", "gyro_spin_rate", "gyro_z_dps"])
                        if gyro is not None:
                            series["gyro_spin_rate"].time.append(t_val)
                            series["gyro_spin_rate"].values.append(gyro)

                        vel = _get_col_float(row, col_map, ["velocity_ms", "baro_velocity_ms", "velocity", "vel"])
                        if vel is not None:
                            series["velocity"].time.append(t_val)
                            series["velocity"].values.append(vel)

                        rssi = _get_col_float(row, col_map, ["rssi_dbm", "rssi"])
                        if rssi is not None:
                            series["rssi"].time.append(t_val)
                            series["rssi"].values.append(rssi)

                        snr = _get_col_float(row, col_map, ["snr_db", "snr"])
                        if snr is not None:
                            series["snr"].time.append(t_val)
                            series["snr"].values.append(snr)

                        valid_count += 1
                        timestamps.append(t_val)

                    except Exception:
                        rejected_count += 1

            else:
                # Wire format: lines starting with $T
                all_rows = [first_row] + list(reader)
                for line_no, row in enumerate(all_rows, start=1):
                    if not row or not any(row):
                        continue

                    if row[0] == "#REJECTED":
                        rejected_count += 1
                        if len(row) > 1 and "CHECKSUM" in row[1].upper():
                            checksum_fails += 1
                        continue

                    if row[0] != "$T" or len(row) < 18:
                        rejected_count += 1
                        continue

                    try:
                        t_val = _try_float(row[2])
                        if t_val is None and len(row) > 19:
                            t_val = _try_float(row[19])
                        if t_val is None:
                            t_val = float(valid_count)

                        pkt_cnt = _try_int(row[3])
                        if pkt_cnt is not None:
                            if last_packet_count is not None:
                                if pkt_cnt <= last_packet_count:
                                    dup_count += 1
                                elif pkt_cnt > last_packet_count + 1:
                                    missing_count += (pkt_cnt - last_packet_count - 1)
                            last_packet_count = pkt_cnt

                        alt = _try_scaled_float(row[4], 10.0)
                        if alt is not None:
                            series["altitude"].time.append(t_val)
                            series["altitude"].values.append(alt)

                        p = _try_float(row[5])
                        if p is not None:
                            series["pressure"].time.append(t_val)
                            series["pressure"].values.append(p)

                        temp = _try_scaled_float(row[6], 10.0)
                        if temp is not None:
                            series["temperature"].time.append(t_val)
                            series["temperature"].values.append(temp)

                        v = _try_scaled_float(row[7], 100.0)
                        if v is not None:
                            series["voltage"].time.append(t_val)
                            series["voltage"].values.append(v)

                        lat = _try_scaled_float(row[9], 10000.0)
                        if lat is not None:
                            series["latitude"].time.append(t_val)
                            series["latitude"].values.append(lat)

                        lon = _try_scaled_float(row[10], 10000.0)
                        if lon is not None:
                            series["longitude"].time.append(t_val)
                            series["longitude"].values.append(lon)

                        galt = _try_scaled_float(row[11], 10.0)
                        if galt is not None:
                            series["gnss_altitude"].time.append(t_val)
                            series["gnss_altitude"].values.append(galt)

                        sats = _try_int(row[12])
                        if sats is not None:
                            series["satellites"].time.append(t_val)
                            series["satellites"].values.append(float(sats))

                        acc = _try_scaled_float(row[13], 10.0)
                        if acc is not None:
                            series["acceleration"].time.append(t_val)
                            series["acceleration"].values.append(acc)

                        gyro = _try_scaled_float(row[14], 10.0)
                        if gyro is not None:
                            series["gyro_spin_rate"].time.append(t_val)
                            series["gyro_spin_rate"].values.append(gyro)

                        vel = None
                        if len(row) > 17 and (row[16].strip().upper() in ("1", "2", "ROCKET", "CANSAT") or _try_float(row[17]) is not None):
                            vel = _try_float(row[17])
                        elif len(row) > 16:
                            vel = _try_float(row[16])
                        if vel is not None:
                            series["velocity"].time.append(t_val)
                            series["velocity"].values.append(vel)

                        if len(row) > 20:
                            rssi = _try_float(row[20])
                            if rssi is not None:
                                series["rssi"].time.append(t_val)
                                series["rssi"].values.append(rssi)

                        if len(row) > 21:
                            snr = _try_float(row[21])
                            if snr is not None:
                                series["snr"].time.append(t_val)
                                series["snr"].values.append(snr)

                        valid_count += 1
                        timestamps.append(t_val)

                    except Exception:
                        rejected_count += 1

        stats = PostFlightStats(
            max_altitude=_max_or_none(series["altitude"].values),
            max_velocity=_max_or_none(series["velocity"].values),
            max_acceleration=_max_or_none(series["acceleration"].values),
            max_gyro_spin_rate=_max_or_none(series["gyro_spin_rate"].values),
            min_pressure=_min_or_none(series["pressure"].values),
            max_temperature=_max_or_none(series["temperature"].values),
            min_voltage=_min_or_none(series["voltage"].values),
            max_satellites=int(_max_or_none(series["satellites"].values)) if series["satellites"].values else None,
            min_satellites=int(_min_or_none(series["satellites"].values)) if series["satellites"].values else None,
            total_valid_packets=valid_count,
            total_rejected_packets=rejected_count,
            missing_packets=missing_count,
            duplicate_packets=dup_count,
            checksum_failures=checksum_fails,
        )

        if len(timestamps) > 1:
            dur = timestamps[-1] - timestamps[0]
            if dur > 0:
                stats.observed_telemetry_rate_hz = round((len(timestamps) - 1) / dur, 2)

        basename = os.path.basename(csv_path)
        return FlightAnalysisResult(
            session_id=basename,
            vehicle="ROCKET" if "ROCKET" in basename.upper() else ("CANSAT" if "CANSAT" in basename.upper() else "UNKNOWN"),
            series=series,
            stats=stats,
            raw_lines_count=valid_count + rejected_count,
        )


    @staticmethod
    def extract_packets_from_file(csv_path: str) -> List[Tuple[TelemetryPacket, IngestResult]]:
        """Extract canonical TelemetryPacket and IngestResult instances from flight file."""
        from ..parser import LossTier, IngestResult
        out: List[Tuple[TelemetryPacket, IngestResult]] = []

        if not os.path.exists(csv_path):
            return out

        with open(csv_path, "r", encoding="utf-8", errors="replace") as fh:
            reader = csv.reader(fh)
            first_row = next(reader, None)
            if not first_row:
                return out

            first_joined = ",".join(first_row).lower()
            is_csv_header = any(k in first_joined for k in (
                "mission_ms", "altitude_m", "gs_ms", "pressure_pa", "state_name", "mission_time_s", "altitude"
            ))

            if is_csv_header:
                col_map = {col.strip().lower(): idx for idx, col in enumerate(first_row)}
                for line_no, row in enumerate(reader, start=2):
                    if not row or not any(row) or row[0].startswith("#"):
                        continue
                    try:
                        t_val = _get_col_float(row, col_map, ["mission_time_s", "gs_time_s", "mission_time", "time"])
                        if t_val is None:
                            ms = _get_col_float(row, col_map, ["mission_ms", "gs_ms"])
                            if ms is not None:
                                t_val = ms / 1000.0
                        if t_val is None:
                            t_val = float(len(out))

                        pkt_cnt = _get_col_int(row, col_map, ["packet_count", "pkt_count", "pkt"]) or len(out)
                        alt = _get_col_float(row, col_map, ["altitude_m", "baro_altitude_m", "altitude"]) or 0.0
                        vel = _get_col_float(row, col_map, ["velocity_ms", "baro_velocity_ms", "velocity"]) or 0.0
                        p = _get_col_float(row, col_map, ["pressure_pa", "pressure"]) or 101325.0
                        temp = _get_col_float(row, col_map, ["temperature_c", "ms5611_temp_c", "temperature"]) or 20.0
                        v = _get_col_float(row, col_map, ["load_v", "voltage_v", "voltage", "ina_bus_v"]) or 5.0
                        lat = _get_col_float(row, col_map, ["latitude_deg", "latitude"]) or 0.0
                        lon = _get_col_float(row, col_map, ["longitude_deg", "longitude"]) or 0.0
                        galt = _get_col_float(row, col_map, ["gnss_altitude_m", "gnss_altitude"]) or 0.0
                        sats = _get_col_int(row, col_map, ["satellites", "gnss_satellites"]) or 0
                        acc = _get_col_float(row, col_map, ["acc_mag_ms2", "acceleration_ms2", "acc_axial_ms2", "accelerometer"]) or 9.8
                        spin = _get_col_float(row, col_map, ["spin_dps", "spin_rate_dps", "gyro_spin_rate"]) or 0.0
                        gt = _get_col_val(row, col_map, ["gnss_utc", "gnss_time_s", "gnss_time"])
                        st_name_raw = _get_col_val(row, col_map, ["state_name"])
                        st_code_raw = _get_col_int(row, col_map, ["state"])
                        st_code, st_name, fs = _parse_state(st_name_raw, st_code_raw)
                        vid_str = _get_col_val(row, col_map, ["vehicle_id", "vehicle"]) or "ROCKET"
                        vid = VehicleID.CANSAT if "CANSAT" in vid_str.upper() or vid_str == "2" else VehicleID.ROCKET
                        raw_str = ",".join(row)

                        pkt = TelemetryPacket(
                            vehicle_id=vid,
                            mission_time=t_val,
                            packet_count=pkt_cnt,
                            state=fs,
                            flight_state_code=st_code,
                            altitude=alt,
                            velocity=vel,
                            pressure=p,
                            temperature=temp,
                            battery_voltage=v,
                            gnss_time=gt,
                            gnss_latitude=lat,
                            gnss_longitude=lon,
                            gnss_altitude=galt,
                            gnss_satellites=sats,
                            accelerometer=acc,
                            gyro_spin_rate=spin,
                            received_at=time.monotonic(),
                            raw=raw_str,
                        )
                        res = IngestResult(packet=pkt, tier=LossTier.NONE)
                        out.append((pkt, res))
                    except Exception:
                        pass
        return out


def _try_float(val: Any) -> Optional[float]:
    if val is None:
        return None
    s = str(val).strip().upper()
    if s in ("N/A", "NOT AVAILABLE", "NONE", "—", "", "ERR"):
        return None
    try:
        return float(s)
    except ValueError:
        return None


def _try_scaled_float(val: Any, scale: float = 1.0) -> Optional[float]:
    if val is None:
        return None
    s = str(val).strip().upper()
    if s in ("N/A", "NOT AVAILABLE", "NONE", "—", "", "ERR"):
        return None
    try:
        if "." in s:
            return float(s)
        else:
            return float(int(s)) / scale
    except ValueError:
        try:
            return float(s)
        except ValueError:
            return None


def _try_int(val: Any) -> Optional[int]:
    f = _try_float(val)
    return int(f) if f is not None else None


def _max_or_none(vals: List[float]) -> Optional[float]:
    clean = [v for v in vals if v is not None and not math.isnan(v)]
    return max(clean) if clean else None


def _min_or_none(vals: List[float]) -> Optional[float]:
    clean = [v for v in vals if v is not None and not math.isnan(v)]
    return min(clean) if clean else None


class PostFlightSource(QObject):
    """Data source for Post-Flight SD Card Analysis (§25–§29).
    
    Reads physically recorded flight logs from SD card, decodes them through the
    same canonical Book1 CsvCodec and TelemetryDemux, and feeds the identical
    dashboard, graphs, and packet inspector without transmitting over LoRa.
    """
    rocket_packet = pyqtSignal(object, object)
    cansat_packet = pyqtSignal(object, object)
    line_rejected = pyqtSignal(object)
    raw_line = pyqtSignal(str, float)

    source_state_changed = pyqtSignal(object)
    receiver_state_changed = pyqtSignal(object)
    vehicle_state_changed = pyqtSignal(object, object)
    notice = pyqtSignal(str)
    command_reply = pyqtSignal(str, str, dict, str)
    ground_cmd = pyqtSignal(int, str, str, str)
    servo_report = pyqtSignal(int, float, float, float, int, int, str)

    def __init__(self, filepath: str, thresholds: Optional[LossThresholds] = None, parent=None):
        super().__init__(parent)
        self.filepath = filepath
        self.codec = CsvCodec()
        self.demux = TelemetryDemux(self.codec, thresholds)
        self.demux.on_command_ack = lambda cmd, st, det, raw: self.command_reply.emit(cmd, st, det, raw)
        self.demux.on_ground_cmd = lambda cmd_id, st, info, raw: self.ground_cmd.emit(cmd_id, st, info, raw)
        self.demux.on_servo_report = lambda ch, deg, est, rem, mode, step, raw: self.servo_report.emit(ch, deg, est, rem, mode, step, raw)
        self._command_sequence = 0
        self.analysis: Optional[FlightAnalysisResult] = None

    def vehicle_link_state(self, vid: VehicleID) -> VehicleLinkState:
        return VehicleLinkState.RECEIVING

    @property
    def port_name(self) -> str:
        return "SD-CARD"

    @property
    def baudrate(self) -> int:
        return 0

    def connect_to(self, *_args, **_kwargs) -> None:
        self.source_state_changed.emit(SourceState.CONNECTED)
        self.receiver_state_changed.emit(ReceiverState.ALIVE)
        self._replay_file()

    def disconnect_source(self) -> None:
        self.source_state_changed.emit(SourceState.DISCONNECTED)

    def send_command(self, target: VehicleID, command: str, sequence: Optional[int] = None) -> int:
        self.notice.emit("Commands are disabled in Post-Flight SD Card Analysis mode.")
        if sequence is None:
            self._command_sequence += 1
            sequence = self._command_sequence
        return sequence

    def _replay_file(self) -> None:
        if not os.path.exists(self.filepath):
            self.notice.emit(f"SD flight file not found: {self.filepath}")
            return

        self.analysis = PostFlightAnalyzer.analyze_file(self.filepath)
        extracted = PostFlightAnalyzer.extract_packets_from_file(self.filepath)

        if extracted:
            for pkt, res in extracted:
                self.raw_line.emit(pkt.raw, pkt.received_at)
                if pkt.vehicle_id is VehicleID.ROCKET:
                    self.rocket_packet.emit(pkt, res)
                else:
                    self.cansat_packet.emit(pkt, res)
        else:
            with open(self.filepath, "r", encoding="utf-8", errors="replace") as fh:
                lines = fh.readlines()

            for line in lines:
                text = line.strip()
                if not text or text.startswith("$T,TEAM_ID") or text.startswith("TEAM_ID"):
                    continue

                received_at = time.monotonic()
                res = self.demux.feed(text, received_at)
                self.raw_line.emit(text, received_at)

                if res is not None:
                    if res.packet is not None:
                        if res.packet.vehicle_id is VehicleID.ROCKET:
                            self.rocket_packet.emit(res.packet, res)
                        else:
                            self.cansat_packet.emit(res.packet, res)
                    elif res.rejected is not None:
                        self.line_rejected.emit(res.rejected)

        for vid in VehicleID:
            self.vehicle_state_changed.emit(vid, VehicleLinkState.RECEIVING)

        self.notice.emit(
            f"Post-Flight Analysis complete: {self.analysis.stats.total_valid_packets} valid packets, "
            f"max alt: {self.analysis.stats.max_altitude if self.analysis.stats.max_altitude is not None else 'N/A'} m"
        )


