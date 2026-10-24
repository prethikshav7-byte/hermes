"""
ground_station/logs/sd_log.py
═════════════════════════════
SD-Card Compatible Telemetry Logger & Session Flight Record (§18–§20).

Preserves the exact Book1 19-field header and field order:
$T,TEAM_ID,TIMESTAMP,PACKET_COUNT,ALTITUDE,PRESSURE,TEMPERATURE,VOLTAGE,GNSS_TIME,GNSS_LATITUDE,GNSS_LONGITUDE,GNSS_ALTITUDE,GNSS_SATELLITES,ACCELEROMETER,GYRO_SPIN_RATE,FLIGHT_SOFTWARE_STATE,VELOCITY,VEHICLE,CHECKSUM

Metadata (RECEIVE_TIME, RSSI, SNR, VALIDATION_STATUS, DATA_SOURCE) is recorded
at the end of each record to preserve the official telemetry field order.

Deterministic filenames:
  Root: PHOENIX_<DATE>_<TIME>_<VEHICLE>.csv
  ground_station/: rocket_received.csv, cansat_received.csv
  session_metadata.json: Comprehensive statistical and integrity summary

Thread-safe, non-blocking queue architecture prevents I/O pauses on the UI/render loop.
"""
from __future__ import annotations

import csv
import json
import os
import queue
import threading
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from typing import Any, Dict, List, Optional, TextIO

from ..models import (
    FlightState, TelemetryPacket, ValidationStatus, VehicleID,
)

BOOK1_HEADER = [
    "$T",
    "TEAM_ID",
    "TIMESTAMP",
    "PACKET_COUNT",
    "ALTITUDE",
    "PRESSURE",
    "TEMPERATURE",
    "VOLTAGE",
    "GNSS_TIME",
    "GNSS_LATITUDE",
    "GNSS_LONGITUDE",
    "GNSS_ALTITUDE",
    "GNSS_SATELLITES",
    "ACCELEROMETER",
    "GYRO_SPIN_RATE",
    "FLIGHT_SOFTWARE_STATE",
    "VELOCITY",
    "VEHICLE",
    "CHECKSUM",
    "RECEIVE_TIME",
    "RSSI",
    "SNR",
    "VALIDATION_STATUS",
    "DATA_SOURCE",
]


@dataclass
class SessionSummary:
    """Post-flight / Session Metadata Summary (§20)."""
    session_id: str
    vehicle: str
    start_time: str
    team_id: str = "N/A"
    data_source: str = "HARDWARE"
    valid_packets: int = 0
    total_packets: int = 0
    rejected_packets: int = 0
    missing_packets: int = 0
    duplicate_packets: int = 0
    checksum_failures: int = 0
    max_altitude_m: Optional[float] = None
    max_altitude: Optional[float] = None
    min_altitude_m: Optional[float] = None
    avg_altitude_m: Optional[float] = None
    max_velocity_ms: Optional[float] = None
    max_acceleration_ms2: Optional[float] = None
    min_pressure_pa: Optional[float] = None
    max_temperature_c: Optional[float] = None
    min_voltage_v: Optional[float] = None
    min_rssi: Optional[float] = None
    max_rssi: Optional[float] = None
    avg_rssi: Optional[float] = None
    min_snr: Optional[float] = None
    max_snr: Optional[float] = None
    avg_snr: Optional[float] = None
    observed_frequency_hz: float = 0.0


class SDTelemetryLogger:
    """High-performance, thread-safe SD-card compatible flight telemetry logger."""

    def __init__(self, directory: str = "logs", session: Optional[str] = None,
                 data_source: str = "HARDWARE", use_async_queue: bool = True):
        self.session = session or datetime.now().strftime("%Y%m%d_%H%M%S")
        self.data_source = data_source.upper()
        self.directory = os.path.join(directory, self.session)
        self.gs_dir = os.path.join(self.directory, "ground_station")
        self.onboard_dir = os.path.join(self.directory, "onboard")

        os.makedirs(self.directory, exist_ok=True)
        os.makedirs(self.gs_dir, exist_ok=True)
        os.makedirs(self.onboard_dir, exist_ok=True)

        self.start_iso = datetime.now().isoformat()
        self.start_mono = time.monotonic()

        self._files: Dict[VehicleID, List[TextIO]] = {v: [] for v in VehicleID}
        self._writers: Dict[VehicleID, List[Any]] = {v: [] for v in VehicleID}
        self._lock = threading.Lock()

        self._rssi_values: Dict[VehicleID, List[float]] = {v: [] for v in VehicleID}
        self._snr_values: Dict[VehicleID, List[float]] = {v: [] for v in VehicleID}
        self._timestamps: Dict[VehicleID, List[float]] = {v: [] for v in VehicleID}
        self._team_ids: Dict[VehicleID, Optional[str]] = {v: None for v in VehicleID}

        # Detailed metrics for session metadata
        self._alt_values: Dict[VehicleID, List[float]] = {v: [] for v in VehicleID}
        self._vel_values: Dict[VehicleID, List[float]] = {v: [] for v in VehicleID}
        self._acc_values: Dict[VehicleID, List[float]] = {v: [] for v in VehicleID}
        self._press_values: Dict[VehicleID, List[float]] = {v: [] for v in VehicleID}
        self._temp_values: Dict[VehicleID, List[float]] = {v: [] for v in VehicleID}
        self._volt_values: Dict[VehicleID, List[float]] = {v: [] for v in VehicleID}

        self.valid_counts: Dict[VehicleID, int] = {v: 0 for v in VehicleID}
        self.rejected_counts: Dict[VehicleID, int] = {v: 0 for v in VehicleID}
        self.missing_counts: Dict[VehicleID, int] = {v: 0 for v in VehicleID}
        self.duplicate_counts: Dict[VehicleID, int] = {v: 0 for v in VehicleID}
        self.checksum_fail_counts: Dict[VehicleID, int] = {v: 0 for v in VehicleID}

        self._init_files()

        # Asynchronous non-blocking writer queue
        self._use_async = use_async_queue
        self._queue: queue.Queue = queue.Queue(maxsize=20000)
        self._stop_event = threading.Event()
        self._worker_thread: Optional[threading.Thread] = None
        self._last_flush = time.monotonic()

        if self._use_async:
            self._worker_thread = threading.Thread(
                target=self._queue_worker,
                name="SDTelemetryLoggerWorker",
                daemon=True,
            )
            self._worker_thread.start()

    def _init_files(self) -> None:
        for vid in VehicleID:
            fname_root = f"PHOENIX_{self.session}_{vid.value.upper()}.csv"
            path_root = os.path.join(self.directory, fname_root)

            fname_gs = f"{vid.value.lower()}_received.csv"
            path_gs = os.path.join(self.gs_dir, fname_gs)

            for p in (path_root, path_gs):
                try:
                    fh = open(p, "w", newline="", encoding="utf-8")
                    writer = csv.writer(fh)
                    writer.writerow(BOOK1_HEADER)
                    fh.flush()
                    self._files[vid].append(fh)
                    self._writers[vid].append(writer)
                except Exception as exc:
                    import sys
                    print(f"[ERR] Failed to initialize {p}: {exc}", file=sys.stderr)

    def filepath_for(self, vid: VehicleID) -> str:
        fname = f"PHOENIX_{self.session}_{vid.value.upper()}.csv"
        return os.path.join(self.directory, fname)

    def gs_filepath_for(self, vid: VehicleID) -> str:
        fname = f"{vid.value.lower()}_received.csv"
        return os.path.join(self.gs_dir, fname)

    def __enter__(self) -> "SDTelemetryLogger":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()

    def log_packet(self, packet: TelemetryPacket,
                   validation_status: ValidationStatus = ValidationStatus.VALID,
                   derived_velocity: Optional[float] = None) -> None:
        """Enqueue one validated telemetry packet for non-blocking disk writing."""
        if self._use_async and self._worker_thread and self._worker_thread.is_alive():
            try:
                self._queue.put_nowait(("packet", packet, validation_status, derived_velocity))
            except queue.Full:
                # If queue overflows under extreme load, write synchronously to protect data
                self._write_packet_sync(packet, validation_status, derived_velocity)
        else:
            self._write_packet_sync(packet, validation_status, derived_velocity)

    def log_rejected(self, raw_line: str, reason_str: str,
                      vehicle_id: Optional[VehicleID] = None,
                      received_at: Optional[float] = None) -> None:
        """Enqueue rejected packet for recording."""
        vid = vehicle_id or VehicleID.ROCKET
        if self._use_async and self._worker_thread and self._worker_thread.is_alive():
            try:
                self._queue.put_nowait(("rejected", raw_line, reason_str, vid, received_at))
            except queue.Full:
                self._write_rejected_sync(raw_line, reason_str, vid, received_at)
        else:
            self._write_rejected_sync(raw_line, reason_str, vid, received_at)

    def _queue_worker(self) -> None:
        """Background thread worker draining telemetry records and periodic flushes."""
        while not self._stop_event.is_set() or not self._queue.empty():
            try:
                item = self._queue.get(timeout=0.25)
            except queue.Empty:
                self._flush_all()
                continue

            if item is None:
                self._queue.task_done()
                break

            kind = item[0]
            if kind == "packet":
                _, pkt, status, d_vel = item
                self._write_packet_sync(pkt, status, d_vel)
            elif kind == "rejected":
                _, raw, reason, vid, rx_t = item
                self._write_rejected_sync(raw, reason, vid, rx_t)
            self._queue.task_done()

            # Bounded periodic flush: every 1.0s or when queue is drained
            now = time.monotonic()
            if now - self._last_flush >= 1.0 or self._queue.empty():
                self._flush_all()
                self._last_flush = now

    def _write_packet_sync(self, packet: TelemetryPacket,
                           validation_status: ValidationStatus = ValidationStatus.VALID,
                           derived_velocity: Optional[float] = None) -> None:
        vid = packet.vehicle_id or VehicleID.ROCKET
        with self._lock:
            writers = self._writers.get(vid, [])
            if not writers:
                return

            if packet.team_id is not None:
                self._team_ids[vid] = str(packet.team_id)

            team_id_str = str(packet.team_id) if packet.team_id is not None else "N/A"
            ts_str = f"{packet.mission_time:.2f}" if packet.mission_time is not None else "N/A"
            pkt_cnt_str = str(packet.packet_count)
            alt_str = f"{packet.altitude:.2f}" if packet.altitude is not None else "N/A"
            p_str = f"{packet.pressure:.1f}" if packet.pressure is not None else "N/A"
            t_str = f"{packet.temperature:.2f}" if packet.temperature is not None else "N/A"
            v_str = f"{packet.battery_voltage:.2f}" if packet.battery_voltage is not None else "N/A"
            gnss_time_str = packet.gnss_time if packet.gnss_time else "N/A"
            lat_str = f"{packet.gnss_latitude:.6f}" if packet.gnss_latitude is not None else "N/A"
            lon_str = f"{packet.gnss_longitude:.6f}" if packet.gnss_longitude is not None else "N/A"
            galt_str = f"{packet.gnss_altitude:.2f}" if packet.gnss_altitude is not None else "N/A"
            sats_str = str(packet.gnss_satellites) if packet.gnss_satellites is not None else "N/A"

            # Acceleration
            accel_val = packet.accelerometer
            if accel_val is None and (packet.accel_x is not None and packet.accel_y is not None and packet.accel_z is not None):
                accel_val = (packet.accel_x**2 + packet.accel_y**2 + packet.accel_z**2)**0.5
            accel_str = f"{accel_val:.2f}" if accel_val is not None else "N/A"

            spin_str = f"{packet.gyro_spin_rate:.2f}" if packet.gyro_spin_rate is not None else "N/A"
            state_str = str(packet.state.ordinal) if packet.state is not None else (
                str(packet.flight_state_code) if packet.flight_state_code is not None else "N/A"
            )

            vel_val = packet.velocity if packet.velocity is not None else derived_velocity
            vel_str = f"{vel_val:.2f}" if vel_val is not None else "N/A"

            veh_code = "1" if vid is VehicleID.ROCKET else "2"
            checksum_str = packet.checksum if packet.checksum else "N/A"

            rx_time_str = f"{packet.received_at:.3f}" if packet.received_at > 0 else f"{time.monotonic():.3f}"
            rssi_str = f"{packet.rssi:.1f}" if packet.rssi is not None else "N/A"
            snr_str = f"{packet.snr:.2f}" if packet.snr is not None else "N/A"

            row = [
                "$T",
                team_id_str,
                ts_str,
                pkt_cnt_str,
                alt_str,
                p_str,
                t_str,
                v_str,
                gnss_time_str,
                lat_str,
                lon_str,
                galt_str,
                sats_str,
                accel_str,
                spin_str,
                state_str,
                vel_str,
                veh_code,
                checksum_str,
                rx_time_str,
                rssi_str,
                snr_str,
                validation_status.value,
                self.data_source,
            ]

            for w in writers:
                try:
                    w.writerow(row)
                except Exception:
                    pass

            # Update stats
            self.valid_counts[vid] += 1
            if packet.altitude is not None:
                self._alt_values[vid].append(packet.altitude)
            if vel_val is not None:
                self._vel_values[vid].append(vel_val)
            if accel_val is not None:
                self._acc_values[vid].append(accel_val)
            if packet.pressure is not None:
                self._press_values[vid].append(packet.pressure)
            if packet.temperature is not None:
                self._temp_values[vid].append(packet.temperature)
            if packet.battery_voltage is not None:
                self._volt_values[vid].append(packet.battery_voltage)
            if packet.rssi is not None:
                self._rssi_values[vid].append(packet.rssi)
            if packet.snr is not None:
                self._snr_values[vid].append(packet.snr)
            if packet.received_at > 0:
                self._timestamps[vid].append(packet.received_at)
            else:
                self._timestamps[vid].append(time.monotonic())

    def _write_rejected_sync(self, raw_line: str, reason_str: str,
                             vid: VehicleID,
                             received_at: Optional[float] = None) -> None:
        with self._lock:
            self.rejected_counts[vid] += 1
            reason_upper = str(reason_str).upper()
            if "CHECKSUM" in reason_upper:
                self.checksum_fail_counts[vid] += 1
            elif "DUPLICATE" in reason_upper:
                self.duplicate_counts[vid] += 1

    def _flush_all(self) -> None:
        with self._lock:
            for vid, fhs in self._files.items():
                for fh in fhs:
                    try:
                        fh.flush()
                    except Exception:
                        pass

    def flush(self) -> None:
        self._flush_all()

    def get_session_summary(self, vid: VehicleID) -> SessionSummary:
        """Compute full session flight summary (§20)."""
        rssi_list = self._rssi_values[vid]
        snr_list = self._snr_values[vid]
        ts_list = self._timestamps[vid]
        alt_list = self._alt_values[vid]
        vel_list = self._vel_values[vid]
        acc_list = self._acc_values[vid]
        p_list = self._press_values[vid]
        t_list = self._temp_values[vid]
        v_list = self._volt_values[vid]

        min_rssi = min(rssi_list) if rssi_list else None
        max_rssi = max(rssi_list) if rssi_list else None
        avg_rssi = sum(rssi_list) / len(rssi_list) if rssi_list else None

        min_snr = min(snr_list) if snr_list else None
        max_snr = max(snr_list) if snr_list else None
        avg_snr = sum(snr_list) / len(snr_list) if snr_list else None

        freq = 0.0
        if len(ts_list) > 1:
            dur = ts_list[-1] - ts_list[0]
            if dur > 0:
                freq = round((len(ts_list) - 1) / dur, 2)

        return SessionSummary(
            session_id=self.session,
            vehicle=vid.value,
            start_time=self.start_iso,
            team_id=self._team_ids[vid] or "N/A",
            data_source=self.data_source,
            valid_packets=self.valid_counts[vid],
            total_packets=self.valid_counts[vid],
            rejected_packets=self.rejected_counts[vid],
            missing_packets=self.missing_counts[vid],
            duplicate_packets=self.duplicate_counts[vid],
            checksum_failures=self.checksum_fail_counts[vid],
            max_altitude_m=round(max(alt_list), 2) if alt_list else None,
            max_altitude=round(max(alt_list), 2) if alt_list else None,
            min_altitude_m=round(min(alt_list), 2) if alt_list else None,
            avg_altitude_m=round(sum(alt_list) / len(alt_list), 2) if alt_list else None,
            max_velocity_ms=round(max(vel_list), 2) if vel_list else None,
            max_acceleration_ms2=round(max(acc_list), 2) if acc_list else None,
            min_pressure_pa=round(min(p_list), 1) if p_list else None,
            max_temperature_c=round(max(t_list), 2) if t_list else None,
            min_voltage_v=round(min(v_list), 2) if v_list else None,
            min_rssi=min_rssi,
            max_rssi=max_rssi,
            avg_rssi=round(avg_rssi, 2) if avg_rssi is not None else None,
            min_snr=min_snr,
            max_snr=max_snr,
            avg_snr=round(avg_snr, 2) if avg_snr is not None else None,
            observed_frequency_hz=freq,
        )

    def write_session_metadata(self) -> None:
        """Serialize full session metadata JSON into session folders."""
        end_iso = datetime.now().isoformat()
        duration_s = round(time.monotonic() - self.start_mono, 2)

        vehicles_summary = {}
        for vid in VehicleID:
            vehicles_summary[vid.value] = asdict(self.get_session_summary(vid))

        meta = {
            "session_id": self.session,
            "data_source": self.data_source,
            "start_time": self.start_iso,
            "end_time": end_iso,
            "duration_seconds": duration_s,
            "status": "COMPLETED",
            "completion_status": "COMPLETED",
            "files": {
                "rocket_csv": self.filepath_for(VehicleID.ROCKET),
                "cansat_csv": self.filepath_for(VehicleID.CANSAT),
                "rocket_gs_csv": self.gs_filepath_for(VehicleID.ROCKET),
                "cansat_gs_csv": self.gs_filepath_for(VehicleID.CANSAT),
            },
            "vehicles": vehicles_summary,
            "statistics": vehicles_summary,
        }

        # Write to session root and ground_station/ subfolder
        for target_dir in (self.directory, self.gs_dir):
            target_path = os.path.join(target_dir, "session_metadata.json")
            try:
                with open(target_path, "w", encoding="utf-8") as f:
                    json.dump(meta, f, indent=2)
            except Exception as exc:
                import sys
                print(f"[ERR] Failed to write session metadata to {target_path}: {exc}", file=sys.stderr)

    def close(self) -> None:
        # Signal queue worker to stop and wait for it to flush
        if self._worker_thread and self._worker_thread.is_alive():
            self._stop_event.set()
            try:
                self._queue.put(None, timeout=0.5)
                self._worker_thread.join(timeout=3.0)
            except Exception:
                pass

        self._flush_all()
        self.write_session_metadata()

        with self._lock:
            for vid, fhs in list(self._files.items()):
                for fh in fhs:
                    try:
                        fh.flush()
                        fh.close()
                    except Exception:
                        pass
            self._files.clear()
            self._writers.clear()


# Alias for compatibility with external references
SDCardLog = SDTelemetryLogger
