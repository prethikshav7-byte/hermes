"""
ground_station/pages/csv_page.py
════════════════════════════════
CSV Telemetry Acquisition, Viewing, Graphing, and Export Console (§10).

Displays verified real-time records collected by the telemetry acquisition
and logging pipeline for both Rocket and CanSat vehicles.
Features:
- Live streaming table view of mission records across the session
- Filtering by Vehicle (All / Rocket / CanSat) and Source (All / Live Hardware / Simulation / SD Import)
- Native file export to CSV with overwrite protection and status reporting
- Synchronized time-series telemetry plots (Altitude, Velocity, Acceleration, Temperature, Voltage)
- Optional direct import of recovered onboard SD card logs
"""
from __future__ import annotations

import csv
import os
import time
from typing import Any, Dict, List, Optional, Tuple

from PyQt6.QtCore import Qt, pyqtSignal, QAbstractTableModel, QModelIndex
from PyQt6.QtGui import QColor, QFont
from PyQt6.QtWidgets import (
    QComboBox, QFileDialog, QFrame, QGridLayout, QHBoxLayout,
    QHeaderView, QLabel, QMessageBox, QPushButton, QSizePolicy,
    QSplitter, QStackedWidget, QTabWidget, QTableView, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget,
)

from .. import axes as axspec
from ..graphs.plot import TelemetryGraph
from ..models import (
    FlightState, TelemetryPacket, ValidationStatus, VehicleID,
)
from ..parser import IngestResult, LossTier
from ..theme import ThemeManager


def _btn_css(accent: str) -> str:
    c = ThemeManager.C()
    return (
        f"QPushButton {{ background-color: {c.BG_INPUT}; color: {accent}; "
        f"border: 1px solid {accent}; border-radius: 4px; padding: 6px 14px; font-weight: bold; }}"
        f"QPushButton:hover {{ background-color: {c.BG_CARD_HOVER}; }}"
        f"QPushButton:disabled {{ color: {c.TEXT_MUTED}; border-color: {c.BORDER}; }}"
    )


# ── Canonical Telemetry Record for CSV View & Export ─────────────────────────

class CsvTelemetryRecord:
    """Standardized record holding all §10.1 fields."""

    __slots__ = (
        "team_id", "mission_time", "packet_count", "vehicle_id", "altitude",
        "pressure", "temperature", "voltage", "gnss_time",
        "latitude", "longitude", "gnss_altitude", "gnss_satellites",
        "accel_magnitude", "spin_rate", "state_code", "state_name",
        "velocity", "rssi", "snr", "validation_status", "checksum",
        "source", "raw_line",
    )

    def __init__(
        self,
        team_id: str = "IN-SPACe-ROCKETRY-010",
        mission_time: float = 0.0,
        packet_count: int = 0,
        vehicle_id: str = "ROCKET",
        altitude: float = 0.0,
        pressure: float = 0.0,
        temperature: float = 0.0,
        voltage: float = 0.0,
        gnss_time: str = "—",
        latitude: float = 0.0,
        longitude: float = 0.0,
        gnss_altitude: float = 0.0,
        gnss_satellites: int = 0,
        accel_magnitude: float = 0.0,
        spin_rate: float = 0.0,
        state_code: int = 0,
        state_name: str = "BOOT",
        velocity: float = 0.0,
        rssi: Optional[float] = None,
        snr: Optional[float] = None,
        validation_status: str = "VALID",
        checksum: str = "—",
        source: str = "Live Hardware",
        raw_line: str = "",
    ):
        self.team_id = team_id
        self.mission_time = mission_time
        self.packet_count = packet_count
        self.vehicle_id = vehicle_id
        self.altitude = altitude
        self.pressure = pressure
        self.temperature = temperature
        self.voltage = voltage
        self.gnss_time = gnss_time
        self.latitude = latitude
        self.longitude = longitude
        self.gnss_altitude = gnss_altitude
        self.gnss_satellites = gnss_satellites
        self.accel_magnitude = accel_magnitude
        self.spin_rate = spin_rate
        self.state_code = state_code
        self.state_name = state_name
        self.velocity = velocity
        self.rssi = rssi
        self.snr = snr
        self.validation_status = validation_status
        self.checksum = checksum
        self.source = source
        self.raw_line = raw_line

    @classmethod
    def from_packet(
        cls,
        packet: TelemetryPacket,
        derived_v: Optional[float] = None,
        tier: LossTier = LossTier.NONE,
        source_label: Optional[str] = None,
    ) -> CsvTelemetryRecord:
        if source_label is None:
            if getattr(packet, "is_simulation", False):
                src = "Simulation"
            elif "SD" in packet.raw or "IMPORT" in packet.raw:
                src = "Imported SD"
            else:
                src = "Live Hardware"
        else:
            src = source_label

        t_id = str(getattr(packet, "team_id", None) or "IN-SPACe-ROCKETRY-010")
        st_code = packet.state.ordinal if packet.state is not None else (packet.flight_state_code or 0)
        st_name = packet.state.value if packet.state is not None else "UNKNOWN"

        v_final = packet.velocity if packet.velocity is not None else (derived_v if derived_v is not None else 0.0)
        acc_final = packet.accel_magnitude if packet.accel_magnitude is not None else 0.0
        spin_final = packet.gyro_spin_rate if packet.gyro_spin_rate is not None else (packet.gyro_z or 0.0)

        # Validation status representation
        v_status = packet.validation_status.value if hasattr(packet.validation_status, "value") else str(packet.validation_status)
        if tier is not LossTier.NONE:
            v_status = f"{v_status} ({tier.value})"

        return cls(
            team_id=t_id,
            mission_time=packet.mission_time,
            packet_count=packet.packet_count,
            vehicle_id=packet.vehicle_id.value if hasattr(packet.vehicle_id, "value") else str(packet.vehicle_id),
            altitude=packet.altitude,
            pressure=packet.pressure,
            temperature=packet.temperature,
            voltage=packet.battery_voltage if packet.battery_voltage is not None else 0.0,
            gnss_time=packet.gnss_time or "—",
            latitude=packet.gnss_latitude,
            longitude=packet.gnss_longitude,
            gnss_altitude=packet.gnss_altitude,
            gnss_satellites=packet.gnss_satellites,
            accel_magnitude=acc_final,
            spin_rate=spin_final,
            state_code=st_code,
            state_name=st_name,
            velocity=v_final,
            rssi=packet.rssi,
            snr=packet.snr,
            validation_status=v_status,
            checksum=packet.checksum or "—",
            source=src,
            raw_line=packet.raw,
        )

    def to_csv_row(self) -> List[Any]:
        return [
            self.team_id,
            f"{self.mission_time:.2f}",
            self.packet_count,
            self.vehicle_id,
            f"{self.altitude:.2f}",
            f"{self.pressure:.1f}",
            f"{self.temperature:.2f}",
            f"{self.voltage:.2f}",
            self.gnss_time,
            f"{self.latitude:.6f}",
            f"{self.longitude:.6f}",
            f"{self.gnss_altitude:.2f}",
            self.gnss_satellites,
            f"{self.accel_magnitude:.2f}",
            f"{self.spin_rate:.2f}",
            self.state_code,
            self.state_name,
            f"{self.velocity:.2f}",
            f"{self.rssi:.1f}" if self.rssi is not None else "",
            f"{self.snr:.1f}" if self.snr is not None else "",
            self.validation_status,
            self.checksum,
            self.source,
        ]


CSV_COLUMNS: List[str] = [
    "Team ID", "Mission Time (s)", "Packet Count", "Vehicle", "Altitude (m)", "Pressure (Pa)",
    "Temperature (°C)", "Voltage (V)", "GNSS Time", "Latitude (°)", "Longitude (°)",
    "GNSS Alt (m)", "Satellites", "Accel Mag (m/s²)", "Spin Rate (°/s)", "State Code",
    "State Name", "Velocity (m/s)", "RSSI (dBm)", "SNR (dB)", "Validation",
    "Checksum", "Source",
]


# ── High-Performance Table Model ─────────────────────────────────────────────

class CsvTelemetryTableModel(QAbstractTableModel):
    """Fast table model for displaying thousands of telemetry records."""

    def __init__(self, records: List[CsvTelemetryRecord], parent=None):
        super().__init__(parent)
        self._records = records

    def rowCount(self, parent=QModelIndex()) -> int:
        return len(self._records)

    def columnCount(self, parent=QModelIndex()) -> int:
        return len(CSV_COLUMNS)

    def headerData(self, section: int, orientation: Qt.Orientation, role: int = Qt.ItemDataRole.DisplayRole):
        if orientation == Qt.Orientation.Horizontal and role == Qt.ItemDataRole.DisplayRole:
            if 0 <= section < len(CSV_COLUMNS):
                return CSV_COLUMNS[section]
        return None

    def data(self, index: QModelIndex, role: int = Qt.ItemDataRole.DisplayRole):
        if not index.isValid() or not (0 <= index.row() < len(self._records)):
            return None

        rec = self._records[index.row()]
        col = index.column()

        if role == Qt.ItemDataRole.DisplayRole:
            if col == 0:  return rec.team_id
            if col == 1:  return f"{rec.mission_time:,.2f}"
            if col == 2:  return f"#{rec.packet_count:04d}"
            if col == 3:  return rec.vehicle_id
            if col == 4:  return f"{rec.altitude:,.1f}"
            if col == 5:  return f"{rec.pressure:,.0f}"
            if col == 6:  return f"{rec.temperature:,.1f}"
            if col == 7:  return f"{rec.voltage:.2f}"
            if col == 8:  return rec.gnss_time
            if col == 9:  return f"{rec.latitude:.6f}"
            if col == 10: return f"{rec.longitude:.6f}"
            if col == 11: return f"{rec.gnss_altitude:,.1f}"
            if col == 12: return str(rec.gnss_satellites)
            if col == 13: return f"{rec.accel_magnitude:.2f}"
            if col == 14: return f"{rec.spin_rate:.2f}"
            if col == 15: return str(rec.state_code)
            if col == 16: return rec.state_name
            if col == 17: return f"{rec.velocity:,.1f}"
            if col == 18: return f"{rec.rssi:.1f}" if rec.rssi is not None else "—"
            if col == 19: return f"{rec.snr:.1f}" if rec.snr is not None else "—"
            if col == 20: return rec.validation_status
            if col == 21: return rec.checksum
            if col == 22: return rec.source

        if role == Qt.ItemDataRole.TextAlignmentRole:
            if col in (1, 2, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 17, 18, 19):
                return Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter
            return Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter

        if role == Qt.ItemDataRole.ForegroundRole:
            c = ThemeManager.C()
            if col == 3:   # Vehicle
                return QColor(c.CYAN if rec.vehicle_id == "ROCKET" else c.AMBER)
            if col == 16:  # State name
                if rec.state_name == "APOGEE": return QColor(c.CYAN)
                if rec.state_name in ("BOOST", "DESCENT"): return QColor(c.AMBER)
                if rec.state_name in ("LANDING", "RECOVERY"): return QColor(c.GREEN)
                return QColor(c.TEXT)
            if col == 20:  # Validation
                return QColor(c.GREEN if "VALID" in rec.validation_status else c.RED)
            if col == 22:  # Source
                return QColor(c.GREEN if rec.source == "Live Hardware" else (c.CYAN if rec.source == "Simulation" else c.AMBER))
            return QColor(c.TEXT)

        return None


# ── The CSV Page ─────────────────────────────────────────────────────────────

class CsvPage(QWidget):
    """§10 — CSV Data Inspection, Export, and Graphing Page."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._all_records: List[CsvTelemetryRecord] = []
        self._filtered_records: List[CsvTelemetryRecord] = []
        self._seen_keys = set()
        self._supervisor = None

        self._filter_vehicle = "ALL"
        self._filter_source = "ALL"

        self._build_ui()

    def _build_ui(self) -> None:
        c = ThemeManager.C()
        outer = QVBoxLayout(self)
        outer.setContentsMargins(14, 12, 14, 12)
        outer.setSpacing(10)

        # ── Header & Action Controls ─────────────────────────────────────────
        header_row = QHBoxLayout()
        header_row.setSpacing(12)

        title = QLabel("CSV TELEMETRY ACQUISITION & LOGGING")
        title.setFont(QFont("Segoe UI", 12, QFont.Weight.Bold))
        title.setStyleSheet(f"color: {c.CYAN}; letter-spacing: 1px;")
        header_row.addWidget(title)

        header_row.addStretch()

        # Vehicle filter
        header_row.addWidget(QLabel("Vehicle:"))
        self.vehicle_combo = QComboBox()
        self.vehicle_combo.setFont(QFont("Segoe UI", 9))
        self.vehicle_combo.setStyleSheet(self._combo_css())
        self.vehicle_combo.addItems(["ALL VEHICLES", "ROCKET", "CANSAT"])
        self.vehicle_combo.currentTextChanged.connect(self._on_filter_changed)
        header_row.addWidget(self.vehicle_combo)

        # Source filter
        header_row.addWidget(QLabel("Source:"))
        self.source_combo = QComboBox()
        self.source_combo.setFont(QFont("Segoe UI", 9))
        self.source_combo.setStyleSheet(self._combo_css())
        self.source_combo.addItems(["ALL SOURCES", "Live Hardware", "Simulation", "Imported SD"])
        self.source_combo.currentTextChanged.connect(self._on_filter_changed)
        header_row.addWidget(self.source_combo)

        # Import SD log button
        import_btn = QPushButton("Import SD Flight Log...")
        import_btn.setFont(QFont("Segoe UI", 9))
        import_btn.setStyleSheet(_btn_css(c.CYAN))
        import_btn.setToolTip("Import physically recorded SD-card log from recovered hardware")
        import_btn.clicked.connect(self._on_import_sd_clicked)
        header_row.addWidget(import_btn)

        # Download CSV button (§2.C)
        self.download_btn = QPushButton("Download CSV")
        self.download_btn.setFont(QFont("Segoe UI", 9))
        self.download_btn.setStyleSheet(_btn_css(c.GREEN))
        self.download_btn.setToolTip("Download collected telemetry records as a CSV file")
        self.download_btn.clicked.connect(self._on_download_csv_clicked)
        header_row.addWidget(self.download_btn)

        outer.addLayout(header_row)

        # ── Status strip ─────────────────────────────────────────────────────
        status_card = QFrame()
        status_card.setStyleSheet(
            f"QFrame {{ background-color: {c.BG_PANEL}; border: 1px solid {c.BORDER}; border-radius: 6px; }}"
        )
        status_lay = QHBoxLayout(status_card)
        status_lay.setContentsMargins(12, 6, 12, 6)
        status_lay.setSpacing(16)

        self.lbl_total_records = QLabel("Records: 0")
        self.lbl_total_records.setFont(QFont("monospace", 8, QFont.Weight.Bold))
        self.lbl_total_records.setStyleSheet(f"color: {c.TEXT};")
        status_lay.addWidget(self.lbl_total_records)

        self.lbl_filtered_records = QLabel("Showing: 0")
        self.lbl_filtered_records.setFont(QFont("monospace", 8))
        self.lbl_filtered_records.setStyleSheet(f"color: {c.TEXT_DIM};")
        status_lay.addWidget(self.lbl_filtered_records)

        self.lbl_last_time = QLabel("Latest: T+ — s")
        self.lbl_last_time.setFont(QFont("monospace", 8))
        self.lbl_last_time.setStyleSheet(f"color: {c.CYAN};")
        status_lay.addWidget(self.lbl_last_time)

        self.lbl_export_status = QLabel("")
        self.lbl_export_status.setFont(QFont("Segoe UI", 8))
        self.lbl_export_status.setStyleSheet(f"color: {c.GREEN}; font-weight: bold;")
        status_lay.addWidget(self.lbl_export_status)

        status_lay.addStretch()

        outer.addWidget(status_card)

        # ── Direct Telemetry Records Table (No separate charts or graphs per §2) ──
        self.table_view = QTableView()
        self.model = CsvTelemetryTableModel(self._filtered_records, self)
        self.table_view.setModel(self.model)
        self.table_view.setFont(QFont("monospace", 8))
        self.table_view.setStyleSheet(
            f"QTableView {{ background-color: {c.BG_CARD}; color: {c.TEXT}; "
            f"gridline-color: {c.BORDER}; border: 1px solid {c.BORDER}; border-radius: 6px; selection-background-color: #122840; }}"
            f"QHeaderView::section {{ background-color: {c.BG_INPUT}; color: {c.TEXT_DIM}; "
            f"border: 1px solid {c.BORDER}; padding: 4px 6px; font-weight: bold; }}"
        )
        self.table_view.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.table_view.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.table_view.horizontalHeader().setStretchLastSection(True)
        self.table_view.setAlternatingRowColors(True)
        self.table_view.verticalHeader().setVisible(False)
        self.table_view.setSelectionBehavior(QTableView.SelectionBehavior.SelectRows)

        outer.addWidget(self.table_view, stretch=1)

    def _combo_css(self) -> str:
        c = ThemeManager.C()
        return (
            f"QComboBox {{ background-color: {c.BG_INPUT}; color: {c.TEXT}; "
            f"border: 1px solid {c.BORDER}; border-radius: 4px; padding: 4px 8px; min-width: 130px; }}"
            f"QComboBox QAbstractItemView {{ background-color: {c.BG_CARD}; color: {c.TEXT}; "
            f"selection-background-color: #122840; border: 1px solid {c.BORDER}; }}"
        )

    # ── Telemetry Reception ──────────────────────────────────────────────────

    def receive_packet(
        self,
        packet: TelemetryPacket,
        result: Optional[IngestResult] = None,
        derived_v: Optional[float] = None,
    ) -> None:
        """Receive live or replayed telemetry record from the pipeline."""
        tier = result.tier if result is not None else LossTier.NONE
        rec = CsvTelemetryRecord.from_packet(packet, derived_v=derived_v, tier=tier)
        self.add_record(rec)

    def add_record(self, rec: CsvTelemetryRecord) -> None:
        """Append one record if not duplicate, and update table view."""
        # De-duplication check (§2.E): avoid duplicate records from UI redraws
        dedup_key = (rec.vehicle_id, rec.packet_count, round(rec.mission_time, 2), rec.source)
        if dedup_key in self._seen_keys:
            return
        self._seen_keys.add(dedup_key)

        self._all_records.append(rec)
        self.lbl_total_records.setText(f"Records: {len(self._all_records)}")
        self.lbl_last_time.setText(f"Latest: T+{rec.mission_time:.2f} s")

        # Check filter
        if self._matches_filter(rec):
            self.model.beginInsertRows(QModelIndex(), 0, 0)
            self._filtered_records.insert(0, rec)
            self.model.endInsertRows()
            self.lbl_filtered_records.setText(f"Showing: {len(self._filtered_records)}")

    def _matches_filter(self, rec: CsvTelemetryRecord) -> bool:
        if self._filter_vehicle != "ALL" and rec.vehicle_id != self._filter_vehicle:
            return False
        if self._filter_source != "ALL" and rec.source != self._filter_source:
            return False
        return True

    def _on_filter_changed(self) -> None:
        veh = self.vehicle_combo.currentText().upper()
        if "ROCKET" in veh:
            self._filter_vehicle = "ROCKET"
        elif "CANSAT" in veh:
            self._filter_vehicle = "CANSAT"
        else:
            self._filter_vehicle = "ALL"

        src = self.source_combo.currentText()
        if "Live" in src:
            self._filter_source = "Live Hardware"
        elif "Simulation" in src:
            self._filter_source = "Simulation"
        elif "SD" in src:
            self._filter_source = "Imported SD"
        else:
            self._filter_source = "ALL"

        self._apply_filters()

    def _apply_filters(self) -> None:
        self.model.beginResetModel()
        self._filtered_records.clear()
        for r in reversed(self._all_records):
            if self._matches_filter(r):
                self._filtered_records.append(r)
        self.model.endResetModel()
        self.lbl_filtered_records.setText(f"Showing: {len(self._filtered_records)}")

    # ── Export & Import ──────────────────────────────────────────────────────

    def _on_download_csv_clicked(self) -> None:
        """Download collected telemetry records to a CSV file using native file dialog (§2.C)."""
        if not self._all_records:
            QMessageBox.information(
                self, "Download CSV", "No telemetry records currently available to download."
            )
            return

        records_to_write = list(reversed(self._filtered_records)) if self._filtered_records else self._all_records
        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        veh_tag = self._filter_vehicle.lower() if self._filter_vehicle != "ALL" else "all_vehicles"
        default_name = f"machzone_telemetry_{veh_tag}_{timestamp_str}.csv"

        filename, _ = QFileDialog.getSaveFileName(
            self, "Download Telemetry CSV", default_name, "CSV Files (*.csv);;All Files (*)"
        )
        if not filename:
            return

        # Overwrite safety check if file already exists
        if os.path.exists(filename):
            reply = QMessageBox.question(
                self,
                "Confirm Overwrite",
                f"The file '{os.path.basename(filename)}' already exists.\nDo you want to overwrite it?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if reply != QMessageBox.StandardButton.Yes:
                return

        try:
            with open(filename, "w", newline="", encoding="utf-8") as fh:
                writer = csv.writer(fh)
                writer.writerow(CSV_COLUMNS)
                for rec in records_to_write:
                    writer.writerow(rec.to_csv_row())

            count = len(records_to_write)
            self.lbl_export_status.setText(f"✓ Downloaded {count} records to {os.path.basename(filename)}")
            QMessageBox.information(
                self,
                "Download Complete",
                f"Successfully downloaded {count} telemetry records to:\n{filename}",
            )
        except Exception as e:
            self.lbl_export_status.setText(f"Download error: {e}")
            QMessageBox.critical(
                self, "Download Failed", f"Could not write CSV file:\n{e}"
            )

    # Alias for compatibility with previous callers
    _on_export_csv_clicked = _on_download_csv_clicked

    def _on_import_sd_clicked(self) -> None:
        """Allow operator to select a recovered SD-card flight log directly (§10.2, §13)."""
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Select Recovered SD Card Flight Log",
            "",
            "CSV Files (*.csv);;Text Logs (*.txt *.log);;All Files (*)",
        )
        if not filename:
            return

        from ..logs.post_flight import PostFlightAnalyzer
        try:
            result = PostFlightAnalyzer.analyze_file(filename)
            records = PostFlightAnalyzer.extract_records_from_file(filename)
            for rec in records:
                self.add_record(rec)

            self.lbl_export_status.setText(
                f"Imported {len(records)} records from {os.path.basename(filename)}"
            )
            stats = result.stats
            QMessageBox.information(
                self,
                "SD Card Import Complete",
                f"Loaded {len(records)} valid records from:\n{filename}\n\n"
                f"• Maximum Altitude: {stats.max_altitude if stats.max_altitude is not None else 'N/A'} m\n"
                f"• Maximum Velocity: {stats.max_velocity if stats.max_velocity is not None else 'N/A'} m/s\n"
                f"• Max Accel Magnitude: {stats.max_acceleration if stats.max_acceleration is not None else 'N/A'} m/s²\n"
                f"• Min / Max Temp: {stats.max_temperature} °C\n"
                f"• Valid Packets: {stats.total_valid_packets}\n"
                f"• Missing Packets: {stats.missing_packets}",
            )
        except Exception as e:
            QMessageBox.critical(
                self, "Import Failed", f"Could not parse SD-card flight file:\n{e}"
            )
