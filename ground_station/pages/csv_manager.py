"""
ground_station/pages/csv_manager.py
═══════════════════════════════════
Flight Data & Telemetry Logging Console (§10, §12, §18–§20).

Provides a comprehensive, dark-mode MachZone-styled workspace for:
- Browsing all real-time and historical flight telemetry sessions
- Identifying session source: HARDWARE, SIMULATION, or IMPORTED_ONBOARD
- Distinguishing Rocket and CanSat telemetry files
- Viewing raw and decoded CSV telemetry records in an optimized read-only table
- Native OS export of verified CSV flight logs (with overwrite protection)
- Opening session folders natively in macOS Finder / Windows Explorer
- Importing recovered onboard SD-card logs for post-flight analysis
"""
from __future__ import annotations

import csv
import json
import os
import shutil
from typing import Any, Dict, List, Optional, Tuple

from PyQt6.QtCore import (
    QAbstractTableModel, QModelIndex, QSize, Qt, QUrl, pyqtSignal,
)
from PyQt6.QtGui import QColor, QDesktopServices, QFont
from PyQt6.QtWidgets import (
    QComboBox, QFileDialog, QFrame, QGridLayout, QHBoxLayout,
    QHeaderView, QLabel, QLineEdit, QListWidget, QListWidgetItem,
    QMessageBox, QPushButton, QSizePolicy, QSplitter, QStackedWidget,
    QTableView, QVBoxLayout, QWidget,
)

from ..logs.post_flight import PostFlightAnalyzer
from ..logs.raw_log import get_default_storage_dir
from ..theme import ThemeManager


def _btn_css(accent: str) -> str:
    c = ThemeManager.C()
    return (
        f"QPushButton {{ background-color: {c.BG_INPUT}; color: {accent}; "
        f"border: 1px solid {accent}; border-radius: 5px; padding: 6px 14px; font-weight: bold; "
        f"font-family: 'Segoe UI', Arial, sans-serif; }}"
        f"QPushButton:hover {{ background-color: {c.BG_CARD_HOVER}; }}"
        f"QPushButton:disabled {{ color: {c.TEXT_MUTED}; border-color: {c.BORDER}; }}"
    )


# ── High-Performance Read-Only CSV Table Model ───────────────────────────────

class CsvFileTableModel(QAbstractTableModel):
    """Memory-efficient table model for viewing thousands of telemetry rows."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.headers: List[str] = []
        self.rows: List[List[str]] = []
        self.total_disk_rows: int = 0
        self.filepath: str = ""

    def rowCount(self, parent=QModelIndex()) -> int:
        return len(self.rows)

    def columnCount(self, parent=QModelIndex()) -> int:
        return len(self.headers)

    def headerData(self, section: int, orientation: Qt.Orientation, role: int = Qt.ItemDataRole.DisplayRole):
        if orientation == Qt.Orientation.Horizontal and role == Qt.ItemDataRole.DisplayRole:
            if 0 <= section < len(self.headers):
                return self.headers[section]
        elif orientation == Qt.Orientation.Vertical and role == Qt.ItemDataRole.DisplayRole:
            return str(section + 1)
        return None

    def data(self, index: QModelIndex, role: int = Qt.ItemDataRole.DisplayRole):
        if not index.isValid() or not (0 <= index.row() < len(self.rows)):
            return None

        row_data = self.rows[index.row()]
        col = index.column()

        if role == Qt.ItemDataRole.DisplayRole:
            if 0 <= col < len(row_data):
                return row_data[col]
            return ""

        if role == Qt.ItemDataRole.TextAlignmentRole:
            return int(Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft)

        return None

    def load_file(self, path: str, max_rows: int = 10000) -> Tuple[int, Optional[str]]:
        """Load headers and up to max_rows from the CSV file."""
        self.beginResetModel()
        self.headers.clear()
        self.rows.clear()
        self.total_disk_rows = 0
        self.filepath = path

        if not path or not os.path.exists(path):
            self.endResetModel()
            return 0, "File does not exist."

        try:
            with open(path, "r", encoding="utf-8", errors="replace") as fh:
                reader = csv.reader(fh)
                first_row = next(reader, None)
                if not first_row:
                    self.endResetModel()
                    return 0, None  # Empty file

                self.headers = [h.strip() for h in first_row]

                line_count = 0
                for row in reader:
                    if not row or not any(field.strip() for field in row):
                        continue
                    line_count += 1
                    if len(self.rows) < max_rows:
                        self.rows.append(row)

                self.total_disk_rows = line_count
            self.endResetModel()
            return self.total_disk_rows, None
        except Exception as exc:
            self.endResetModel()
            return 0, str(exc)


# ── Flight Data Manager Page ─────────────────────────────────────────────────

class CsvManagerPage(QWidget):
    """Aerospace Telemetry File & Session Manager."""

    def __init__(self, log_dir: Optional[str] = None, parent=None):
        super().__init__(parent)
        self.custom_log_dir = log_dir
        self.active_session_dir: Optional[str] = None
        self.selected_session_dir: Optional[str] = None
        self.selected_file_path: Optional[str] = None

        self._model = CsvFileTableModel(self)
        self._build_ui()
        self.refresh_sessions()

    def set_session_logs(self, session_logs) -> None:
        """Connect active session logs to highlight current flight."""
        if session_logs and hasattr(session_logs, "directory"):
            self.active_session_dir = session_logs.directory
            self.refresh_sessions()

    def _build_ui(self) -> None:
        c = ThemeManager.C()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 14, 16, 14)
        layout.setSpacing(10)

        # ── 1. Top Header & Action Controls ──────────────────────────────────
        top_bar = QHBoxLayout()
        top_bar.setSpacing(12)

        title_lay = QVBoxLayout()
        title_lay.setSpacing(2)
        h_title = QLabel("FLIGHT DATA & TELEMETRY LOGS")
        h_title.setFont(QFont("Segoe UI", 12, QFont.Weight.Bold))
        h_title.setStyleSheet("color: #ffffff; letter-spacing: 1.5px;")

        h_sub = QLabel("Automatic Session Storage, Decoded CSV Inspection & Native Export")
        h_sub.setFont(QFont("Segoe UI", 8))
        h_sub.setStyleSheet(f"color: {c.TEXT_DIM};")
        title_lay.addWidget(h_title)
        title_lay.addWidget(h_sub)
        top_bar.addLayout(title_lay)

        top_bar.addStretch()

        # Action Buttons
        self._refresh_btn = QPushButton("Refresh")
        self._refresh_btn.setStyleSheet(_btn_css(c.TEXT))
        self._refresh_btn.clicked.connect(self.refresh_sessions)
        top_bar.addWidget(self._refresh_btn)

        self._open_folder_btn = QPushButton("Open Session Folder")
        self._open_folder_btn.setStyleSheet(_btn_css(c.CYAN))
        self._open_folder_btn.clicked.connect(self.open_current_session_folder)
        top_bar.addWidget(self._open_folder_btn)

        self._import_sd_btn = QPushButton("Import SD Card Log...")
        self._import_sd_btn.setStyleSheet(_btn_css(c.AMBER))
        self._import_sd_btn.clicked.connect(self.import_onboard_sd_log)
        top_bar.addWidget(self._import_sd_btn)

        self._export_btn = QPushButton("Export CSV...")
        self._export_btn.setStyleSheet(_btn_css(c.GREEN))
        self._export_btn.clicked.connect(self.export_selected_csv)
        top_bar.addWidget(self._export_btn)

        layout.addLayout(top_bar)

        # Subtle divider
        div = QFrame()
        div.setFrameShape(QFrame.Shape.HLine)
        div.setStyleSheet(f"color: {c.BORDER};")
        layout.addWidget(div)

        # ── 2. Splitter (Left: Sessions & Files, Right: File Preview) ──────────
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setStyleSheet(
            f"QSplitter::handle {{ background-color: {c.BORDER}; width: 2px; }}"
        )

        # ── Left Navigation Panel ──
        left_panel = QWidget()
        left_lay = QVBoxLayout(left_panel)
        left_lay.setContentsMargins(0, 0, 8, 0)
        left_lay.setSpacing(8)

        # Filter row
        filter_row = QHBoxLayout()
        filter_row.setSpacing(6)

        self._search_input = QLineEdit()
        self._search_input.setPlaceholderText("Search sessions...")
        self._search_input.setFont(QFont("Segoe UI", 8))
        self._search_input.setStyleSheet(
            f"QLineEdit {{ background-color: {c.BG_INPUT}; color: {c.TEXT}; "
            f"border: 1px solid {c.BORDER}; border-radius: 4px; padding: 4px 8px; }}"
        )
        self._search_input.textChanged.connect(self._apply_filter)
        filter_row.addWidget(self._search_input)

        self._source_filter = QComboBox()
        self._source_filter.setFont(QFont("Segoe UI", 8))
        self._source_filter.setStyleSheet(
            f"QComboBox {{ background-color: {c.BG_INPUT}; color: {c.TEXT}; "
            f"border: 1px solid {c.BORDER}; border-radius: 4px; padding: 4px 8px; }}"
        )
        self._source_filter.addItems(["All Sources", "Hardware Only", "Simulation Only", "Imported Onboard"])
        self._source_filter.currentIndexChanged.connect(self._apply_filter)
        filter_row.addWidget(self._source_filter)
        left_lay.addLayout(filter_row)

        # Sessions list
        s_lbl = QLabel("RECORDED SESSIONS")
        s_lbl.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        s_lbl.setStyleSheet(f"color: {c.TEXT_DIM}; letter-spacing: 1px;")
        left_lay.addWidget(s_lbl)

        self._session_list = QListWidget()
        self._session_list.setFont(QFont("Segoe UI", 9))
        self._session_list.setStyleSheet(
            f"QListWidget {{ background-color: {c.BG_CARD}; color: {c.TEXT}; "
            f"border: 1px solid {c.BORDER}; border-radius: 6px; padding: 4px; }}"
            f"QListWidget::item {{ padding: 8px 10px; margin: 2px 0px; border-radius: 4px; }}"
            f"QListWidget::item:hover {{ background-color: {c.BG_CARD_HOVER}; }}"
            f"QListWidget::item:selected {{ background-color: #0e263d; border: 1px solid {c.CYAN}; color: {c.CYAN}; }}"
        )
        self._session_list.currentItemChanged.connect(self._on_session_selected)
        left_lay.addWidget(self._session_list, stretch=1)

        # Available files in selected session
        f_lbl = QLabel("SESSION FILES")
        f_lbl.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        f_lbl.setStyleSheet(f"color: {c.TEXT_DIM}; letter-spacing: 1px;")
        left_lay.addWidget(f_lbl)

        self._file_list = QListWidget()
        self._file_list.setFont(QFont("Segoe UI", 8))
        self._file_list.setStyleSheet(
            f"QListWidget {{ background-color: {c.BG_INPUT}; color: {c.TEXT}; "
            f"border: 1px solid {c.BORDER}; border-radius: 6px; padding: 4px; }}"
            f"QListWidget::item {{ padding: 6px 8px; margin: 2px 0px; border-radius: 4px; }}"
            f"QListWidget::item:hover {{ background-color: {c.BG_CARD_HOVER}; }}"
            f"QListWidget::item:selected {{ background-color: #1a334d; color: {c.CYAN}; font-weight: bold; }}"
        )
        self._file_list.currentItemChanged.connect(self._on_file_selected)
        left_lay.addWidget(self._file_list, stretch=1)

        splitter.addWidget(left_panel)

        # ── Right Preview Panel ──
        right_panel = QWidget()
        right_lay = QVBoxLayout(right_panel)
        right_lay.setContentsMargins(8, 0, 0, 0)
        right_lay.setSpacing(8)

        # Metadata metrics bar
        self._meta_card = QFrame()
        self._meta_card.setStyleSheet(
            f"QFrame {{ background-color: {c.BG_CARD}; border: 1px solid {c.BORDER}; border-radius: 6px; padding: 6px 12px; }}"
        )
        meta_row = QHBoxLayout(self._meta_card)
        meta_row.setContentsMargins(8, 6, 8, 6)
        meta_row.setSpacing(16)

        self._lbl_file = QLabel("No file selected")
        self._lbl_file.setFont(QFont("Segoe UI", 9, QFont.Weight.Bold))
        self._lbl_file.setStyleSheet(f"color: {c.TEXT};")
        meta_row.addWidget(self._lbl_file)

        self._lbl_source_badge = QLabel("SOURCE: —")
        self._lbl_source_badge.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        self._lbl_source_badge.setStyleSheet(
            f"background-color: {c.BG_INPUT}; color: {c.TEXT_DIM}; border: 1px solid {c.BORDER}; border-radius: 3px; padding: 2px 8px;"
        )
        meta_row.addWidget(self._lbl_source_badge)

        self._lbl_rows = QLabel("Rows: —")
        self._lbl_rows.setFont(QFont("Segoe UI", 8))
        self._lbl_rows.setStyleSheet(f"color: {c.CYAN};")
        meta_row.addWidget(self._lbl_rows)

        self._lbl_size = QLabel("Size: —")
        self._lbl_size.setFont(QFont("Segoe UI", 8))
        self._lbl_size.setStyleSheet(f"color: {c.TEXT_DIM};")
        meta_row.addWidget(self._lbl_size)

        meta_row.addStretch()
        right_lay.addWidget(self._meta_card)

        # Table View
        self._table_view = QTableView()
        self._table_view.setModel(self._model)
        self._table_view.setStyleSheet(
            f"QTableView {{ background-color: {c.BG_CARD}; color: {c.TEXT}; "
            f"gridline-color: {c.BORDER}; border: 1px solid {c.BORDER}; border-radius: 6px; }}"
            f"QTableView::item:selected {{ background-color: #0e263d; color: {c.CYAN}; }}"
            f"QHeaderView::section {{ background-color: {c.BG_PANEL}; color: {c.TEXT_DIM}; "
            f"font-weight: bold; border: 1px solid {c.BORDER}; padding: 6px; }}"
        )
        self._table_view.horizontalHeader().setStretchLastSection(True)
        self._table_view.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Interactive)
        self._table_view.setAlternatingRowColors(True)
        self._table_view.verticalHeader().setDefaultSectionSize(26)
        right_lay.addWidget(self._table_view, stretch=1)

        # Status footer bar
        self._status_bar = QLabel("Ready")
        self._status_bar.setFont(QFont("Segoe UI", 8))
        self._status_bar.setStyleSheet(f"color: {c.TEXT_MUTED};")
        right_lay.addWidget(self._status_bar)

        splitter.addWidget(right_panel)
        splitter.setSizes([320, 880])
        layout.addWidget(splitter, stretch=1)

    # ── Session & File Discovery ─────────────────────────────────────────────

    def _get_search_dirs(self) -> List[str]:
        dirs = []
        if self.custom_log_dir and os.path.exists(self.custom_log_dir):
            dirs.append(self.custom_log_dir)

        default_dir = get_default_storage_dir()
        if os.path.exists(default_dir) and default_dir not in dirs:
            dirs.append(default_dir)

        local_logs = os.path.abspath("logs")
        if os.path.exists(local_logs) and local_logs not in dirs:
            dirs.append(local_logs)

        return dirs

    def refresh_sessions(self) -> None:
        """Scan log roots for available sessions."""
        self._session_list.clear()
        search_dirs = self._get_search_dirs()
        sessions: List[Tuple[str, str, Dict[str, Any]]] = []

        for base_dir in search_dirs:
            if not os.path.isdir(base_dir):
                continue
            for entry in os.listdir(base_dir):
                full_path = os.path.join(base_dir, entry)
                if os.path.isdir(full_path):
                    # Check for session indicators
                    has_csv = any(f.endswith(".csv") or f.endswith(".log") for f in os.listdir(full_path))
                    gs_subdir = os.path.join(full_path, "ground_station")
                    has_gs_subdir = os.path.isdir(gs_subdir)
                    if has_csv or has_gs_subdir:
                        meta_file = os.path.join(full_path, "session_metadata.json")
                        meta: Dict[str, Any] = {}
                        if os.path.exists(meta_file):
                            try:
                                with open(meta_file, "r", encoding="utf-8") as mf:
                                    meta = json.load(mf)
                            except Exception:
                                pass
                        sessions.append((entry, full_path, meta))

        # Sort newest first
        sessions.sort(key=lambda s: s[0], reverse=True)

        filter_choice = self._source_filter.currentText()
        query = self._search_input.text().strip().lower() if hasattr(self, "_search_input") else ""
        for s_id, s_path, meta in sessions:
            src = meta.get("data_source", "HARDWARE")
            if "SIM" in s_id.upper():
                src = "SIMULATION"

            # Apply filter
            if filter_choice == "Hardware Only" and src != "HARDWARE":
                continue
            if filter_choice == "Simulation Only" and src != "SIMULATION":
                continue
            if filter_choice == "Imported Onboard" and src != "IMPORTED_ONBOARD":
                continue
            if query and query not in s_id.lower() and query not in src.lower():
                continue

            # Format title
            is_active = (s_path == self.active_session_dir)
            active_prefix = "● [LIVE] " if is_active else ""
            display_text = f"{active_prefix}{s_id} [{src}]"

            item = QListWidgetItem(display_text)
            item.setData(Qt.ItemDataRole.UserRole, (s_path, src, meta))
            if is_active:
                item.setForeground(QColor("#22c55e"))
            elif src == "SIMULATION":
                item.setForeground(QColor("#38bdf8"))
            elif src == "IMPORTED_ONBOARD":
                item.setForeground(QColor("#f59e0b"))

            self._session_list.addItem(item)

        if self._session_list.count() > 0:
            self._session_list.setCurrentRow(0)
        else:
            self._file_list.clear()
            self._lbl_file.setText("No sessions found")
            self._model.load_file("")
            self._status_bar.setText("No flight session folders found.")

    def _apply_filter(self) -> None:
        self.refresh_sessions()

    def _on_session_selected(self, current: Optional[QListWidgetItem], previous: Optional[QListWidgetItem] = None) -> None:
        if not current:
            return

        s_path, src, meta = current.data(Qt.ItemDataRole.UserRole)
        self.selected_session_dir = s_path
        self._populate_session_files(s_path, src)

    def _populate_session_files(self, session_path: str, src: str) -> None:
        self._file_list.clear()
        if not os.path.exists(session_path):
            return

        c = ThemeManager.C()
        # Scan root and ground_station / onboard subdirectories
        candidates: List[Tuple[str, str, str]] = []  # (label, rel_path, full_path)

        def add_dir(dirpath: str, category: str):
            if not os.path.isdir(dirpath):
                return
            for fname in sorted(os.listdir(dirpath)):
                fpath = os.path.join(dirpath, fname)
                if not os.path.isfile(fpath):
                    continue
                if fname.endswith(".csv"):
                    if "ROCKET" in fname.upper():
                        candidates.append((f"🚀 Rocket Telemetry ({category}): {fname}", fname, fpath))
                    elif "CANSAT" in fname.upper():
                        candidates.append((f"🛰️ CanSat Telemetry ({category}): {fname}", fname, fpath))
                    else:
                        candidates.append((f"📊 Telemetry CSV ({category}): {fname}", fname, fpath))
                elif fname.endswith(".log"):
                    candidates.append((f"📋 Raw Frame Log ({category}): {fname}", fname, fpath))
                elif fname.endswith(".json"):
                    candidates.append((f"⚙️ Session Metadata: {fname}", fname, fpath))

        # Check ground_station subfolder first, then root
        gs_sub = os.path.join(session_path, "ground_station")
        if os.path.isdir(gs_sub):
            add_dir(gs_sub, "GS")
        add_dir(session_path, "Root")

        # Check onboard subfolder
        onboard_sub = os.path.join(session_path, "onboard")
        if os.path.isdir(onboard_sub):
            add_dir(onboard_sub, "Onboard SD")

        for label, rel, fpath in candidates:
            item = QListWidgetItem(label)
            item.setData(Qt.ItemDataRole.UserRole, (fpath, src))
            self._file_list.addItem(item)

        if self._file_list.count() > 0:
            self._file_list.setCurrentRow(0)
        else:
            self._lbl_file.setText("No data files in session")
            self._model.load_file("")

    def _on_file_selected(self, current: Optional[QListWidgetItem], previous: Optional[QListWidgetItem] = None) -> None:
        if not current:
            return

        fpath, src = current.data(Qt.ItemDataRole.UserRole)
        self.selected_file_path = fpath
        self._load_file_into_table(fpath, src)

    def _load_file_into_table(self, fpath: str, src: str) -> None:
        c = ThemeManager.C()
        fname = os.path.basename(fpath)
        self._lbl_file.setText(fname)

        # Source badge color
        badge_color = c.GREEN if src == "HARDWARE" else (c.CYAN if src == "SIMULATION" else c.AMBER)
        self._lbl_source_badge.setText(f"● {src}")
        self._lbl_source_badge.setStyleSheet(
            f"background-color: {c.BG_INPUT}; color: {badge_color}; "
            f"border: 1px solid {badge_color}; border-radius: 4px; padding: 2px 8px; font-weight: bold;"
        )

        # Size
        try:
            sz = os.path.getsize(fpath)
            sz_str = f"{sz / 1024:.1f} KB" if sz < 1024 * 1024 else f"{sz / (1024 * 1024):.2f} MB"
        except Exception:
            sz_str = "—"
        self._lbl_size.setText(f"Size: {sz_str}")

        if not fpath.endswith(".csv"):
            # Non-csv (raw log or json) preview in model as single column
            self._load_text_file(fpath)
            return

        # Load CSV into model
        rows_read, err = self._model.load_file(fpath)
        if err:
            self._status_bar.setText(f"Error reading file: {err}")
            self._lbl_rows.setText("Error")
            return

        self._lbl_rows.setText(f"Records: {rows_read:,} rows")
        self._status_bar.setText(f"Loaded {rows_read:,} verified records from {fpath}")

    def _load_text_file(self, fpath: str) -> None:
        try:
            with open(fpath, "r", encoding="utf-8", errors="replace") as f:
                lines = [line.rstrip("\r\n") for line in f.readlines()[:1000]]
            self._model.beginResetModel()
            self._model.headers = ["Log Record"]
            self._model.rows = [[l] for l in lines]
            self._model.endResetModel()
            self._lbl_rows.setText(f"Lines: {len(lines):,}")
            self._status_bar.setText(f"Displaying raw text preview of {fpath}")
        except Exception as e:
            self._status_bar.setText(f"Failed to read file: {e}")

    # ── Action Controls ──────────────────────────────────────────────────────

    def open_current_session_folder(self) -> None:
        """Open the currently selected or active session in macOS Finder or Windows Explorer."""
        target_dir = self.selected_session_dir or self.active_session_dir or get_default_storage_dir()
        if not target_dir or not os.path.exists(target_dir):
            QMessageBox.warning(self, "Folder Not Found", f"The directory does not exist:\n{target_dir}")
            return

        QDesktopServices.openUrl(QUrl.fromLocalFile(target_dir))
        self._status_bar.setText(f"Opened folder in file manager: {target_dir}")

    def export_selected_csv(self) -> None:
        """Export the currently selected CSV flight file to operator's choice of location."""
        if not self.selected_file_path or not os.path.exists(self.selected_file_path):
            QMessageBox.warning(self, "Export Failed", "Please select a valid CSV telemetry file to export.")
            return

        base_name = os.path.basename(self.selected_file_path)
        save_path, _ = QFileDialog.getSaveFileName(
            self, "Export Telemetry Flight Log", base_name, "CSV Files (*.csv);;All Files (*)"
        )
        if not save_path:
            return  # Cancelled

        try:
            shutil.copyfile(self.selected_file_path, save_path)
            QMessageBox.information(
                self, "Export Complete",
                f"Successfully exported telemetry log to:\n{save_path}"
            )
            self._status_bar.setText(f"Exported {base_name} -> {save_path}")
        except Exception as exc:
            QMessageBox.critical(self, "Export Error", f"Failed to export file:\n{exc}")
            self._status_bar.setText(f"Export error: {exc}")

    def import_onboard_sd_log(self) -> None:
        """Import recovered physical onboard microSD flight log into session archive."""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Select Recovered Onboard SD-Card Log", "", "CSV Files (*.csv);;All Files (*)"
        )
        if not filename:
            return

        if not os.path.exists(filename):
            QMessageBox.warning(self, "Import Error", "Selected file does not exist.")
            return

        # Validate with PostFlightAnalyzer
        try:
            analysis = PostFlightAnalyzer.analyze_file(filename)
            pkts = PostFlightAnalyzer.extract_packets_from_file(filename)
        except Exception as exc:
            QMessageBox.critical(
                self, "Validation Failed",
                f"The selected file is not a supported flight telemetry log:\n{exc}"
            )
            return

        target_session = self.selected_session_dir or self.active_session_dir
        if not target_session or not os.path.exists(target_session):
            target_session = os.path.join(get_default_storage_dir(), "Imported_Flights")
            os.makedirs(target_session, exist_ok=True)

        onboard_dir = os.path.join(target_session, "onboard")
        os.makedirs(onboard_dir, exist_ok=True)
        dest_path = os.path.join(onboard_dir, os.path.basename(filename))

        try:
            shutil.copyfile(filename, dest_path)
        except Exception as e:
            QMessageBox.warning(self, "Archival Notice", f"File analyzed but copy failed: {e}")

        # Show analysis alert
        st = analysis.stats
        QMessageBox.information(
            self, "Onboard SD Log Verified",
            f"Onboard Flight File: {os.path.basename(filename)}\n\n"
            f"Status: VALID & VERIFIED\n"
            f"Valid Packets: {st.total_valid_packets}\n"
            f"Max Altitude: {st.max_altitude if st.max_altitude is not None else 'N/A'} m\n"
            f"Max Velocity: {st.max_velocity if st.max_velocity is not None else 'N/A'} m/s\n"
            f"Max Acceleration: {st.max_acceleration if st.max_acceleration is not None else 'N/A'} m/s²\n"
            f"Observed Rate: {st.observed_telemetry_rate_hz} Hz\n\n"
            f"Archived to: {dest_path}"
        )

        self.refresh_sessions()
        self._load_file_into_table(dest_path, "IMPORTED_ONBOARD")

    # ── Public accessor properties ───────────────────────────────────────
    @property
    def export_btn(self): return self._export_btn
    @property
    def open_folder_btn(self): return self._open_folder_btn
    @property
    def import_sd_btn(self): return self._import_sd_btn
    @property
    def refresh_btn(self): return self._refresh_btn
    @property
    def search_input(self): return self._search_input
    @property
    def source_combo(self): return self._source_filter
    @property
    def session_list(self): return self._session_list
    @property
    def session_tree(self): return self._session_list
    @property
    def file_list(self): return self._file_list
    @property
    def data_table(self): return self._table_view
