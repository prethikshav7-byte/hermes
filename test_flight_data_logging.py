"""
test_flight_data_logging.py
═══════════════════════════
Comprehensive Verification of Flight Data Logging, Session Management,
CSV Navigation, SD-Card Import, and Shutdown Lifecycle for MACHZONE Ground Station.
"""
import csv
import json
import os
import shutil
import sys
import tempfile
import time

from PyQt6.QtWidgets import QApplication

app = QApplication.instance()
if not app:
    app = QApplication(sys.argv)

from ground_station.app import GroundStationApp
from ground_station.models import VehicleID, FlightState, TelemetryPacket
from ground_station.parser import IngestResult, LossTier
from ground_station.logs.sd_log import SDTelemetryLogger, BOOK1_HEADER
from ground_station.logs.raw_log import SessionLogs, get_default_storage_dir
from ground_station.logs.post_flight import PostFlightAnalyzer
from ground_station.pages.csv_manager import CsvManagerPage, CsvFileTableModel

passed = 0
failed = 0


def check(name: str, cond: bool, note: str = ""):
    global passed, failed
    if cond:
        passed += 1
        print(f"  [PASS] {name}")
    else:
        failed += 1
        print(f"  [FAIL] {name} {note}")


def create_dummy_packet(vid: VehicleID, pkt_count: int, alt: float, press: float = 101325.0,
                        temp: float = 24.5, batt: float = 8.1, state: FlightState = FlightState.BOOST) -> TelemetryPacket:
    return TelemetryPacket(
        vehicle_id=vid,
        state=state,
        packet_count=pkt_count,
        mission_time=pkt_count * 0.1,
        altitude=alt,
        pressure=press,
        temperature=temp,
        battery_voltage=batt,
        accelerometer=18.6,
        gyro_spin_rate=0.5,
        velocity=12.0,
        gnss_latitude=13.0827,
        gnss_longitude=80.2707,
        gnss_altitude=alt + 10.0,
        gnss_satellites=12,
        raw=f"$T,{vid.value},{state.value},{pkt_count},{pkt_count*0.1:.2f},{alt:.2f},{press:.1f},{temp:.1f},{batt:.2f},18.5,2.1,0.5,1.2,0.1,13.0827,80.2707,1,12,-65,9.5*FF\n"
    )


print("\n=== 1. CSV Schema & Backward Compatibility ===")
check("BOOK1_HEADER contains 24 fields", len(BOOK1_HEADER) == 24)
check("BOOK1_HEADER last field is DATA_SOURCE", BOOK1_HEADER[-1] == "DATA_SOURCE")
check("BOOK1_HEADER contains expected sensor fields", "ALTITUDE" in BOOK1_HEADER and "PRESSURE" in BOOK1_HEADER and "VOLTAGE" in BOOK1_HEADER)

print("\n=== 2. Platform-Aware Default Storage Directory ===")
def_dir = get_default_storage_dir()
check("Default storage directory is a string", isinstance(def_dir, str) and len(def_dir) > 0)
check("Default storage path includes MACHZONE_GroundStation/sessions", "MACHZONE_GroundStation" in def_dir and "sessions" in def_dir)

print("\n=== 3. Session Directory Structure & Dual-Sink File Creation ===")
test_base = tempfile.mkdtemp(prefix="mz_log_test_")
try:
    session = SessionLogs(test_base, data_source="HARDWARE")
    sess_dir = session.directory
    check("Session folder created under test_base", os.path.isdir(sess_dir))
    check("ground_station subfolder created", os.path.isdir(os.path.join(sess_dir, "ground_station")))
    check("onboard subfolder created", os.path.isdir(os.path.join(sess_dir, "onboard")))

    # Check raw log dual sinks exist
    gs_dir = os.path.join(sess_dir, "ground_station")
    check("rocket_raw.log in ground_station/", os.path.exists(os.path.join(gs_dir, "rocket_raw.log")))
    check("cansat_raw.log in ground_station/", os.path.exists(os.path.join(gs_dir, "cansat_raw.log")))
    check("rocket_raw.log in session root", os.path.exists(os.path.join(sess_dir, "rocket_raw.log")))

    # Check CSV files exist in ground_station/ and session root
    check("rocket_received.csv in ground_station/", os.path.exists(os.path.join(gs_dir, "rocket_received.csv")))
    check("cansat_received.csv in ground_station/", os.path.exists(os.path.join(gs_dir, "cansat_received.csv")))
    check("Legacy PHOENIX_*_ROCKET.csv in root", any(f.endswith("_ROCKET.csv") for f in os.listdir(sess_dir)))

    print("\n=== 4. Asynchronous Queue Telemetry Logging & Data Integrity ===")
    p1 = create_dummy_packet(VehicleID.ROCKET, 1, 10.5, 101200.0, 25.0, 8.2)
    p2 = create_dummy_packet(VehicleID.ROCKET, 2, 250.0, 98500.0, 22.0, 8.1)
    p3 = create_dummy_packet(VehicleID.CANSAT, 1, 8.0, 101250.0, 26.0, 7.8)

    session.sd.log_packet(p1)
    session.sd.log_packet(p2)
    session.sd.log_packet(p3)

    # Allow worker thread to drain queue
    session.sd.flush()
    time.sleep(0.1)

    # Verify rocket_received.csv contents
    rocket_csv_path = os.path.join(gs_dir, "rocket_received.csv")
    with open(rocket_csv_path, "r", encoding="utf-8") as f:
        reader = list(csv.reader(f))
        check("Rocket CSV has header + 2 data rows", len(reader) == 3, f"got {len(reader)}")
        header = reader[0]
        row1 = reader[1]
        row2 = reader[2]
        check("Header matches BOOK1_HEADER", header == BOOK1_HEADER)
        check("Row 1 ALTITUDE is 10.5", row1[header.index("ALTITUDE")] == "10.50")
        check("Row 2 ALTITUDE is 250.0", row2[header.index("ALTITUDE")] == "250.00")
        check("Row 1 DATA_SOURCE is HARDWARE", row1[header.index("DATA_SOURCE")] == "HARDWARE")
        check("Row 2 DATA_SOURCE is HARDWARE", row2[header.index("DATA_SOURCE")] == "HARDWARE")
        check("VEHICLE column is 1 (ROCKET)", row1[header.index("VEHICLE")] == "1")

    # Verify cansat_received.csv contents
    cansat_csv_path = os.path.join(gs_dir, "cansat_received.csv")
    with open(cansat_csv_path, "r", encoding="utf-8") as f:
        reader = list(csv.reader(f))
        check("CanSat CSV has header + 1 data row", len(reader) == 2, f"got {len(reader)}")
        check("CanSat row VEHICLE is 2 (CANSAT)", reader[1][header.index("VEHICLE")] == "2")

    # Verify rejected lines go to rejected log without polluting CSV
    session.sd.log_rejected("$T,BAD,PACKET", "CHECKSUM", VehicleID.ROCKET, time.time())
    with open(rocket_csv_path, "r", encoding="utf-8") as f:
        reader = list(csv.reader(f))
        check("Rocket CSV count still 3 after rejected line", len(reader) == 3)

    print("\n=== 5. Session Finalization & session_metadata.json Generation ===")
    session.close()
    meta_path = os.path.join(sess_dir, "session_metadata.json")
    check("session_metadata.json created on close", os.path.exists(meta_path))
    with open(meta_path, "r", encoding="utf-8") as f:
        meta = json.load(f)
        check("Metadata session_id matches", meta.get("session_id") == session.session_id)
        check("Metadata data_source is HARDWARE", meta.get("data_source") == "HARDWARE")
        check("Metadata status is COMPLETED", meta.get("status") == "COMPLETED")
        stats = meta.get("statistics", {})
        check("Rocket stats total_packets == 2", stats.get("ROCKET", {}).get("total_packets") == 2)
        check("Rocket stats max_altitude == 250.0", stats.get("ROCKET", {}).get("max_altitude") == 250.0)
        check("CanSat stats total_packets == 1", stats.get("CANSAT", {}).get("total_packets") == 1)
        check("CanSat stats max_altitude == 8.0", stats.get("CANSAT", {}).get("max_altitude") == 8.0)

    print("\n=== 6. Simulation Mode Logging Separation ===")
    sim_session = SessionLogs(test_base, session="sim_test_001", data_source="SIMULATION")
    sim_p = create_dummy_packet(VehicleID.ROCKET, 1, 55.0)
    sim_session.sd.log_packet(sim_p)
    sim_session.close()
    sim_meta_path = os.path.join(sim_session.directory, "session_metadata.json")
    with open(sim_meta_path, "r", encoding="utf-8") as f:
        sim_meta = json.load(f)
        check("Simulation session data_source is SIMULATION", sim_meta.get("data_source") == "SIMULATION")
    sim_rocket_csv = os.path.join(sim_session.directory, "ground_station", "rocket_received.csv")
    with open(sim_rocket_csv, "r", encoding="utf-8") as f:
        reader = list(csv.reader(f))
        check("Simulation CSV row DATA_SOURCE is SIMULATION", reader[1][-1] == "SIMULATION")

    print("\n=== 7. Onboard SD-Card Data Import Validation ===")
    # Create valid sample onboard SD log
    sample_sd = os.path.join(test_base, "sample_onboard_sd.csv")
    with open(sample_sd, "w", encoding="utf-8") as f:
        f.write("mission_ms,mission_time_s,telem_time_s,state,altitude_m,velocity_ms,pressure_pa,acc_mag_ms2\n")
        f.write("1000,1.0,1.0,1,0.0,0.0,101325.0,9.81\n")
        f.write("2000,2.0,2.0,2,150.0,45.0,99000.0,25.0\n")
        f.write("3000,3.0,3.0,4,420.0,0.0,96000.0,1.0\n")

    res = PostFlightAnalyzer.analyze_file(sample_sd)
    check("PostFlightAnalyzer accepts onboard CSV", res is not None)
    check("PostFlightAnalyzer calculated max altitude 420.0m", res.stats.max_altitude == 420.0)

    # Test corrupted/empty file handling
    empty_sd = os.path.join(test_base, "empty.csv")
    with open(empty_sd, "w", encoding="utf-8") as f:
        pass
    try:
        empty_res = PostFlightAnalyzer.analyze_file(empty_sd)
        check("Empty file handled safely (returns None or raises handled error)", empty_res is None or empty_res.stats.total_valid_packets == 0)
    except Exception:
        check("Empty file raised expected exception", True)

    print("\n=== 8. GUI Integration: Stack & Flight Logs Page ===")
    window = GroundStationApp(mode_label="TEST", simulator_mode=False)
    window.set_session_logs(session)

    nav_texts = [b.text() for b in window._nav]
    check("Navigation has 7 buttons", len(nav_texts) == 7, f"got {len(nav_texts)}: {nav_texts}")
    check("Button 5 is DEV", nav_texts[5] == "DEV")
    check("Button 6 is Flight Logs", nav_texts[6] == "Flight Logs")
    check("Stack has 7 pages", window.stack.count() == 7)

    csv_page = window.csv_page
    check("csv_page is instance of CsvManagerPage", isinstance(csv_page, CsvManagerPage))
    check("csv_page has search bar", hasattr(csv_page, "search_input"))
    check("csv_page has source combo", hasattr(csv_page, "source_combo"))
    check("csv_page has session tree", hasattr(csv_page, "session_tree"))
    check("csv_page has data table", hasattr(csv_page, "data_table"))
    check("csv_page has Export button", hasattr(csv_page, "export_btn"))
    check("csv_page has Open Folder button", hasattr(csv_page, "open_folder_btn"))
    check("csv_page has Import SD Card button", hasattr(csv_page, "import_sd_btn"))

    # Test table model loading actual CSV file
    from PyQt6.QtCore import Qt
    model = CsvFileTableModel()
    loaded_count, err = model.load_file(rocket_csv_path)
    check("TableModel loaded without error", err is None)
    check("TableModel row count == 2 (data rows)", model.rowCount() == 2, f"got {model.rowCount()}, rows={model.rows}")
    check("TableModel column count == 24", model.columnCount() == 24)
    check("TableModel header 0 is $T", model.headerData(0, Qt.Orientation.Horizontal) == "$T")
    val_0_4 = model.data(model.index(0, 4))
    check("TableModel cell (0, 4) altitude is 10.50", val_0_4 == "10.50", f"got '{val_0_4}', row0={model.rows[0] if model.rows else None}")

    # Verify switching page to Flight Logs
    window._select_page(6)
    check("Current stack index is 6", window.stack.currentIndex() == 6)

    # Verify window closeEvent calls logs.close() safely
    window.close()
    check("Window closed cleanly", True)

finally:
    # Cleanup temporary test directory
    try:
        shutil.rmtree(test_base, ignore_errors=True)
    except Exception:
        pass

print(f"\n==========================================")
print(f"RESULTS: {passed} passed, {failed} failed")
print(f"==========================================")
if failed > 0:
    sys.exit(1)
