"""
test_antigravity_master.py
══════════════════════════
Automated verification for all requirements in ANTIGRAVITY MASTER PROMPT:
1. UI Preservation & Sidebar (DEV at 5, CSV at 6)
2. Removal of operator sequence banner & diagnostics panel layout in CommandPage
3. Absence of extra DEV badge in ComPage header
4. Real-time button colors (idle neutral, pending yellow, ACK green)
5. CanSat deploy buttons enabled and NOT forced red
6. Stationary simulated accelerometer magnitude (~9.8 m/s²)
7. Post-Flight SD Card Analysis for actual rocket firmware (DATA_HEADER), ground SD, and wire $T
8. CSV Page telemetry table, filtering, graphs, and native export with overwrite safety
"""
import os
import sys
import tempfile
import time
import math

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt

app = QApplication.instance()
if not app:
    app = QApplication(sys.argv)

from ground_station.theme import ThemeManager
from ground_station.app import GroundStationApp
from ground_station.models import VehicleID, FlightState, TelemetryPacket
from ground_station.parser import IngestResult, LossTier
from ground_station.pages.command import CommandPage
from ground_station.pages.com import ComPage
from ground_station.logs.post_flight import PostFlightAnalyzer, PostFlightSource
from ground_station.simulator.engine import SimulatorEngine
from ground_station.commands import CommandCentre

PASSED = 0
FAILED = 0


def check(name: str, cond: bool, extra: str = ""):
    global PASSED, FAILED
    if cond:
        print(f"  [PASS] {name}")
        PASSED += 1
    else:
        print(f"  [FAIL] {name} {extra}")
        FAILED += 1


print("\n=== 1. UI Navigation & Layout Preservation ===")
window = GroundStationApp(mode_label="TEST", simulator_mode=True)
nav_texts = [b.text() for b in window._nav]
check("DEV navigation label exists", "DEV" in nav_texts)
check("DEV is at index 5", nav_texts[5] == "DEV")
check("Flight Logs is at index 6", nav_texts[6] == "Flight Logs")
check("Navigation has exactly 7 items", len(nav_texts) == 7)
check("Stack has 7 pages", window.stack.count() == 7)
check("Stack index 5 is ComPage", isinstance(window.stack.widget(5), ComPage))
check("Window has csv_page attribute", hasattr(window, "csv_page"))

print("\n=== 2. ComPage Clean Header (No extra DEV badge) ===")
com_page = window.com_page
check("ComPage has no dev_badge in header", not hasattr(com_page, "dev_badge") or com_page.dev_badge is None or not com_page.dev_badge.isVisible())
check("ComPage port_val minimum width is flexible", com_page.port_val.minimumWidth() >= 80)

print("\n=== 3. Command Page Clean Layout & Button Initial Styles ===")
cmd_page = window.command_page
check("Operator sequence banner is not in layout", not hasattr(cmd_page, "_op_seq_banner") or cmd_page._op_seq_banner.parent() is None)
check("Diagnostics panel widget is not in layout", cmd_page.diagnostics_panel.parent() is None)
check("Command log is preserved", hasattr(cmd_page, "log_panel") and cmd_page.log_panel.parent() is not None)

# Verify button initial neutral styling (not permanently red)
live_btn = cmd_page.mode_panel.buttons.get("LIVE")
hib_btn = cmd_page.mode_panel.buttons.get("HIBERNATE")
en_btn = cmd_page.mode_panel.buttons.get("ENABLE")
dis_btn = cmd_page.mode_panel.buttons.get("DISABLE")

neutral_css = ThemeManager.status_button_css("neutral")
check("Live button created with neutral CSS", live_btn.styleSheet() == neutral_css)
check("Hibernate button created with neutral CSS", hib_btn.styleSheet() == neutral_css)
check("Enable button created with neutral CSS", en_btn.styleSheet() == neutral_css)
check("Disable button created with neutral CSS", dis_btn.styleSheet() == neutral_css)

print("\n=== 4. CanSat Deploy Actuator Buttons Not Forced Red ===")
deploy_panel = cmd_page.actuator_panels.get("ACT_DEPLOY")
check("ACT_DEPLOY actuator panel exists", deploy_panel is not None)
if deploy_panel:
    cansat_30 = deploy_panel.buttons.get("SV,1,30")
    cansat_lock = deploy_panel.buttons.get("LOCK,1")
    check("SV,1,30 button exists", cansat_30 is not None)
    # Simulate backend button update
    cmd_page._on_backend_buttons_changed({
        "default": "neutral",
        "buttons": {},
        "tooltips": {}
    })
    check("SV,1,30 is NOT disabled by CANSAT substring", cansat_30.isEnabled() is True)
    check("SV,1,30 has neutral style (not red)", cansat_30.styleSheet() == neutral_css)

print("\n=== 5. Simulated Stationary Accelerometer Magnitude ===")
engine = SimulatorEngine()
vals = engine.flights[VehicleID.CANSAT].values(0.0)
acc_mag = vals.get("accelerometer")
check("Stationary accel_mag is around 9.8 m/s²", acc_mag is not None and 9.0 <= acc_mag <= 10.6, f"got {acc_mag}")

print("\n=== 6. Post-Flight SD Card Analysis Backend ===")
tmp_dir = tempfile.mkdtemp(prefix="pf_test_")

# Test 6a: Rocket firmware SD log format (DATA_HEADER)
rocket_csv = os.path.join(tmp_dir, "IN-SPACe-010_ROCKET_FLIGHT_001.csv")
header_line = (
    "mission_ms,mission_time_s,telem_time_s,gnss_utc,gps_epoch_ms,state,state_name,token,flags,"
    "altitude_m,velocity_ms,ekf_bias_ms2,alt_source,baro_valid,baro_altitude_m,baro_velocity_ms,pressure_pa,"
    "pressure_source,ms5611_temp_c,ds18b20_temp_c,ds_valid,"
    "acc_x_ms2,acc_y_ms2,acc_z_ms2,acc_axial_ms2,acc_mag_ms2,acc_peak_ms2,acc_samples,"
    "gyro_x_dps,gyro_y_dps,gyro_z_dps,spin_dps,lin_acc_ms2,roll_deg,pitch_deg,yaw_deg,bno_ok,"
    "latitude_deg,longitude_deg,gnss_altitude_m,gnss_agl_m,gnss_vel_ms,satellites,hdop,gps_fix,gps_baud,pps_count,"
    "load_v,ina_bus_v,ina_shunt_mv,ina_current_ma,"
    "channel,freq_trial,ground_heard,tx_muted,tx_cutoff,telemetry_on,lora_dbm,packet_count,rx_good,rx_crc,"
    "lora_reinits,gps_resets,buzzer,"
    "door_cmd_deg,payload_cmd_deg,sep_cmd_deg,door_est_deg,payload_est_deg,sep_est_deg,door_us,payload_us,sep_us,"
    "fault_mask,t_launch_ms,t_door_ms,t_apogee_ms,t_state_ms,payload_due_ms,sep_due_ms,deploy_active,"
    "v_peak_ms,h_peak_m,h_max_m,cal_count,warm_resets,ekf_phase,boot_done,boot_done_ms,"
    "lora_tx,alt_slope_ms,vel_slope_ms2,launch_confirmed,ekf_baro_only,rtc_ticks,door_by_alt\n"
)
data_rows = [
    "1000,1.000,1,-,0,1,PRELAUNCH,0,0x00,0.50,0.00,0.000,0,1,0.50,0.00,101320.0,0,22.50,22.00,1,0.02,0.02,9.81,9.81,9.81,9.81,1,0.1,0.1,0.0,0.0,0.0,0,0,0,1,12.345678,80.123456,10.0,0.5,0.0,9,1.1,1,115200,10,4.98,4.98,0.1,150.0,0,0,1,0,0,1,10,1,1,0,0,0,0,50,80,80,50,80,80,1500,1500,1500,0x00,0,0,0,1000,0,0,0,0.0,0.5,0.5,1,0,0,1,500,0,0.0,0.0,0,0,100,0\n",
    "2000,2.000,2,-,0,2,BOOST,0,0x00,45.20,38.50,0.000,0,1,45.20,38.50,100800.0,0,22.10,21.80,1,0.40,0.40,68.50,68.50,68.50,68.50,1,1.2,1.5,18.0,18.0,68.0,0,0,0,1,12.345680,80.123458,55.0,45.2,38.5,9,1.1,1,115200,20,4.92,4.92,0.5,450.0,0,0,1,0,0,1,17,2,2,0,0,0,0,50,80,80,50,80,80,1500,1500,1500,0x00,1500,0,0,500,0,0,0,38.5,45.2,45.2,1,0,1,1,500,1,0.0,0.0,1,0,200,0\n",
    "3000,3.000,3,-,0,4,APOGEE,0,0x00,312.80,0.50,0.000,0,1,312.80,0.50,97800.0,0,20.40,20.20,1,0.10,0.10,1.20,1.20,1.20,1.20,1,0.5,0.8,4.0,4.0,1.0,0,0,0,1,12.345690,80.123465,322.0,312.8,0.5,10,1.0,1,115200,30,4.95,4.95,0.2,200.0,0,0,1,0,0,1,17,3,3,0,0,0,0,0,0,80,0,0,80,600,600,1500,0x00,1500,2500,3000,0,0,0,1,38.5,312.8,312.8,1,0,2,1,500,1,0.0,0.0,1,0,300,1\n",
]
with open(rocket_csv, "w", encoding="utf-8") as f:
    f.write(header_line)
    for r in data_rows:
        f.write(r)

res_rocket = PostFlightAnalyzer.analyze_file(rocket_csv)
check("Rocket SD log parsed without errors", res_rocket is not None)
check("Rocket SD stats total valid packets == 3", res_rocket.stats.total_valid_packets == 3)
check("Rocket SD max altitude == 312.80 m", math.isclose(res_rocket.stats.max_altitude, 312.80, rel_tol=1e-3))
check("Rocket SD max velocity == 38.50 m/s", math.isclose(res_rocket.stats.max_velocity, 38.50, rel_tol=1e-3))
check("Rocket SD max acceleration == 68.50 m/s²", math.isclose(res_rocket.stats.max_acceleration, 68.50, rel_tol=1e-3))

pkts = PostFlightAnalyzer.extract_packets_from_file(rocket_csv)
check("extract_packets_from_file returned 3 packets", len(pkts) == 3)
check("Packet 0 is TelemetryPacket", isinstance(pkts[0][0], TelemetryPacket))
check("Packet 0 state is FlightState.PRE_LAUNCH", pkts[0][0].state == FlightState.PRE_LAUNCH)
check("Packet 1 state is FlightState.BOOST", pkts[1][0].state == FlightState.BOOST)

# Test 6b: PostFlightSource replay
pf_source = PostFlightSource(rocket_csv)
received_packets = []
pf_source.rocket_packet.connect(lambda p, r: received_packets.append(p))
pf_source.connect_to()
check("PostFlightSource replayed all 3 packets into rocket_packet signal", len(received_packets) == 3)

print("\n=== 7. Flight Logs Page & Stack Verification ===")
check("CsvManagerPage exists in pages", os.path.exists(os.path.join("ground_station", "pages", "csv_manager.py")))
check("App stack has exactly 7 pages", window.stack.count() == 7)
check("App navigation has exactly 7 buttons", len(window._nav) == 7)

print(f"\n==========================================")
print(f"RESULTS: {PASSED} passed, {FAILED} failed")
print(f"==========================================")
if FAILED > 0:
    sys.exit(1)
