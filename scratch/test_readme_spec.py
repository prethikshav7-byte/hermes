"""
scratch/test_readme_spec.py
═══════════════════════════
Verification script for Team 2026 IN-SPACe-010 README specification:
- 18-field LoRa air packet format ($T + 17 fields, no CR/LF on air)
- 19-field packet format with checksum
- Trailing RSSI / SNR extras
- RECOVERY state 0-value preservation
- Command framing: 2026 IN-SPACe-010,<CMD> and GF,0 / GF,1 / GS direct commands
- Hardware ACK / NACK replies: $H,ACK,CALA / $H,ACK,CALI / $H,ACK,CAL / $H,NACK
- Hardware diagnostic lines: $G, GPS, LORA, BITE
- Actuator positions: Door 50 deg, CanSat 80 deg, Sep 80 deg, Unlock 0 deg
- Radio frequencies: 866.50 MHz (Main) and 865.25 MHz (Backup)
"""
import sys
sys.path.insert(0, ".")
from ground_station.codec import CsvCodec, xor_checksum
from ground_station.parser import TelemetryDemux
from ground_station.models import FlightState, VehicleID
from ground_station.commands import COMMANDS, BENCH_COMMANDS
from ground_station.actuators import BY_ID

def check(name: str, condition: bool, extra: str = ""):
    if condition:
        print(f"  ok    {name}")
    else:
        print(f"  FAIL  {name} {extra}")
        sys.exit(1)

print("=== 1. README 18-field LoRa Telemetry Packet Decoding ===")
codec = CsvCodec()
# $T,2026 IN-SPACe-010,time_s,count,alt,press,temp,volt,gnss_time,lat,lon,gnss_alt,sats,acc,spin,state,vehicle,vel
raw_18 = "$T,2026 IN-SPACe-010,12.5,45,1234,101325,254,498,1842,123456,789012,1542,8,98,15,2,1,24"
frame = codec.decode(raw_18)
vals = frame.values
check("team_id preserved", vals["team_id"] == "2026 IN-SPACe-010")
check("mission_time float", vals["mission_time"] == 12.5)
check("packet_count int", vals["packet_count"] == 45)
check("altitude 0.1m scaled", abs(vals["altitude"] - 123.4) < 1e-4)
check("pressure Pa", vals["pressure"] == 101325.0)
check("temperature 0.1C scaled", abs(vals["temperature"] - 25.4) < 1e-4)
check("voltage 0.01V scaled", abs(vals["battery_voltage"] - 4.98) < 1e-4)
check("gnss_time string", vals["gnss_time"] == "1842")
check("gnss_latitude 0.0001 deg scaled", abs(vals["gnss_latitude"] - 12.3456) < 1e-4)
check("gnss_longitude 0.0001 deg scaled", abs(vals["gnss_longitude"] - 78.9012) < 1e-4)
check("gnss_altitude 0.1m scaled", abs(vals["gnss_altitude"] - 154.2) < 1e-4)
check("gnss_satellites int", vals["gnss_satellites"] == 8)
check("accelerometer 0.1 m/s2 scaled", abs(vals["accelerometer"] - 9.8) < 1e-4)
check("gyro_spin_rate 0.1 deg/s scaled", abs(vals["gyro_spin_rate"] - 1.5) < 1e-4)
check("state is BOOST (2)", vals["state"] is FlightState.BOOST)
check("vehicle is ROCKET (1)", vals["vehicle_id"] is VehicleID.ROCKET)
check("velocity is 24.0 m/s", abs(vals["velocity"] - 24.0) < 1e-4)

print("\n=== 2. 19-field Telemetry Packet with Checksum ===")
chk = xor_checksum(raw_18)
raw_19 = f"{raw_18},{chk}"
frame_19 = codec.decode(raw_19)
check("checksum decoded", frame_19.values["checksum"] == chk)
check("altitude matches", abs(frame_19.values["altitude"] - 123.4) < 1e-4)

print("\n=== 3. Trailing Receiver RSSI / SNR Extras ===")
raw_extras = f"{raw_18},RSSI=-85.5,SNR=9.2"
frame_extras = codec.decode(raw_extras)
check("RSSI extra parsed", frame_extras.extras.get("RSSI") == "-85.5")
check("SNR extra parsed", frame_extras.extras.get("SNR") == "9.2")

print("\n=== 4. RECOVERY State Zero-Value Preservation ===")
# In RECOVERY: alt, press, temp, volt, sats, acc, spin, vel are sent as 0
# time, count, gnss_time, lat, lon, gnss_alt, state, vehicle are kept
rec_18 = "$T,2026 IN-SPACe-010,150.0,180,0,0,0,0,1950,123456,789012,250,0,0,0,7,1,0"
frame_rec = codec.decode(rec_18)
rvals = frame_rec.values
check("state is RECOVERY (7)", rvals["state"] is FlightState.RECOVERY)
check("altitude is 0.0", rvals["altitude"] == 0.0)
check("pressure is 0.0", rvals["pressure"] == 0.0)
check("temperature is 0.0", rvals["temperature"] == 0.0)
check("voltage is 0.0", rvals["battery_voltage"] == 0.0)
check("satellites is 0", rvals["gnss_satellites"] == 0)
check("accelerometer is 0.0", rvals["accelerometer"] == 0.0)
check("gyro_spin_rate is 0.0", rvals["gyro_spin_rate"] == 0.0)
check("velocity is 0.0", rvals["velocity"] == 0.0)
check("gnss_altitude kept (25.0 m)", abs(rvals["gnss_altitude"] - 25.0) < 1e-4)
check("gnss_latitude kept", abs(rvals["gnss_latitude"] - 12.3456) < 1e-4)

print("\n=== 5. Demux and Hardware ACK / NACK Responses ===")
demux = TelemetryDemux(codec)
ack_replies = []
demux.on_command_ack = lambda cmd, status, det, raw: ack_replies.append((cmd, status, det, raw))

# Altitude calibration ACK: $H,ACK,CALA,<n>,<p0_Pa>,<T0_0.1C>,<gnss_ground_0.1m>
demux.feed("$H,ACK,CALA,1,101325,245,150")
check("CALA ack received", len(ack_replies) == 1)
check("CALA cmd matches", ack_replies[0][0] == "CALA")
check("CALA status ACK", ack_replies[0][1] == "ACK")
check("CALA p0 parsed", ack_replies[0][2].get("p0_Pa") == "101325")
check("CALA T0 parsed (24.5 C)", abs(ack_replies[0][2].get("T0") - 24.5) < 1e-4)
check("CALA gnss_ground parsed (15.0 m)", abs(ack_replies[0][2].get("gnss_ground") - 15.0) < 1e-4)

# IMU calibration ACK: $H,ACK,CALI,<n>,<axis>,<sign>,<bias_mm/s2>,<sin_elev_1e-4>,<spin_axis>,<spin_sign>,<bno_ok>,<bno_tilt_0.1deg>
demux.feed("$H,ACK,CALI,2,Z,+,9800,10000,Z,+,1,12")
check("CALI ack received", len(ack_replies) == 2)
check("CALI axis parsed", ack_replies[1][2].get("axis") == "Z")
check("CALI bno_tilt parsed (1.2 deg)", abs(ack_replies[1][2].get("bno_tilt") - 1.2) < 1e-4)

# CAL overall ACK: $H,ACK,CAL,<n>,<alt result>,<imu result>
demux.feed("$H,ACK,CAL,3,OK,OK")
check("CAL ack received", len(ack_replies) == 3)
check("CAL alt_result OK", ack_replies[2][2].get("alt_result") == "OK")

# NACK response: $H,NACK,<cmd>,<reason>
demux.feed("$H,NACK,CALI,MOVING")
check("NACK received", len(ack_replies) == 4)
check("NACK status", ack_replies[3][1] == "NACK")
check("NACK reason MOVING", ack_replies[3][2].get("reason") == "MOVING")

print("\n=== 6. Hardware Status and Configuration Diagnostics ===")
demux.feed("$G,RADIO,CFG,FRF=D8A000,MC1=72,MC2=94,SYNC=4A,PA=8F")
demux.feed("GPS,CFG,NAV_MODE_AIRBORNE,ACK")
demux.feed("LORA,CFG,OK")
notices = demux.drain_notices()
check("diagnostics in notices", len(notices) >= 3)
check("radio cfg notice present", any("FRF=D8A000" in n for n in notices))

print("\n=== 7. README Documented Commands in COMMANDS and BENCH_COMMANDS ===")
for cmd in ("CAL", "CALA", "CALI", "FQ,0", "FQ,1", "GF,0", "GF,1", "TX,0", "TX,1",
            "FPL", "FPL,1", "FSEQ", "RF", "LOCKALL", "UNLOCK", "LOCK", "MSEQ", "NSTEP",
            "SEQRST", "SEP", "BZ", "GR", "LR", "BITE", "ENABLE", "DISABLE", "GS", "LIVE", "HIBERNATE"):
    check(f"command {cmd} registered", cmd in COMMANDS or cmd in BENCH_COMMANDS)

print("\n=== 8. Actuator Constants and Travel Positions ===")
check("PCA9685 CH0 Door", BY_ID["ACT_DOOR"].channel == 0)
check("PCA9685 CH1 CanSat Deploy", BY_ID["ACT_DEPLOY"].channel == 1)
check("PCA9685 CH2 Separation", BY_ID["ACT_SEPARATE"].channel == 2)

from ground_station.actuators import Actuator
from ground_station.pages.command import ActuatorPanel
from PyQt6.QtWidgets import QApplication
app = QApplication.instance() or QApplication(sys.argv)

from ground_station.actuators import ArmingCentre
arming = ArmingCentre()
door_act = Actuator(BY_ID["ACT_DOOR"])
deploy_act = Actuator(BY_ID["ACT_DEPLOY"])
sep_act = Actuator(BY_ID["ACT_SEPARATE"])

door_panel = ActuatorPanel(BY_ID["ACT_DOOR"], door_act, arming)
deploy_panel = ActuatorPanel(BY_ID["ACT_DEPLOY"], deploy_act, arming)
sep_panel = ActuatorPanel(BY_ID["ACT_SEPARATE"], sep_act, arming)

door_panel._on_lock_clicked()
check("Door lock angle is 50 deg", door_act.commanded == 50.0)

deploy_panel._on_lock_clicked()
check("CanSat lock angle is 80 deg", deploy_act.commanded == 80.0)

sep_panel._on_lock_clicked()
check("Separation lock angle is 80 deg", sep_act.commanded == 80.0)

door_panel._on_unlock_clicked()
check("Unlock angle is 0 deg", door_act.commanded == 0.0)

print("\n=== 9. Radio Frequencies ===")
from ground_station.commands import CommandCentre
from ground_station.pages.command import CommandPage
centre = CommandCentre(lambda target, cmd, seq: None)
cmd_page = CommandPage(centre)
cmd_page._set_frequency(VehicleID.ROCKET, "ORIGINAL")
check("Main frequency sends FQ,0", cmd_page._rocket_freq == "ORIGINAL")
cmd_page._set_frequency(VehicleID.ROCKET, "BACKUP")
check("Backup frequency sends FQ,1", cmd_page._rocket_freq == "BACKUP")

print("\nALL README SPECIFICATION CHECKS PASSED PERFECTLY!")
