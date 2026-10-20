"""
scratch/test_teensy_integration.py
Comprehensive test suite verifying backend command integration with Teensy 4.1 hardware.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ground_station.commands import (
    CommandCentre, CommandStatus, to_firmware_command, FIRMWARE_COMMAND_MAP
)
from ground_station.models import VehicleID, FlightState, VehicleLinkState
from ground_station.parser.parser import TelemetryDemux
from ground_station.link import LinkSupervisor, SerialSource
from ground_station.actuators import ACTUATORS, CalibrationOutcome
from ground_station.pages.command import CommandPage


def check(name: str, cond: bool) -> None:
    assert cond, f"FAILED: {name}"
    print(f"  ok    {name}")


def test_command_mapping():
    print("\n=== 1. UI Commands to Teensy 4.1 Firmware Protocol ===")
    check("LIVE mapped to LIVE", to_firmware_command("LIVE") == "LIVE")
    check("SET_MODE_LIVE mapped to LIVE", to_firmware_command("SET_MODE_LIVE") == "LIVE")
    check("SET_MODE_HIBERNATE mapped to HIBERNATE", to_firmware_command("SET_MODE_HIBERNATE") == "HIBERNATE")
    check("ENABLE_TELEMETRY mapped to ENABLE", to_firmware_command("ENABLE_TELEMETRY") == "ENABLE")
    check("DISABLE_TELEMETRY mapped to TX,0", to_firmware_command("DISABLE_TELEMETRY") == "TX,0")

    # Actuators
    check("DOOR:30 mapped to SV,0,30", to_firmware_command("DOOR:30") == "SV,0,30")
    check("DOOR:60 mapped to SV,0,60", to_firmware_command("DOOR:60") == "SV,0,60")
    check("DOOR:90 mapped to SV,0,90", to_firmware_command("DOOR:90") == "SV,0,90")
    check("LOCK:DOOR mapped to LOCK,0", to_firmware_command("LOCK:DOOR") == "LOCK,0")
    check("UNLOCK:DOOR mapped to UNLOCK,0", to_firmware_command("UNLOCK:DOOR") == "UNLOCK,0")

    check("CANSAT:30 mapped to SV,1,30", to_firmware_command("CANSAT:30") == "SV,1,30")
    check("LOCK:CANSAT mapped to LOCK,1", to_firmware_command("LOCK:CANSAT") == "LOCK,1")
    check("UNLOCK:CANSAT mapped to UNLOCK,1", to_firmware_command("UNLOCK:CANSAT") == "UNLOCK,1")

    check("SEPARATION:90 mapped to SV,2,90", to_firmware_command("SEPARATION:90") == "SV,2,90")
    check("LOCK:SEPARATION mapped to LOCK,2", to_firmware_command("LOCK:SEPARATION") == "LOCK,2")
    check("UNLOCK:SEPARATION mapped to UNLOCK,2", to_firmware_command("UNLOCK:SEPARATION") == "UNLOCK,2")

    # Sequences
    check("LOCKALL mapped to LOCKALL", to_firmware_command("LOCKALL") == "LOCKALL")
    check("LOCK:ALL mapped to LOCKALL", to_firmware_command("LOCK:ALL") == "LOCKALL")
    check("RESET mapped to SEQRST", to_firmware_command("RESET") == "SEQRST")
    check("SEQRST mapped to SEQRST", to_firmware_command("SEQRST") == "SEQRST")
    check("TEST5 mapped to MSEQ", to_firmware_command("TEST5") == "MSEQ")
    check("MSEQ mapped to MSEQ", to_firmware_command("MSEQ") == "MSEQ")
    check("NEXT mapped to NSTEP", to_firmware_command("NEXT") == "NSTEP")
    check("NSTEP mapped to NSTEP", to_firmware_command("NSTEP") == "NSTEP")
    check("TEST6 mapped to FSEQ", to_firmware_command("TEST6") == "FSEQ")
    check("FSEQ mapped to FSEQ", to_firmware_command("FSEQ") == "FSEQ")

    # Calibrations
    check("CALIBRATE_ALTITUDE mapped to CALA", to_firmware_command("CALIBRATE_ALTITUDE") == "CALA")
    check("CALIBRATE_IMU mapped to CALI", to_firmware_command("CALIBRATE_IMU") == "CALI")
    check("CALIBRATE_ALL mapped to CAL", to_firmware_command("CALIBRATE_ALL") == "CAL")
    check("REFRESH_PROCESSOR mapped to RF", to_firmware_command("REFRESH_PROCESSOR") == "RF")


def test_ground_serial_framing():
    print("\n=== 2. Ground Teensy USB Serial Framing ===")
    sent_wires = []

    src = SerialSource("COM_TEST")
    supervisor = LinkSupervisor()
    supervisor.source = src

    supervisor.send_command(VehicleID.ROCKET, "SET_MODE_LIVE", 101)
    check("Frame sent format <id>,<CMD>\\n", src._tx[0] == "101,LIVE\n")

    supervisor.send_command(VehicleID.ROCKET, "DOOR:30", 102)
    check("Actuator angle sent as 102,SV,0,30\\n", src._tx[1] == "102,SV,0,30\n")

    supervisor.send_command(VehicleID.ROCKET, "LOCK:SEPARATION", 103)
    check("Separation lock sent as 103,LOCK,2\\n", src._tx[2] == "103,LOCK,2\n")

    supervisor.send_command(VehicleID.ROCKET, "FSEQ", 104)
    check("Full sequence sent as 104,FSEQ\\n", src._tx[3] == "104,FSEQ\n")


def test_ground_response_parsing():
    print("\n=== 3. Ground Teensy Status & Command Responses ===")
    demux = TelemetryDemux()

    ground_cmds = []
    servo_reports = []
    acks = []

    demux.on_ground_cmd = lambda seq, st, inf, raw: ground_cmds.append((seq, st, inf, raw))
    demux.on_servo_report = lambda ch, deg, est, rem, mode, st, raw: servo_reports.append((ch, deg, est, rem, mode, st, raw))
    demux.on_command_ack = lambda cmd, st, det, raw: acks.append((cmd, st, det, raw))

    # $G,CMD,<id>,QUEUED
    demux.feed("$G,CMD,101,QUEUED\n")
    check("Parsed $G,CMD,QUEUED", len(ground_cmds) == 1 and ground_cmds[0][:2] == (101, "QUEUED"))

    # $G,CMD,<id>,SENT,1
    demux.feed("$G,CMD,101,SENT,1\n")
    check("Parsed $G,CMD,SENT", len(ground_cmds) == 2 and ground_cmds[1][:3] == (101, "SENT", "1"))

    # $G,CMD,<id>,OK
    demux.feed("$G,CMD,101,OK\n")
    check("Parsed $G,CMD,OK", len(ground_cmds) == 3 and ground_cmds[2][:2] == (101, "OK"))

    # $G,CMD,<id>,REJECTED,lora_disabled
    demux.feed("$G,CMD,102,REJECTED,lora_disabled\n")
    check("Parsed $G,CMD,REJECTED", len(ground_cmds) == 4 and ground_cmds[3][:3] == (102, "REJECTED", "lora_disabled"))

    # $G,CMD,<id>,FAILED,no_reply
    demux.feed("$G,CMD,103,FAILED,no_reply\n")
    check("Parsed $G,CMD,FAILED", len(ground_cmds) == 5 and ground_cmds[4][:3] == (103, "FAILED", "no_reply"))

    # $H,SV servo report
    demux.feed("$H,SV,0,50,45,5,1,2\n")
    check("Parsed $H,SV servo report", len(servo_reports) == 1 and servo_reports[0][:6] == (0, 50, 45, 5, 1, 2))

    # $H,ACK,CALA
    demux.feed("$H,ACK,CALA,1,101325,245,150\n")
    check("Parsed $H,ACK,CALA", len(acks) == 1 and acks[0][0] == "CALA" and acks[0][1] == "ACK")


def test_command_lifecycle_in_command_centre():
    print("\n=== 4. CommandCentre Lifecycle & Diagnostics ===")
    centre = CommandCentre(lambda vid, cmd, seq: None, ack_timeout=18.0)

    # 1. Send command
    rec = centre.send(VehicleID.ROCKET, "LOCK:DOOR")
    seq = rec.sequence
    check("Record created with status SENT", rec.status == CommandStatus.SENT)
    check("Unacknowledged initially True", rec.unacknowledged is True)

    # 2. Ground Teensy reports SENT
    centre.handle_ground_cmd(seq, "SENT", "1")
    check("Status still SENT after SENT,1", rec.status == CommandStatus.SENT)

    # 3. Ground Teensy reports OK
    centre.handle_ground_cmd(seq, "OK")
    check("Status ACKNOWLEDGED after Ground Teensy OK", rec.status == CommandStatus.ACKNOWLEDGED)
    check("Unacknowledged is False", rec.unacknowledged is False)

    centre.mark_executed(seq)
    check("Status EXECUTED after execution confirmed", rec.status == CommandStatus.EXECUTED)

    # 4. Rejected command
    rec2 = centre.send(VehicleID.ROCKET, "DOOR:30")
    centre.handle_ground_cmd(rec2.sequence, "REJECTED", "lora_disabled")
    check("Status REJECTED recorded", rec2.status == CommandStatus.REJECTED)

    # 5. Failed command (no_reply)
    rec3 = centre.send(VehicleID.ROCKET, "SEPARATION:90")
    centre.handle_ground_cmd(rec3.sequence, "FAILED", "no_reply")
    check("Status TIMED_OUT recorded on failed", rec3.status == CommandStatus.TIMED_OUT)


def test_command_page_widgets():
    print("\n=== 5. CommandPage Widgets & Live Firmware Reports ===")
    from PyQt6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication(sys.argv)

    sent_commands = []
    centre = CommandCentre(lambda vid, cmd, seq: sent_commands.append((vid, cmd, seq)), ack_timeout=18.0)
    page = CommandPage(centre)

    # 1. Test Actuator Lock button
    door_panel = page.actuator_panels["ACT_DOOR"]
    door_panel._on_lock_clicked()
    check("Door lock clicked transmits LOCK:DOOR", sent_commands[-1][1] == "LOCK:DOOR")
    check("Wire mapping for LOCK:DOOR is LOCK,0", to_firmware_command(sent_commands[-1][1]) == "LOCK,0")

    # 2. Test Actuator Unlock button
    door_panel._on_unlock_clicked()
    check("Door unlock clicked transmits UNLOCK:DOOR", sent_commands[-1][1] == "UNLOCK:DOOR")
    check("Wire mapping for UNLOCK:DOOR is UNLOCK,0", to_firmware_command(sent_commands[-1][1]) == "UNLOCK,0")

    # 3. Test Actuator Angle button
    cansat_panel = page.actuator_panels["ACT_DEPLOY"]
    cansat_panel._on_angle_clicked(60)
    check("CanSat 60° clicked transmits CANSAT:60", sent_commands[-1][1] == "CANSAT:60")
    check("Wire mapping for CANSAT:60 is SV,1,60", to_firmware_command(sent_commands[-1][1]) == "SV,1,60")

    # 4. Test Sequence Controls
    page.sequence_panel._on_lock_all_clicked()
    check("LOCK ALL clicked transmits LOCKALL", sent_commands[-1][1] == "LOCKALL")

    page.sequence_panel._on_full_seq_clicked()
    check("FULL SEQUENCE clicked transmits FSEQ", sent_commands[-1][1] == "FSEQ")

    page.sequence_panel._on_reset_clicked()
    check("RESET clicked transmits SEQRST", sent_commands[-1][1] == "SEQRST")

    # 5. Live Servo Report from Rocket Teensy: $H,SV,0,50,48,2,2,3
    # ch=0, deg=50, est=48, rem=2, mode=2 (FULL SEQUENCE), step=3
    page.handle_servo_report(0, 50, 48, 2, 2, 3, "$H,SV,0,50,48,2,2,3")
    check("Door commanded updated to 50°", door_panel._commanded.text() == "50°")
    check("Door actual distinguished as estimate: 48° (est)", door_panel._actual.text() == "48° (est)")
    check("Door remaining travel updated to 2° to target", door_panel._travel.text() == "2° to target")
    check("Sequence mode updated from firmware to FULL SEQUENCE", page.sequence_panel._mode_val.text() == "FULL SEQUENCE")
    check("Sequence step updated from firmware to 3 / 3", page.sequence_panel._step_val.text() == "3 / 3")

    # 6. Command ACK matching
    rec = centre.send(VehicleID.ROCKET, "SET_MODE_LIVE")
    page.handle_command_reply("SD", "ACK", {}, "$H,ACK,SD")
    check("Live mode ACKed via SD", rec.status in (CommandStatus.ACKNOWLEDGED, CommandStatus.EXECUTED))


def main():
    test_command_mapping()
    test_ground_serial_framing()
    test_ground_response_parsing()
    test_command_lifecycle_in_command_centre()
    test_command_page_widgets()
    print("\nALL TEENSY 4.1 HARDWARE INTEGRATION TESTS PASSED!\n")


if __name__ == "__main__":
    main()
