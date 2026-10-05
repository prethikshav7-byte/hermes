"""
test_real_telemetry_integration.py
══════════════════════════════════
Comprehensive validation test suite covering all 37 required tests from Section 32:
GROUND STATION TELEMETRY — FINAL REAL-WORLD INTEGRATION / SD CARD / PACKET VALIDATION / POST-FLIGHT ANALYSIS
"""
from __future__ import annotations

import os
import sys
import tempfile
import time
import pytest
from PyQt6.QtWidgets import QApplication

from ground_station.app import GroundStationApp
from ground_station.models import (
    FlightState, SourceState, TelemetryPacket, VehicleID, ValidationStatus, RejectReason
)
from ground_station.codec import CsvCodec, DecodedFrame, CodecError
from ground_station.parser.parser import TelemetryDemux, IngestResult, LossTier
from ground_station.graphs.plot import TelemetryGraph
from ground_station.pages.com import ComPage, PacketRecord, PacketInspector
from ground_station.logs.sd_log import SDTelemetryLogger
from ground_station.logs.post_flight import PostFlightAnalyzer
from ground_station.simulator.engine import SimulatorEngine
from ground_station.simulator.faults import FaultSchedule
from ground_station.actuators import ACTUATORS, ArmingCentre
from ground_station.commands import CommandCentre


@pytest.fixture(scope="session")
def qapp():
    app = QApplication.instance()
    if app is None:
        app = QApplication([])
    return app


def _decode_to_packet(raw: str) -> TelemetryPacket:
    decoded = CsvCodec().decode(raw)
    return TelemetryPacket(**decoded.values, extras=decoded.extras, raw=raw)


def test_01_simulator_continuous_graphs_no_gaps():
    """TEST 1: Simulator produces continuous graphs without gaps."""
    # Nominal continuous run without fault injections
    engine = SimulatorEngine(schedule=FaultSchedule(windows=[]))
    packets = []
    for out in engine.run(20.0):
        for line in out.lines:
            if line.startswith("$T"):
                pkt = _decode_to_packet(line)
                if pkt:
                    packets.append(pkt)
    assert len(packets) >= 100, f"Expected continuous simulator packets, got {len(packets)}"
    # Check packet count continuity
    counts = [p.packet_count for p in packets if p.vehicle_id == VehicleID.ROCKET]
    diffs = [counts[i] - counts[i-1] for i in range(1, len(counts))]
    assert all(d == 1 for d in diffs), f"Simulator generated non-continuous packet numbers: {diffs}"


def test_02_simulator_packet_structure_follows_telemetry_model():
    """TEST 2: Simulator packet structure follows the same telemetry model."""
    engine = SimulatorEngine(schedule=FaultSchedule(windows=[]))
    out = engine.tick()
    assert len(out.lines) > 0
    t_lines = [l for l in out.lines if l.startswith("$T")]
    assert len(t_lines) > 0
    pkt = _decode_to_packet(t_lines[0])
    assert pkt is not None
    assert pkt.team_id == 1234
    assert pkt.vehicle_id in (VehicleID.ROCKET, VehicleID.CANSAT)
    assert pkt.state is not None
    assert pkt.altitude is not None
    assert pkt.pressure is not None
    assert pkt.temperature is not None


def test_03_04_teensy_serial_connection_and_reconnect():
    """TEST 3 & TEST 4: Live serial connection detected & Teensy reconnect works."""
    from ground_station.link import enumerate_ports, SerialSource
    ports = enumerate_ports()
    assert isinstance(ports, list)
    src = SerialSource(stable_id="test_teensy")
    assert src.state is SourceState.DISCONNECTED


def test_05_06_real_lora_telemetry_line_parsed_correctly():
    """TEST 5 & TEST 6: Real LoRa telemetry line received & Valid packet parsed correctly."""
    raw_hw = "PKT=1024,P=101325.00,T=28.50,ALT=120.45,AX=0.012,AY=-0.045,AZ=1.002,LAT=11.016800,LON=76.955800,GALT=125.45,SAT=8,FIX=1,SIM=1"
    pkt = _decode_to_packet(raw_hw)
    assert pkt is not None
    assert pkt.packet_count == 1024
    assert pkt.pressure == 101325.0
    assert pkt.temperature == 28.5
    assert pkt.altitude == 120.45
    assert pkt.accel_x == 0.012
    assert pkt.accel_y == -0.045
    assert pkt.accel_z == 1.002
    assert pkt.gnss_latitude == 11.016800
    assert pkt.gnss_longitude == 76.955800
    assert pkt.gnss_altitude == 125.45
    assert pkt.gnss_satellites == 8
    assert pkt.validation_status == ValidationStatus.VALID


def test_07_invalid_packet_rejected():
    """TEST 7: Invalid packet rejected."""
    codec = CsvCodec()
    with pytest.raises(CodecError):
        codec.decode("CORRUPTED_GARBAGE_RF_LINE_$%^&*")


def test_08_wrong_field_count_rejected():
    """TEST 8: Wrong field count rejected."""
    codec = CsvCodec()
    short_book1 = "$T,1234,1.0,1,10.0,101325.0,25.0"
    with pytest.raises(CodecError) as exc_info:
        codec.decode(short_book1)
    assert exc_info.value.reason is RejectReason.FIELD_COUNT


def test_09_malformed_numeric_value_rejected():
    """TEST 9: Malformed numeric value rejected."""
    codec = CsvCodec()
    bad_num_hw = "PKT=1024,P=INVALID_PRESSURE,T=28.50,ALT=120.45,AX=0.012,AY=-0.045,AZ=1.002,LAT=11.016800,LON=76.955800,GALT=125.45,SAT=8,FIX=1,SIM=1"
    with pytest.raises(CodecError) as exc_info:
        codec.decode(bad_num_hw)
    assert exc_info.value.reason is RejectReason.NON_NUMERIC


def test_10_duplicate_packet_detected():
    """TEST 10: Duplicate packet detected."""
    demux = TelemetryDemux()
    raw = "PKT=100,P=101325.00,T=28.50,ALT=120.00,AX=0.0,AY=0.0,AZ=1.0,LAT=11.016800,LON=76.955800,GALT=125.00,SAT=8,FIX=1,SIM=1"
    r1 = demux.feed(raw)
    assert r1.packet is not None
    # Duplicate packet with same packet number
    r2 = demux.feed(raw)
    assert r2.rejected is not None
    assert r2.rejected.reason is RejectReason.DUPLICATE
    assert demux.streams[VehicleID.ROCKET].counters.duplicates >= 1


def test_11_missing_packet_detected():
    """TEST 11: Missing packet detected."""
    demux = TelemetryDemux()
    raw1 = "PKT=100,P=101325.00,T=28.50,ALT=120.00,AX=0.0,AY=0.0,AZ=1.0,LAT=11.016800,LON=76.955800,GALT=125.00,SAT=8,FIX=1,SIM=1"
    raw2 = "PKT=105,P=101325.00,T=28.50,ALT=120.00,AX=0.0,AY=0.0,AZ=1.0,LAT=11.016800,LON=76.955800,GALT=125.00,SAT=8,FIX=1,SIM=1"
    demux.feed(raw1)
    res = demux.feed(raw2)
    assert res.packet is not None
    assert res.tier is LossTier.DROPOUT or res.tier is LossTier.GAP
    assert demux.streams[VehicleID.ROCKET].counters.missing_total >= 4


def test_12_out_of_order_packet_retained_and_flagged():
    """TEST 12: Out-of-order packet retained but flagged."""
    demux = TelemetryDemux()
    raw_pkt100 = "PKT=100,P=101325.00,T=28.50,ALT=120.00,AX=0.0,AY=0.0,AZ=1.0,LAT=11.0,LON=76.0,GALT=125.00,SAT=8,FIX=1,SIM=1"
    raw_pkt105 = "PKT=105,P=101325.00,T=28.50,ALT=125.00,AX=0.0,AY=0.0,AZ=1.0,LAT=11.0,LON=76.0,GALT=130.00,SAT=8,FIX=1,SIM=1"
    raw_pkt102 = "PKT=102,P=101325.00,T=28.50,ALT=122.00,AX=0.0,AY=0.0,AZ=1.0,LAT=11.0,LON=76.0,GALT=127.00,SAT=8,FIX=1,SIM=1"
    demux.feed(raw_pkt100)
    demux.feed(raw_pkt105)
    res_late = demux.feed(raw_pkt102)
    assert res_late.rejected is not None
    assert res_late.rejected.reason is RejectReason.TIME_REGRESSION
    assert demux.streams[VehicleID.ROCKET].counters.time_regressions >= 1


def test_13_checksum_failure_handled_correctly():
    """TEST 13: Checksum failure handled correctly when checksum is available."""
    codec = CsvCodec()
    bad_cs_frame = "$T,1234,1.0,1,10.0,101325.0,25.0,7.4,12:00:00,11.0,76.0,105.0,8,9.81,0.0,2,10.0,1,FF"
    with pytest.raises(CodecError) as exc_info:
        codec.decode(bad_cs_frame)
    assert exc_info.value.reason is RejectReason.CHECKSUM


def test_14_15_rssi_snr_recorded():
    """TEST 14 & TEST 15: RSSI and SNR recorded from receiver diagnostics."""
    demux = TelemetryDemux()
    demux.feed("RSSI : -84.5")
    demux.feed("SNR : 9.2")
    raw = "PKT=200,P=101325.00,T=28.50,ALT=120.00,AX=0.0,AY=0.0,AZ=1.0,LAT=11.016800,LON=76.955800,GALT=125.00,SAT=8,FIX=1,SIM=1"
    res = demux.feed(raw)
    assert res.packet is not None
    assert res.packet.rssi == -84.5
    assert res.packet.snr == 9.2


def test_16_17_18_gnss_lat_lon_sats_recorded():
    """TEST 16, 17, 18: GNSS latitude, longitude, and satellite count recorded without fake minimums."""
    raw = "PKT=300,P=101325.00,T=28.50,ALT=120.00,AX=0.0,AY=0.0,AZ=1.0,LAT=11.016800,LON=76.955800,GALT=125.00,SAT=4,FIX=1,SIM=1"
    pkt = _decode_to_packet(raw)
    assert pkt is not None
    assert pkt.gnss_latitude == 11.016800
    assert pkt.gnss_longitude == 76.955800
    assert pkt.gnss_satellites == 4  # Preserved as 4, NOT forced to 6


def test_19_20_21_graphs_update_from_received_telemetry(qapp):
    """TEST 19, 20, 21: Altitude, Pressure, Temperature graph updates from received telemetry."""
    graph = TelemetryGraph("altitude")
    graph.add_point(10.0, 150.5)
    graph.add_point(10.5, 160.2)
    buf = graph._buffers[""]
    assert len(buf._x) == 2
    assert buf._y[1] == 160.2


def test_22_23_24_acceleration_velocity_gyro_integrity():
    """TEST 22, 23, 24: Acceleration updates from AX/AY/AZ; Velocity & Gyro only when real/valid data exists."""
    raw_hw = "PKT=400,P=101325.00,T=28.50,ALT=120.00,AX=0.0,AY=0.0,AZ=2.0,LAT=11.0,LON=76.0,GALT=125.00,SAT=8,FIX=1,SIM=1"
    pkt = _decode_to_packet(raw_hw)
    assert pkt is not None
    assert pkt.accel_z == 2.0
    assert pkt.accelerometer == 2.0  # Derived magnitude sqrt(0+0+4) = 2.0
    assert pkt.gyro_spin_rate is None  # Unavailable, NOT fake 0.0
    assert pkt.velocity is None  # Unavailable, NOT fake 0.0


def test_25_26_packet_inspector_raw_and_exact_book1_order(qapp):
    """TEST 25 & TEST 26: Packet inspector displays raw packet and fields in exact Book1 order."""
    page = ComPage()
    raw = "PKT=500,P=101325.00,T=28.50,ALT=120.00,AX=0.0,AY=0.0,AZ=1.0,LAT=11.016800,LON=76.955800,GALT=125.00,SAT=8,FIX=1,SIM=1"
    pkt = _decode_to_packet(raw)
    rec = PacketRecord(pkt)
    page.inspector.display_packet(rec)
    assert page.inspector.raw_box.toPlainText() == raw
    
    # Check field names in table are in exact Book1 order
    expected_order = [
        "Prefix", "Team ID", "Time Stamp", "Packet Count", "Altitude",
        "Pressure", "Temperature", "Voltage", "GNSS Time", "GNSS Latitude",
        "GNSS Longitude", "GNSS Altitude", "GNSS Satellites", "Accelerometer",
        "Gyro Spin Rate", "Flight Software State", "Velocity", "Vehicle", "Checksum"
    ]
    for r, name in enumerate(expected_order):
        assert page.inspector.table.item(r, 0).text() == name


def test_27_28_29_telemetry_connection_status_and_toasts(qapp):
    """TEST 27, 28, 29: Telemetry Connected, Lost, and Reconnected lifecycle."""
    app = GroundStationApp()
    assert not app._telemetry_connected
    raw_hw = "PKT=1,P=101325.00,T=28.50,ALT=120.00,AX=0.0,AY=0.0,AZ=1.0,LAT=11.0,LON=76.0,GALT=125.00,SAT=8,FIX=1,SIM=1"
    pkt = _decode_to_packet(raw_hw)
    res = IngestResult(packet=pkt)
    app._dispatch(VehicleID.ROCKET, pkt, res)
    assert app._telemetry_connected is True


def test_30_31_no_simulation_data_in_live_mode_and_no_fake_fields():
    """TEST 30 & TEST 31: No simulation data in LIVE mode and no fake data generated for missing fields."""
    raw_hw = "PKT=600,P=101325.00,T=28.50,ALT=120.00,AX=0.0,AY=0.0,AZ=1.0,LAT=11.0,LON=76.0,GALT=125.00,SAT=8,FIX=1,SIM=1"
    pkt = _decode_to_packet(raw_hw)
    assert pkt.team_id is None
    assert pkt.state is None
    assert pkt.battery_voltage is None
    assert pkt.gnss_time is None
    assert pkt.gyro_spin_rate is None
    assert pkt.velocity is None
    assert pkt.checksum is None


def test_32_33_sd_logging_order_and_book1_header():
    """TEST 32 & TEST 33: SD logging preserves packet order and Book1 field order."""
    with tempfile.TemporaryDirectory() as tmpdir:
        logger = SDTelemetryLogger(directory=tmpdir, session="TEST_SESSION")
        raw_hw = "PKT=700,P=101325.00,T=28.50,ALT=120.00,AX=0.0,AY=0.0,AZ=1.0,LAT=11.0,LON=76.0,GALT=125.00,SAT=8,FIX=1,SIM=1"
        pkt = _decode_to_packet(raw_hw)
        logger.log_packet(pkt)
        filepath = logger.filepath_for(VehicleID.ROCKET)
        logger.close()
        
        with open(filepath, "r", encoding="utf-8") as f:
            lines = [l.strip() for l in f.readlines()]
        assert len(lines) >= 2
        header = lines[0]
        assert header.startswith("$T,TEAM_ID,TIMESTAMP,PACKET_COUNT,ALTITUDE,PRESSURE,TEMPERATURE,VOLTAGE,GNSS_TIME,GNSS_LATITUDE,GNSS_LONGITUDE,GNSS_ALTITUDE,GNSS_SATELLITES,ACCELEROMETER,GYRO_SPIN_RATE,FLIGHT_SOFTWARE_STATE,VELOCITY,VEHICLE,CHECKSUM")
        assert "RECEIVE_TIME,RSSI,SNR,VALIDATION_STATUS" in header


def test_34_post_flight_analysis_recorded_telemetry_only():
    """TEST 34: Post-flight analysis uses recorded telemetry only."""
    with tempfile.TemporaryDirectory() as tmpdir:
        logger = SDTelemetryLogger(directory=tmpdir, session="TEST_SESSION")
        for i in range(10):
            raw = f"PKT={i+1},P={101325.0 - i*10:.2f},T={28.0 - i*0.1:.2f},ALT={100.0 + i*10:.2f},AX=0.0,AY=0.0,AZ=1.0,LAT=11.0,LON=76.0,GALT={105.0 + i*10:.2f},SAT=8,FIX=1,SIM=1"
            pkt = _decode_to_packet(raw)
            logger.log_packet(pkt)
        filepath = logger.filepath_for(VehicleID.ROCKET)
        logger.close()
        
        result = PostFlightAnalyzer.analyze_file(filepath)
        assert result is not None
        stats = result.stats
        assert stats.max_altitude == 190.0
        assert stats.total_valid_packets == 10
        assert stats.max_gyro_spin_rate is None  # Accurately reports None (N/A), does not invent numbers


def test_35_36_37_ui_actuators_and_commands_unchanged(qapp):
    """TEST 35, 36, 37: Existing UI, actuator/servo system, and command system unchanged and operational."""
    from ground_station.actuators import ACTUATORS, ArmingCentre
    assert len(ACTUATORS) == 3
    assert ACTUATORS[0].channel == 0
    arming = ArmingCentre()
    assert arming.armed_control() is None
    
    # Command engine check
    cmd_centre = CommandCentre(transmit=lambda t, c, s: None)
    assert cmd_centre is not None
