import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from PyQt6.QtWidgets import QApplication
from ground_station.app import GroundStationApp
from ground_station.simulator.source import SimulatedSource
from ground_station.link import LinkSupervisor
from ground_station.models import VehicleID, VehicleLinkState, ReceiverState, SourceState
from ground_station.actuators import CalibrationOutcome

def test_simulator_execution():
    app = QApplication.instance() or QApplication(sys.argv)
    
    # 1. Initialize GroundStationApp in Simulator mode (as clicked in Launcher)
    window = GroundStationApp(mode_label="SIMULATOR MODE", simulator_mode=True)
    source = SimulatedSource()
    window.connect_supervisor(source)
    source.connect_to()

    print(">>> Supervisor connected and started.")
    
    # 2. Process Qt events to let timers and simulated vehicles tick
    for _ in range(15):
        app.processEvents()
        time.sleep(0.05)

    # 3. Verify telemetry flow into COM page and dashboards
    print(f"Total packets received in COM page: {window.com_page._total_packets}")
    print(f"Rocket stream rows: {window.com_page.rocket_stream.model.rowCount()}")
    print(f"CanSat stream rows: {window.com_page.cansat_stream.model.rowCount()}")
    assert window.com_page._total_packets > 0, "No packets received by COM page!"
    assert window.com_page.rocket_stream.model.rowCount() > 0, "Rocket stream empty!"
    assert window.com_page.cansat_stream.model.rowCount() > 0, "CanSat stream empty!"

    # 4. Verify link states
    print(f"Source state: {window.header.pill_status.text().encode('ascii', 'replace').decode('ascii')}")
    print(f"Rocket link state: {source.vehicle_link_state(VehicleID.ROCKET)}")
    print(f"CanSat link state: {source.vehicle_link_state(VehicleID.CANSAT)}")
    assert source.vehicle_link_state(VehicleID.ROCKET) == VehicleLinkState.RECEIVING
    assert source.vehicle_link_state(VehicleID.CANSAT) == VehicleLinkState.RECEIVING

    # 5. Verify Command Page functionality in simulation
    print(">>> Testing command transmission and feedback in simulation...")
    # Send an actuator command via CommandCentre
    window.command_centre.send(VehicleID.ROCKET, "DOOR:30")
    app.processEvents()
    time.sleep(0.05)
    app.processEvents()

    door_panel = window.command_page.actuator_panels.get("ACT_DOOR")
    print(f"ACT_DOOR panel angle after DOOR:30: {door_panel.actuator.actual}")
    assert door_panel.actuator.actual == 30.0, f"Expected 30.0 deg, got {door_panel.actuator.actual}"

    # Send Full Sequence command
    window.command_centre.send(VehicleID.ROCKET, "FSEQ")
    app.processEvents()
    time.sleep(0.05)
    app.processEvents()

    print(f"Sequence panel mode: {window.command_page.sequence_panel._mode}, step: {window.command_page.sequence_panel._step}")
    assert window.command_page.sequence_panel._mode == "FULL SEQUENCE"
    assert window.command_page.sequence_panel._step == 3

    # Send Altitude Calibration
    window.command_centre.send(VehicleID.ROCKET, "CALA")
    app.processEvents()
    time.sleep(0.05)
    app.processEvents()

    cal = window.command_page.calibration.get(VehicleID.ROCKET)
    alt_res = cal.results.get("ALTITUDE")
    print(f"CALA outcome: {alt_res.value if alt_res else 'None'}")
    assert alt_res == CalibrationOutcome.PASS

    # 6. Verify Navigation between pages
    for i in range(len(window._nav)):
        window._navigate(i)
        app.processEvents()
    print(">>> Page navigation tested successfully.")

    source.disconnect_source()
    print(">>> SIMULATOR TEST PASSED 100%!")


def test_live_mode_init():
    app = QApplication.instance() or QApplication(sys.argv)
    window = GroundStationApp(mode_label="LIVE", simulator_mode=False)
    supervisor = LinkSupervisor()
    window.connect_supervisor(supervisor)
    # Test connection attempt with dummy/empty device
    supervisor.connect_to(stable_id="", baudrate=115200, device_hint=None)
    app.processEvents()
    print(f"Live supervisor initialized, source state: {supervisor.source_state.value}")
    supervisor.disconnect_source()
    print(">>> LIVE MODE INIT TEST PASSED 100%!")


if __name__ == "__main__":
    test_simulator_execution()
    test_live_mode_init()
