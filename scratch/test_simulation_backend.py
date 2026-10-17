import sys
import time
from PyQt6.QtWidgets import QApplication

sys.path.insert(0, ".")

from ground_station.app import GroundStationApp
from ground_station.models import VehicleID, FlightState
from ground_station.pages.com import ComPage, VehicleSimulationControl
from ground_station.simulator.source import SimulatedSource
from ground_station.link import LinkSupervisor

app = QApplication.instance()
if app is None:
    app = QApplication([])

def test_simulation_lifecycle():
    print("\n--- TEST 1: VehicleSimulationControl State Machine & Idempotency ---")
    emitted_lines = []
    def on_line(line):
        emitted_lines.append(line)

    ctrl = VehicleSimulationControl(VehicleID.ROCKET, on_line)
    assert not ctrl.running and not ctrl.paused, "Initially IDLE"
    assert ctrl.status_lbl.text() == "IDLE"

    # Rapid Start
    ctrl.start()
    ctrl.start()
    ctrl.start()
    assert ctrl.running and not ctrl.paused, "Must be RUNNING"
    assert ctrl.status_lbl.text() == "RUNNING"

    # Step a few ticks
    for _ in range(5):
        ctrl._step()
    assert len(emitted_lines) == 5, f"Expected 5 lines, got {len(emitted_lines)}"
    t_paused = ctrl.t
    assert abs(t_paused - 0.5) < 1e-4

    # Rapid Pause
    ctrl.pause()
    ctrl.pause()
    ctrl.pause()
    assert not ctrl.running and ctrl.paused, "Must be PAUSED"
    assert ctrl.status_lbl.text() == "PAUSED"

    # Step while paused should NOT emit or advance time
    for _ in range(5):
        ctrl._step()
    assert len(emitted_lines) == 5, "No new lines while paused"
    assert ctrl.t == t_paused, f"Time must remain frozen at {t_paused}, got {ctrl.t}"

    # Resume with Start
    ctrl.start()
    assert ctrl.running and not ctrl.paused
    assert ctrl.status_lbl.text() == "RUNNING"
    ctrl._step()
    assert len(emitted_lines) == 6, "Resumed line emitted"
    assert abs(ctrl.t - (t_paused + 0.1)) < 1e-4, f"Time should advance smoothly from {t_paused}"

    # Reset
    ctrl.reset()
    assert not ctrl.running and not ctrl.paused
    assert ctrl.t == 0.0
    assert ctrl.status_lbl.text() == "IDLE"
    assert "T+0.0s" in ctrl.info_lbl.text()

    # Step while reset/idle should NOT emit
    ctrl._step()
    assert len(emitted_lines) == 6, "No lines emitted while idle/reset"

    print("  ok  VehicleSimulationControl lifecycle & idempotency passed!")

def test_ground_station_app_integration():
    print("\n--- TEST 2: GroundStationApp Full Simulation Pipeline ---")
    window = GroundStationApp(mode_label="SIMULATOR MODE", simulator_mode=True)
    source = SimulatedSource()
    window.connect_supervisor(source)
    source.connect_to()

    com_page = window.com_page
    assert source._external_driver, "SimulatedSource should have external_driver=True"
    assert not source._timer.isActive(), "SimulatedSource timer must NOT be running independently"
    assert com_page.sim_rocket.running, "sim_rocket started"
    assert com_page.sim_cansat.running, "sim_cansat started"

    # Step sim_rocket 10 times
    for _ in range(10):
        com_page.sim_rocket._step()

    # Step sim_cansat 10 times
    for _ in range(10):
        com_page.sim_cansat._step()

    assert com_page._valid_packets == 20, f"Expected 20 valid packets, got {com_page._valid_packets}"
    assert com_page._rejected_packets == 0, f"Expected 0 rejected packets, got {com_page._rejected_packets}"
    assert com_page._total_packets == 20, f"Expected total 20, got {com_page._total_packets}"
    assert com_page.rocket_stream.model.rowCount() == 10
    assert com_page.cansat_stream.model.rowCount() == 10

    # Verify rocket page and overview page received telemetry
    assert len(window.rocket_page.graphs["altitude"]._buffers[""]) == 10
    assert len(window.cansat_page.graphs["altitude"]._buffers[""]) == 10

    # Pause Rocket
    com_page.sim_rocket.pause()
    assert not com_page.sim_rocket.running and com_page.sim_rocket.paused

    # Step while rocket is paused and cansat is running
    for _ in range(5):
        com_page.sim_rocket._step()
        com_page.sim_cansat._step()

    assert com_page.rocket_stream.model.rowCount() == 10, "Rocket stream frozen"
    assert len(window.rocket_page.graphs["altitude"]._buffers[""]) == 10, "Rocket graph points frozen during pause"
    assert com_page.cansat_stream.model.rowCount() == 15, "CanSat stream progressed"
    assert len(window.cansat_page.graphs["altitude"]._buffers[""]) == 15, "CanSat graph points progressed"
    assert com_page._valid_packets == 25
    assert com_page._rejected_packets == 0

    # Reset Rocket
    com_page.sim_rocket.reset()
    assert com_page.rocket_stream.model.rowCount() == 0, "Rocket stream cleared on reset"
    assert com_page.cansat_stream.model.rowCount() == 15, "CanSat stream preserved"

    # Start Rocket again from 0
    com_page.sim_rocket.start()
    for _ in range(5):
        com_page.sim_rocket._step()

    assert com_page.rocket_stream.model.rowCount() == 5, "Rocket restarted cleanly"
    assert com_page._rejected_packets == 0, f"Rejections occurred after restart! Count: {com_page._rejected_packets}"
    assert com_page._valid_packets == 30

    print("  ok  GroundStationApp full simulation pipeline passed!")

def test_counter_consistency():
    print("\n--- TEST 3: Metric Counters Consistency ---")
    com_page = ComPage()
    assert com_page._total_packets == 0
    assert com_page._valid_packets == 0
    assert com_page._rejected_packets == 0

    com_page.on_rejected_line("CHECKSUM", "bad checksum", "bad line")
    assert com_page._rejected_packets == 1
    assert com_page._valid_packets == 0
    assert com_page._total_packets == 1
    assert com_page.reject_val.text() == "1"
    assert com_page.valid_val.text() == "0"
    assert com_page.total_pkts_val.text() == "0001"
    assert com_page.success_val.text() == "0.0%"

    print("  ok  Metric counters consistency passed!")

if __name__ == "__main__":
    test_simulation_lifecycle()
    test_ground_station_app_integration()
    test_counter_consistency()
    print("\nALL SIMULATION BACKEND TESTS PASSED SUCCESSFULLY!")
