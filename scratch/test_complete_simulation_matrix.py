import sys
import time
from PyQt6.QtWidgets import QApplication

sys.path.insert(0, ".")

from ground_station.app import GroundStationApp
from ground_station.models import VehicleID, FlightState, RejectReason
from ground_station.pages.com import ComPage, VehicleSimulationControl
from ground_station.simulator.source import SimulatedSource
from ground_station.link import LinkSupervisor

app = QApplication.instance()
if app is None:
    app = QApplication([])

def test_flight_state_progression_and_long_run():
    print("\n--- TEST 1: Long-Run Flight State Progression (1200 ticks = 120s flight) ---")
    window = GroundStationApp(mode_label="SIMULATOR MODE", simulator_mode=True)
    source = SimulatedSource()
    window.connect_supervisor(source)
    source.connect_to()

    com_page = window.com_page
    states_seen = set()

    # Run for 1200 ticks (120.0 simulation seconds)
    for _ in range(1200):
        com_page.sim_rocket._step()
        st = com_page.sim_rocket.flight.state_at(com_page.sim_rocket.t)
        states_seen.add(st)

    print(f"  Rocket states traversed: {[s.value for s in states_seen]}")
    expected_states = {
        FlightState.BOOT,
        FlightState.PRE_LAUNCH,
        FlightState.BOOST,
        FlightState.COAST,
        FlightState.APOGEE,
        FlightState.DESCENT,
        FlightState.LANDING,
        FlightState.RECOVERY
    }
    assert expected_states.issubset(states_seen), f"Missing states: {expected_states - states_seen}"
    assert com_page._rejected_packets == 0, f"Spurious rejections in long run: {com_page._rejected_packets}"
    assert com_page._valid_packets == 1200, f"Expected 1200 valid packets, got {com_page._valid_packets}"
    assert com_page._total_packets == 1200
    assert abs(com_page.sim_rocket.t - 120.0) < 1e-4, f"Time drift: {com_page.sim_rocket.t}"
    print("  ok  Long-run flight state progression passed with 0 rejections and 0 time drift!")

def test_rapid_clicking_matrix():
    print("\n--- TEST 2: Rapid Clicking Stress Matrix ---")
    ctrl = VehicleSimulationControl(VehicleID.ROCKET, lambda line: None)

    # Sequence of rapid actions
    for _ in range(20):
        ctrl.start()
    assert ctrl.running and not ctrl.paused

    for _ in range(20):
        ctrl.pause()
    assert not ctrl.running and ctrl.paused

    for _ in range(20):
        ctrl.start()
    assert ctrl.running and not ctrl.paused

    for _ in range(20):
        ctrl.reset()
    assert not ctrl.running and not ctrl.paused
    assert ctrl.t == 0.0
    assert ctrl.status_lbl.text() == "IDLE"

    print("  ok  Rapid clicking stress matrix passed!")

def test_live_mode_isolation():
    print("\n--- TEST 3: Live Mode Isolation ---")
    window = GroundStationApp(mode_label="LIVE", simulator_mode=False)
    source = LinkSupervisor()
    window.connect_supervisor(source)

    com_page = window.com_page
    # In live mode, sim_rocket and sim_cansat must be IDLE initially
    assert not com_page.sim_rocket.running
    assert not com_page.sim_cansat.running
    assert com_page.sim_rocket.status_lbl.text() == "IDLE"
    assert com_page.sim_cansat.status_lbl.text() == "IDLE"
    assert com_page._valid_packets == 0
    assert com_page._rejected_packets == 0

    print("  ok  Live mode isolation passed!")

if __name__ == "__main__":
    test_flight_state_progression_and_long_run()
    test_rapid_clicking_matrix()
    test_live_mode_isolation()
    print("\nALL EXTENDED TESTS PASSED SUCCESSFULLY!")
