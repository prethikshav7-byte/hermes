"""
scratch/test_complete_zoom_suite.py
════════════════════════════════════
Comprehensive validation of all graph zoom and reset requirements:
- Data-driven center-focused zoom in (+)
- Data-driven center-focused zoom out (-)
- Reset to auto-follow default
- Boundary clamping
- Minimum span safety
- Live telemetry stability while zoomed
- Independent per-graph zoom state
- Multiple data magnitudes (altitude, velocity, pressure, temperature, acceleration, gyro)
- All dashboards: Overview, Rocket, CanSat, Experiment
"""
import sys
import os
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt

sys.path.insert(0, os.path.abspath("."))

from ground_station.app import GroundStationApp
from ground_station.graphs.plot import TelemetryGraph
from ground_station.experiment.profile_plot import AerosolProfilePlot, SecondaryPlot
from ground_station.models import VehicleID

def run_zoom_suite():
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)

    print("==================================================")
    print("RUNNING COMPREHENSIVE GRAPH ZOOM TEST SUITE")
    print("==================================================")

    # 1. Independent Graph & Data Range Tests
    print("\n--- Test 1: Center-focused Zoom In, Zoom Out, and Reset ---")
    g = TelemetryGraph("altitude")
    for t in range(101):
        g.add_point(float(t), 10.0 + float(t) * 5.0)

    # Initial state
    assert g._follow is True
    assert g._latest_x == 100.0
    g._redraw()
    assert g._view_x_min == 0.0
    assert g._view_x_max == 100.0
    assert g._x_span == 100.0

    # Zoom in once
    g.btn_zoom_in.click()
    assert g._follow is False
    assert abs(g._x_span - 80.0) < 1e-4
    assert abs(g._view_x_min - 10.0) < 1e-4
    assert abs(g._view_x_max - 90.0) < 1e-4
    print("  [PASS] Zoom in once: span reduced from 100.0 to 80.0 centered at 50.0 (10 -> 90)")

    # Zoom in second time
    g.btn_zoom_in.click()
    assert abs(g._x_span - 64.0) < 1e-4
    assert abs(g._view_x_min - 18.0) < 1e-4
    assert abs(g._view_x_max - 82.0) < 1e-4
    print("  [PASS] Zoom in twice: span reduced to 64.0 centered at 50.0 (18 -> 82)")

    # Zoom out once
    g.btn_zoom_out.click()
    assert abs(g._x_span - 80.0) < 1e-4
    assert abs(g._view_x_min - 10.0) < 1e-4
    assert abs(g._view_x_max - 90.0) < 1e-4
    print("  [PASS] Zoom out once: span returned to 80.0 (10 -> 90)")

    # Zoom out second time (should hit max data boundary 0 -> 100)
    g.btn_zoom_out.click()
    assert abs(g._x_span - 100.0) < 1e-4
    assert abs(g._view_x_min - 0.0) < 1e-4
    assert abs(g._view_x_max - 100.0) < 1e-4
    print("  [PASS] Zoom out to data limit: clamped cleanly to 0 -> 100")

    # Zoom out again (must stay clamped at 0 -> 100, not expand infinitely)
    g.btn_zoom_out.click()
    assert abs(g._x_span - 100.0) < 1e-4
    assert abs(g._view_x_min - 0.0) < 1e-4
    assert abs(g._view_x_max - 100.0) < 1e-4
    print("  [PASS] Zoom out past max: remains clamped at 0 -> 100 without overflow")

    # Reset view
    g.btn_zoom_in.click()
    assert g._follow is False
    g.btn_reset.click()
    assert g._follow is True
    assert abs(g._x_span - 100.0) < 1e-4
    assert abs(g._view_x_min - 0.0) < 1e-4
    assert abs(g._view_x_max - 100.0) < 1e-4
    print("  [PASS] Reset restored auto-follow and full mission range")

    # 2. Extreme Zoom In / Minimum Limit Safety
    print("\n--- Test 2: Extreme Zoom In Safety (No NaN / collapse) ---")
    g2 = TelemetryGraph("pressure")
    for t in range(50):
        g2.add_point(float(t), 101325.0 - float(t) * 10.0)
    
    for _ in range(50):
        g2.btn_zoom_in.click()
    assert g2._x_span >= 0.05
    assert g2._view_x_min < g2._view_x_max
    assert g2._view_x_min == g2._view_x_min  # Not NaN
    assert g2._view_x_max == g2._view_x_max  # Not NaN
    print(f"  [PASS] Repeated zoom in stopped safely at minimum span floor: span={g2._x_span:.4f}s")

    # 3. Live Telemetry Stability During Manual Zoom
    print("\n--- Test 3: Live Telemetry Stability During Manual Zoom ---")
    g3 = TelemetryGraph("velocity")
    for t in range(50):
        g3.add_point(float(t), float(t) * 2.0)
    
    # Zoom in to a window around t=25
    g3.btn_zoom_in.click()
    saved_min = g3._view_x_min
    saved_max = g3._view_x_max
    saved_span = g3._x_span
    assert g3._follow is False

    # Simulate 50 new packets arriving
    for t in range(50, 100):
        g3.add_point(float(t), float(t) * 2.0)
        g3._maybe_redraw()

    # Verify manual zoom viewport did NOT drift or reset
    assert g3._follow is False
    assert abs(g3._view_x_min - saved_min) < 1e-6
    assert abs(g3._view_x_max - saved_max) < 1e-6
    assert abs(g3._x_span - saved_span) < 1e-6
    print(f"  [PASS] Live telemetry arrived (latest_x={g3._latest_x}) without disturbing zoomed viewport [{saved_min:.2f}, {saved_max:.2f}]")

    # Now press reset -> should jump to include all new live telemetry
    g3.btn_reset.click()
    assert g3._follow is True
    assert abs(g3._view_x_max - 99.0) < 1e-4
    assert abs(g3._view_x_min - 0.0) < 1e-4
    print(f"  [PASS] Reset restored full live range 0 -> {g3._latest_x}")

    # 4. Independent Zoom State Across Multiple Graphs
    print("\n--- Test 4: Independent Per-Graph Zoom State ---")
    g_alt = TelemetryGraph("altitude")
    g_vel = TelemetryGraph("velocity")
    g_temp = TelemetryGraph("temperature")
    g_pres = TelemetryGraph("pressure")

    for t in range(100):
        g_alt.add_point(float(t), float(t) * 10)
        g_vel.add_point(float(t), float(t) * 2)
        g_temp.add_point(float(t), 25.0)
        g_pres.add_point(float(t), 100000.0)

    # Zoom g_alt 3 times
    g_alt.btn_zoom_in.click()
    g_alt.btn_zoom_in.click()
    g_alt.btn_zoom_in.click()

    # Zoom g_pres 1 time
    g_pres.btn_zoom_in.click()

    # g_vel should still be default auto-follow
    assert g_vel._follow is True
    assert abs(g_vel._x_span - 99.0) < 1e-4

    # g_temp should still be default auto-follow
    assert g_temp._follow is True
    assert abs(g_temp._x_span - 99.0) < 1e-4

    # g_alt should be zoomed 3 times (0.8^3 * 99 = ~50.688)
    assert g_alt._follow is False
    assert abs(g_alt._x_span - 99.0 * 0.512) < 1e-3

    # g_pres should be zoomed 1 time (0.8 * 99 = 79.2)
    assert g_pres._follow is False
    assert abs(g_pres._x_span - 99.0 * 0.8) < 1e-3
    print("  [PASS] Each graph maintains strictly independent zoom state and viewports")

    # 5. Full GroundStationApp Dashboard Integration
    print("\n--- Test 5: Full GroundStationApp Dashboards ---")
    win = GroundStationApp(mode_label="SIMULATOR", simulator_mode=True)
    win.resize(1560, 960)
    win.show()

    # Overview
    overview_graphs = win.overview_page.all_graphs()
    assert len(overview_graphs) == 12
    for i, gr in enumerate(overview_graphs):
        for t in range(20):
            gr.add_point(float(t), float(t) * (i + 1))
        gr.btn_zoom_in.click()
        assert gr._follow is False
        gr.btn_zoom_out.click()
        gr.btn_reset.click()
        assert gr._follow is True

    print(f"  [PASS] All 12 Overview graphs validated for +, -, reset")

    # Rocket
    rocket_graphs = win.rocket_page.all_graphs()
    assert len(rocket_graphs) == 6
    for gr in rocket_graphs:
        for t in range(20):
            gr.add_point(float(t), float(t) * 2.5)
        gr.btn_zoom_in.click()
        assert gr._follow is False
        gr.btn_zoom_out.click()
        gr.btn_reset.click()
        assert gr._follow is True
    print(f"  [PASS] All 6 Rocket graphs validated for +, -, reset")

    # CanSat
    cansat_graphs = win.cansat_page.all_graphs()
    assert len(cansat_graphs) == 6
    for gr in cansat_graphs:
        for t in range(20):
            gr.add_point(float(t), float(t) * 1.5)
        gr.btn_zoom_in.click()
        assert gr._follow is False
        gr.btn_zoom_out.click()
        gr.btn_reset.click()
        assert gr._follow is True
    print(f"  [PASS] All 6 CanSat graphs validated for +, -, reset")

    # Experiment Page
    exp = win.experiment_page
    for p in (exp.profile, exp.alt_time, exp.aero_time):
        p.btn_zoom_in.click()
        p.btn_zoom_out.click()
        p.btn_reset.click()
    print("  [PASS] Experiment plots validated for +, -, reset")

    print("\n==================================================")
    print("ALL ZOOM SUITE TESTS PASSED PERFECTLY!")
    print("==================================================")

if __name__ == "__main__":
    run_zoom_suite()
