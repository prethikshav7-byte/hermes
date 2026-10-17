"""
scratch/test_graph_panning_suite.py
═══════════════════════════════════
Comprehensive validation test suite for individual graph panning / navigation:
- Horizontal panning (Left & Right)
- Vertical panning (Up & Down)
- Viewport width/height preservation during pan
- Data boundary clamping (X and Y limits)
- Mouse drag interaction simulation on ViewBox
- Independent per-graph pan states
- Zoom + Pan combination workflows
- Live telemetry stability while panned
- Reset restoration to auto-follow and auto-scale
- Multi-dashboard testing (Overview, Rocket, CanSat, Experiment)
"""
import sys
import os
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt, QPoint
from PyQt6.QtTest import QTest

sys.path.insert(0, os.path.abspath("."))

from ground_station.app import GroundStationApp
from ground_station.graphs.plot import TelemetryGraph
from ground_station.models import VehicleID

def run_panning_suite():
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)

    print("==================================================")
    print("RUNNING COMPREHENSIVE GRAPH PANNING TEST SUITE")
    print("==================================================")

    # ─────────────────────────────────────────────────────────────────
    # 1. Horizontal Panning (Left & Right) + Width Preservation
    # ─────────────────────────────────────────────────────────────────
    print("\n--- Test 1: Horizontal Panning (Left & Right) ---")
    g = TelemetryGraph("altitude")
    for t in range(101):
        g.add_point(float(t), 10.0 + float(t) * 5.0)

    # Initial full mission range: 0 -> 100
    assert g._follow is True
    assert g._view_x_min == 0.0
    assert g._view_x_max == 100.0

    # Zoom in twice: span should become 64.0 (18 -> 82)
    g.btn_zoom_in.click()
    g.btn_zoom_in.click()
    initial_span = g._x_span
    assert abs(initial_span - 64.0) < 1e-3
    assert abs(g._view_x_min - 18.0) < 1e-3
    assert abs(g._view_x_max - 82.0) < 1e-3

    # Pan Left by 10s (move viewport to earlier time: 8 -> 72)
    g.pan_by(-10.0, 0.0)
    assert abs(g._view_x_min - 8.0) < 1e-3
    assert abs(g._view_x_max - 72.0) < 1e-3
    assert abs(g._x_span - initial_span) < 1e-3
    assert g._follow is False
    print("  [PASS] Pan Left by 10s: viewport shifted from 18->82 to 8->72 with span preserved (64.0)")

    # Pan Right by 20s (move viewport to later time: 28 -> 92)
    g.pan_by(20.0, 0.0)
    assert abs(g._view_x_min - 28.0) < 1e-3
    assert abs(g._view_x_max - 92.0) < 1e-3
    assert abs(g._x_span - initial_span) < 1e-3
    print("  [PASS] Pan Right by 20s: viewport shifted to 28->92 with span preserved (64.0)")

    # ─────────────────────────────────────────────────────────────────
    # 2. Horizontal Boundary Clamping
    # ─────────────────────────────────────────────────────────────────
    print("\n--- Test 2: Horizontal Boundary Clamping ---")
    # Pan far left beyond 0.0 -> must clamp to 0 -> 64
    g.pan_by(-100.0, 0.0)
    assert abs(g._view_x_min - 0.0) < 1e-3
    assert abs(g._view_x_max - 64.0) < 1e-3
    assert abs(g._x_span - initial_span) < 1e-3
    print("  [PASS] Left boundary clamp: clamped precisely to 0.0 -> 64.0 without span distortion")

    # Pan far right beyond 100.0 -> must clamp to 36 -> 100
    g.pan_by(200.0, 0.0)
    assert abs(g._view_x_min - 36.0) < 1e-3
    assert abs(g._view_x_max - 100.0) < 1e-3
    assert abs(g._x_span - initial_span) < 1e-3
    print("  [PASS] Right boundary clamp: clamped precisely to 36.0 -> 100.0 without span distortion")

    # ─────────────────────────────────────────────────────────────────
    # 3. Vertical Panning (Up & Down) + Height Preservation
    # ─────────────────────────────────────────────────────────────────
    print("\n--- Test 3: Vertical Panning (Up & Down) ---")
    g_alt = TelemetryGraph("altitude")
    for t in range(50):
        g_alt.add_point(float(t), 200.0 + float(t) * 10.0)
    g_alt._redraw()

    # Default altitude range
    y_min_0 = g_alt._view_y_min
    y_max_0 = g_alt._view_y_max
    y_span_0 = y_max_0 - y_min_0

    # Pan Up by +100m
    g_alt.pan_by(0.0, 100.0)
    assert abs(g_alt._view_y_min - (y_min_0 + 100.0)) < 1e-3
    assert abs(g_alt._view_y_max - (y_max_0 + 100.0)) < 1e-3
    assert abs((g_alt._view_y_max - g_alt._view_y_min) - y_span_0) < 1e-3
    assert g_alt._manual_y is True
    print(f"  [PASS] Pan Up by 100m: Y range shifted [{y_min_0:.1f}, {y_max_0:.1f}] -> [{g_alt._view_y_min:.1f}, {g_alt._view_y_max:.1f}]")

    # Pan Down by -150m
    g_alt.pan_by(0.0, -150.0)
    assert abs(g_alt._view_y_min - (y_min_0 - 50.0)) < 1e-3 or g_alt._view_y_min == 0.0
    assert abs((g_alt._view_y_max - g_alt._view_y_min) - y_span_0) < 1e-3
    print(f"  [PASS] Pan Down by 150m: Y range shifted to [{g_alt._view_y_min:.1f}, {g_alt._view_y_max:.1f}]")

    # Pan far down past hard floor (0.0 for Altitude) -> must clamp to 0.0 floor
    g_alt.pan_by(0.0, -2000.0)
    assert g_alt._view_y_min >= 0.0
    assert abs((g_alt._view_y_max - g_alt._view_y_min) - y_span_0) < 1e-3
    print(f"  [PASS] Vertical hard floor clamp: Y range clamped with floor >= 0.0: [{g_alt._view_y_min:.1f}, {g_alt._view_y_max:.1f}]")

    # ─────────────────────────────────────────────────────────────────
    # 4. Reset Restores Both X & Y Auto Viewports
    # ─────────────────────────────────────────────────────────────────
    print("\n--- Test 4: Reset Viewport Restoration ---")
    g_alt.btn_reset.click()
    assert g_alt._follow is True
    assert g_alt._manual_y is False
    assert abs(g_alt._view_x_min - 0.0) < 1e-3
    assert abs(g_alt._view_x_max - 49.0) < 1e-3
    print("  [PASS] Reset restored auto-follow and auto-scale for both X and Y axes")

    # ─────────────────────────────────────────────────────────────────
    # 5. Live Telemetry Stability During Manual Pan
    # ─────────────────────────────────────────────────────────────────
    print("\n--- Test 5: Live Telemetry Stability During Manual Pan ---")
    g_live = TelemetryGraph("pressure")
    for t in range(40):
        g_live.add_point(float(t), 101325.0 - float(t) * 100.0)
    
    # Zoom and Pan
    g_live.btn_zoom_in.click()
    g_live.pan_by(-5.0, 200.0)
    panned_x_min = g_live._view_x_min
    panned_x_max = g_live._view_x_max
    panned_y_min = g_live._view_y_min
    panned_y_max = g_live._view_y_max

    # 50 new packets arrive
    for t in range(40, 90):
        g_live.add_point(float(t), 101325.0 - float(t) * 100.0)
        g_live._maybe_redraw()

    # Verify manual panned viewport remained completely stable
    assert g_live._follow is False
    assert g_live._manual_y is True
    assert abs(g_live._view_x_min - panned_x_min) < 1e-5
    assert abs(g_live._view_x_max - panned_x_max) < 1e-5
    assert abs(g_live._view_y_min - panned_y_min) < 1e-5
    assert abs(g_live._view_y_max - panned_y_max) < 1e-5
    print("  [PASS] Live telemetry arrived without shifting or overwriting manual panned X or Y viewports")

    # ─────────────────────────────────────────────────────────────────
    # 6. Mouse Drag Interaction Simulation on ViewBox
    # ─────────────────────────────────────────────────────────────────
    print("\n--- Test 6: Mouse Drag Direct Interaction Simulation ---")
    g_mouse = TelemetryGraph("velocity")
    g_mouse.resize(500, 300)
    g_mouse.show()
    for t in range(100):
        g_mouse.add_point(float(t), float(t) * 2.0)
    g_mouse.btn_zoom_in.click()
    before_drag_min = g_mouse._view_x_min
    before_drag_max = g_mouse._view_x_max

    # Simulate mouse drag on plot widget viewport
    vp = g_mouse.plot.viewport()
    QTest.mousePress(vp, Qt.MouseButton.LeftButton, pos=QPoint(250, 150))
    QTest.mouseMove(vp, pos=QPoint(200, 150)) # Drag left by 50px
    QTest.mouseRelease(vp, Qt.MouseButton.LeftButton, pos=QPoint(200, 150))

    # Viewport X should have moved to later data (panned right)
    assert g_mouse._follow is False
    assert g_mouse._view_x_min > before_drag_min
    assert g_mouse._view_x_max > before_drag_max
    assert abs((g_mouse._view_x_max - g_mouse._view_x_min) - (before_drag_max - before_drag_min)) < 1e-2
    print("  [PASS] Mouse drag directly panned the graph viewport with width preserved")

    # ─────────────────────────────────────────────────────────────────
    # 7. Independent Per-Graph Pan States Across Dashboard
    # ─────────────────────────────────────────────────────────────────
    print("\n--- Test 7: Independent Per-Graph Pan States Across Dashboard ---")
    win = GroundStationApp(mode_label="SIMULATOR", simulator_mode=True)
    win.resize(1560, 960)
    win.show()

    rocket_graphs = win.rocket_page.all_graphs()
    for g in rocket_graphs:
        for t in range(50):
            g.add_point(float(t), 50.0 + float(t))

    g_r0 = rocket_graphs[0]
    g_r1 = rocket_graphs[1]

    # Zoom & pan first graph only
    g_r0.btn_zoom_in.click()
    g_r0.pan_by(-5.0, 10.0)

    # Second graph should remain unpanned in auto-follow
    assert g_r0._follow is False
    assert g_r1._follow is True
    assert g_r1._manual_y is False
    assert g_r1._view_x_min == 0.0
    print("  [PASS] Panning Rocket graph 0 did not affect Rocket graph 1")

    # ─────────────────────────────────────────────────────────────────
    # 8. Complete Workflow Sequence (Reset -> + -> + -> Pan -> - -> Pan -> Reset)
    # ─────────────────────────────────────────────────────────────────
    print("\n--- Test 8: Complete Workflow Sequence ---")
    g_seq = TelemetryGraph("temperature")
    for t in range(100):
        g_seq.add_point(float(t), 20.0 + float(t) * 0.1)

    # 1. Reset
    g_seq.btn_reset.click()
    assert g_seq._follow is True

    # 2. +
    g_seq.btn_zoom_in.click()
    assert g_seq._follow is False

    # 3. +
    g_seq.btn_zoom_in.click()

    # 4. Pan Left
    g_seq.pan_by(-10.0, 0.0)

    # 5. Pan Right
    g_seq.pan_by(15.0, 0.0)

    # 6. Pan Up
    g_seq.pan_by(0.0, 5.0)

    # 7. Pan Down
    g_seq.pan_by(0.0, -3.0)

    # 8. -
    g_seq.btn_zoom_out.click()

    # 9. Pan Again
    g_seq.pan_by(5.0, -2.0)

    # 10. Reset
    g_seq.btn_reset.click()
    assert g_seq._follow is True
    assert g_seq._manual_y is False
    assert abs(g_seq._view_x_min - 0.0) < 1e-3
    assert abs(g_seq._view_x_max - 99.0) < 1e-3
    print("  [PASS] Complete sequence (Reset -> + -> + -> Pan -> - -> Pan -> Reset) executed seamlessly")

    print("\n==================================================")
    print("ALL GRAPH PANNING TESTS PASSED PERFECTLY!")
    print("==================================================")

if __name__ == "__main__":
    run_panning_suite()
