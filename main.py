#!/usr/bin/env python3
"""
main.py — Ground Station entry point
════════════════════════════════════
§2 — one entry point for both modes. The old split between main.py (live)
and main_sim.py (simulator) is gone: §2.1 puts the choice on a
mode-selection screen at startup, and §2.2 requires both modes to share
the same dashboard, rendering, parsing and reconciliation code.

Keeping two entry files was how the previous build ended up with a
simulator calling a method signature the simulator class did not have —
the two paths were only ever exercised separately. One path, chosen at
runtime, cannot drift like that.

Usage:
    python3 main.py                 # mode-selection screen (§2.1)
    python3 main.py --sim           # skip the screen, go straight to sim
    python3 main.py --port COM5     # skip the screen, connect directly

The two flags are a development convenience. They do not create a third
mode: they preselect one of the same two and take the same code path.
"""
from __future__ import annotations
import argparse
import shutil
import sys

from PyQt6.QtWidgets import QApplication, QMessageBox, QFileDialog

from ground_station.app import GroundStationApp
from ground_station.launcher import LaunchChoice, LaunchMode, LauncherDialog
from ground_station.link import LinkSupervisor, SERIAL_AVAILABLE
from ground_station.logs.raw_log import SessionLogs
from ground_station.logs.post_flight import PostFlightSource
from ground_station.simulator.source import SimulatedSource
from ground_station.theme import ThemeManager


def _choose(app: QApplication, args) -> LaunchChoice | None:
    if args.sim:
        return LaunchChoice(LaunchMode.SIMULATOR)
    if args.sd is not None or args.post_flight is not None:
        filepath = args.sd if args.sd is not None else args.post_flight
        if not filepath:
            filename, _ = QFileDialog.getOpenFileName(
                None, "Open Onboard SD Card Telemetry Log", "", "CSV Files (*.csv);;All Files (*)"
            )
            if not filename:
                return None
            filepath = filename
        return LaunchChoice(LaunchMode.POST_FLIGHT, filepath=filepath)
    if args.port:
        return LaunchChoice(LaunchMode.LIVE, stable_id=f"DEV:{args.port}",
                            device=args.port, baud=args.baud)

    dlg = LauncherDialog()
    if dlg.exec() != LauncherDialog.DialogCode.Accepted:
        return None
    return dlg.choice


def main() -> int:
    ap = argparse.ArgumentParser(description="PHOENIX dB.V1 Ground Station")
    ap.add_argument("--sim", action="store_true",
                    help="skip the mode screen and run the simulator")
    ap.add_argument("--sd", nargs="?", const="", default=None,
                    help="open an onboard SD-card recorded telemetry log (.csv)")
    ap.add_argument("--post-flight", dest="post_flight", nargs="?", const="", default=None,
                    help="open post-flight analysis mode (.csv)")
    ap.add_argument("--port", help="skip the mode screen and connect to a port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--log-dir", default="logs",
                    help="where session logs are written (§12)")
    args = ap.parse_args()

    app = QApplication(sys.argv)
    app.setApplicationName("Ground Station")
    app.setStyleSheet(ThemeManager.main_stylesheet())

    choice = _choose(app, args)
    if choice is None:
        return 0

    simulator = choice.mode is LaunchMode.SIMULATOR
    post_flight = choice.mode is LaunchMode.POST_FLIGHT

    if not simulator and not post_flight and not SERIAL_AVAILABLE:
        QMessageBox.critical(
            None, "pyserial not installed",
            "Live mode needs pyserial.\n\nInstall it with:\n"
            "    pip install pyserial")
        return 1

    # Mode label for title and sidebar
    if simulator:
        mode_label = "SIMULATOR MODE"
    elif post_flight:
        mode_label = "POST-FLIGHT SD CARD"
    else:
        mode_label = "LIVE"

    window = GroundStationApp(
        mode_label=mode_label,
        simulator_mode=simulator)

    logs = SessionLogs(args.log_dir)
    _check_disk_space(args.log_dir, window)

    if simulator:
        source = SimulatedSource()
    elif post_flight:
        source = PostFlightSource(choice.filepath or "")
    else:
        source = LinkSupervisor()

    _attach_logging(source, logs)
    window.connect_supervisor(source)

    if simulator:
        source.connect_to()
        source.fault_injected.connect(
            lambda kind, note: logs.raw.write_event(f"SIM:{kind}", note))
    elif post_flight:
        source.connect_to()
    else:
        source.connect_to(choice.stable_id, choice.baud, choice.device)

    window.statusBar().showMessage(f"Logging to {logs.directory}", 10000)
    window.show()
    window.activateWindow()
    window.raise_()

    code = app.exec()
    source.disconnect_source()
    logs.close()
    return code


def _attach_logging(source, logs: SessionLogs) -> None:
    """§12.1 — every received line, verbatim, before anything judges it.

    Wired to raw_line rather than to the per-vehicle packet signals on
    purpose: raw_line fires on receipt, ahead of the demultiplexer, so
    malformed lines reach the log even though they never become packets
    (§4.5). Wiring to the packet signals would produce a log containing
    only the lines that parsed, which cannot explain a flight where the
    parsing was the problem.
    """
    from ground_station.models import VehicleID

    def on_raw(line: str, received_at: float) -> None:
        vehicle = None
        # Cheap attribution for the log only. A wrong guess costs one
        # mis-filed line and never touches a displayed value.
        if ",ROCKET," in line or line.startswith("PKT="):
            vehicle = VehicleID.ROCKET
        elif ",CANSAT," in line:
            vehicle = VehicleID.CANSAT
        logs.raw.write_line(line, received_at, vehicle)

    def on_packet(packet, result) -> None:
        logs.sd.log_packet(packet)

    def on_rejected(rejected) -> None:
        logs.raw.write_rejected(rejected)
        logs.sd.log_rejected(rejected.raw, rejected.reason.value,
                             rejected.vehicle_id, rejected.received_at)

    source.raw_line.connect(on_raw)
    source.rocket_packet.connect(on_packet)
    source.cansat_packet.connect(on_packet)
    source.line_rejected.connect(on_rejected)
    source.notice.connect(lambda msg: logs.raw.write_event("NOTICE", msg))
    source.source_state_changed.connect(
        lambda st: logs.raw.write_event("SOURCE", st.value))
    source.receiver_state_changed.connect(
        lambda st: logs.raw.write_event("RECEIVER", st.value))
    source.vehicle_state_changed.connect(
        lambda vid, st: logs.raw.write_event("VEHICLE", f"{vid.value} {st.value}"))



def _check_disk_space(log_dir: str, window) -> None:
    """Warn the operator if the log partition is running low.

    §12.2 forbids rotation ("the log is the flight record"), so there is
    nothing to do except alert early.  Thresholds:
      < 500 MB  — advisory warning; operator can dismiss and continue.
      <  50 MB  — critical; likely to exhaust mid-flight.  Operator must
                   acknowledge before the session starts.
    """
    try:
        usage = shutil.disk_usage(log_dir)
        free_mb = usage.free / (1024 * 1024)
    except OSError:
        # Can't stat — not fatal; the sink's write() will catch errors later.
        return

    if free_mb < 50:
        from PyQt6.QtWidgets import QMessageBox
        QMessageBox.critical(
            window,
            "Critically low disk space",
            f"Only {free_mb:.0f} MB free in '{log_dir}'.\n\n"
            f"A 12-hour session writes ~170 MB. The disk will likely fill\n"
            f"mid-flight and logging will stop silently.\n\n"
            f"Free up space before starting, or change --log-dir.\n\n"
            f"(Rotation is not offered — §12.2: the log is the flight record.)"
        )
    elif free_mb < 500:
        from PyQt6.QtWidgets import QMessageBox
        QMessageBox.warning(
            window,
            "Low disk space",
            f"Only {free_mb:.0f} MB free in '{log_dir}'.\n\n"
            f"A 12-hour session writes ~170 MB. Consider freeing space\n"
            f"or changing --log-dir before a long flight."
        )


if __name__ == "__main__":
    sys.exit(main())
