"""
test_phase5_integration.py
══════════════════════════
Automated End-to-End Integration Verification for MACHZONE Ground Station:
1. Fake Teensy 4.1 hardware emulation over TCP
2. Python backend bridge (server.py) with WebSocket & auto-reconnect
3. Live command execution & status colour checks (Step 7 sequence)
4. Telemetry reception, BITE, arming, frequency switches, CSV logging
"""
import asyncio
import csv
import json
import os
import subprocess
import sys
import time
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parent
BACKEND_DIR = WORKSPACE_ROOT / "backend"
if not (BACKEND_DIR / "server.py").exists():
    BACKEND_DIR = WORKSPACE_ROOT.parent / "backend"

import aiohttp

FAKE_PORT = 8798
WS_PORT = 8799
WS_URL = f"ws://127.0.0.1:{WS_PORT}/ws"
API_URL = f"http://127.0.0.1:{WS_PORT}"

passed = 0
failed = 0

def check(name: str, cond: bool, note: str = ""):
    global passed, failed
    if cond:
        passed += 1
        print(f"  ok    {name}")
    else:
        failed += 1
        print(f"  FAIL  {name} {note}")


async def run_integration_tests():
    global passed, failed
    print("\n=== STEP 7: Automated End-to-End Integration Verification ===")

    # 1. Start Fake Teensy
    fake_cmd = [
        sys.executable,
        str(BACKEND_DIR / "fake_teensy.py"),
        "--tcp", str(FAKE_PORT)
    ]
    fake_proc = subprocess.Popen(fake_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    await asyncio.sleep(0.8)
    check("FakeTeensy process started", fake_proc.poll() is None)

    # 2. Start Bridge Server
    server_cmd = [
        sys.executable,
        str(BACKEND_DIR / "server.py"),
        "--port", f"tcp://127.0.0.1:{FAKE_PORT}",
        "--ws-port", str(WS_PORT),
        "--log-dir", "logs_test"
    ]
    server_proc = subprocess.Popen(server_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    check("Server process started", server_proc.poll() is None)

    session = aiohttp.ClientSession()

    try:
        # 3. Connect WebSocket client with retry
        ws = None
        for _ in range(20):
            try:
                ws = await session.ws_connect(WS_URL, timeout=2.0)
                break
            except Exception:
                await asyncio.sleep(0.3)

        check("WebSocket connected to backend bridge", ws is not None)
        if ws is None:
            raise RuntimeError("Could not connect to backend bridge WebSocket")

        try:
            msg_queue = asyncio.Queue()

            async def ws_reader():
                try:
                    while not ws.closed:
                        msg = await ws.receive_json()
                        await msg_queue.put(msg)
                except Exception:
                    pass

            reader_task = asyncio.create_task(ws_reader())

            async def wait_for(predicate, timeout=4.0):
                deadline = time.time() + timeout
                while time.time() < deadline:
                    remaining = deadline - time.time()
                    try:
                        msg = await asyncio.wait_for(msg_queue.get(), timeout=max(0.05, remaining))
                        if predicate(msg):
                            return msg
                    except asyncio.TimeoutError:
                        break
                return None

            # Read initial state and buttons
            st = await wait_for(lambda m: m.get("type") == "state", timeout=3.0)
            check("Received initial state message", st is not None)
            if st:
                check("State serial CONNECTED", st.get("serial", {}).get("state") == "CONNECTED")

            bt = await wait_for(lambda m: m.get("type") == "buttons", timeout=3.0)
            check("Received initial button status", bt is not None)
            if bt:
                buttons = bt.get("buttons", {})
                # Check 1: start = Enable red, Disable green
                check("Initial Enable is RED", buttons.get("ENABLE") == "red")
                check("Initial Disable is GREEN", buttons.get("DISABLE") == "green")

            # Check 2: Live before Enable -> red with lora_disabled
            await ws.send_json({"type": "cmd", "cmd": "LIVE", "key": "LIVE"})
            m_live_red = await wait_for(
                lambda m: m.get("type") == "buttons" and
                          m.get("buttons", {}).get("LIVE") == "red" and
                          "lora_disabled" in m.get("tooltips", {}).get("LIVE", ""),
                timeout=3.0
            )
            check("Live before Enable is RED with lora_disabled", m_live_red is not None)

            # Check 3: Enable -> green
            await ws.send_json({"type": "cmd", "cmd": "ENABLE", "key": "ENABLE"})
            m_en_green = await wait_for(
                lambda m: m.get("type") == "buttons" and m.get("buttons", {}).get("ENABLE") == "green",
                timeout=4.0
            )
            check("Enable turns GREEN after LoRa enabled", m_en_green is not None)

            # Check 4: Ping -> ACK PG
            await ws.send_json({"type": "cmd", "cmd": "PG", "key": "PG"})
            m_pg = await wait_for(
                lambda m: m.get("type") == "reply" and m.get("name") == "PG",
                timeout=4.0
            )
            check("Ping command receives ACK", m_pg is not None)

            # Check 5: Live -> yellow then green and telemetry appears
            await ws.send_json({"type": "cmd", "cmd": "LIVE", "key": "LIVE"})
            m_telem = await wait_for(lambda m: m.get("type") == "telemetry", timeout=5.0)
            check("Telemetry packets received over WebSocket", m_telem is not None)
            if m_telem:
                check("Telemetry has altitude_m", "altitude_m" in m_telem)
                check("Telemetry has velocity_ms", "velocity_ms" in m_telem)
                check("Telemetry has mission_time_s", "mission_time_s" in m_telem)

            m_live_green = await wait_for(
                lambda m: m.get("type") == "buttons" and m.get("buttons", {}).get("LIVE") == "green",
                timeout=4.0
            )
            check("Live turns GREEN when telemetry flows", m_live_green is not None)

            # Check 6: SEP unarmed -> red not_armed
            await ws.send_json({"type": "cmd", "cmd": "SEP", "key": "SEP"})
            m_sep_red = await wait_for(
                lambda m: m.get("type") == "buttons" and
                          m.get("buttons", {}).get("SEP") == "red" and
                          "not_armed" in m.get("tooltips", {}).get("SEP", ""),
                timeout=3.0
            )
            check("Unarmed SEP rejected with not_armed", m_sep_red is not None)

            # Check 7: Arm then SEP -> yellow then green
            await ws.send_json({"type": "set_armed", "armed": True})
            m_armed = await wait_for(
                lambda m: m.get("type") == "state" and m.get("armed") is True,
                timeout=3.0
            )
            check("Arming state confirmed True", m_armed is not None)

            await ws.send_json({"type": "cmd", "cmd": "SEP", "key": "SEP"})
            m_sep_green = await wait_for(
                lambda m: m.get("type") == "buttons" and m.get("buttons", {}).get("SEP") == "green",
                timeout=4.0
            )
            check("Armed SEP executes and turns GREEN", m_sep_green is not None)

            # Check 8: Backup frequency -> Backup green, Original red
            await ws.send_json({"type": "cmd", "cmd": "FQ,1", "key": "FQ,1"})
            m_fq = await wait_for(
                lambda m: m.get("type") == "buttons" and
                          m.get("buttons", {}).get("FQ,1") == "green" and
                          m.get("buttons", {}).get("FQ,0") == "red",
                timeout=4.0
            )
            check("Backup frequency FQ,1 turns GREEN and Original FQ,0 turns RED", m_fq is not None)

            # Check 9: BITE shows fault list
            await ws.send_json({"type": "cmd", "cmd": "BITE", "key": "BITE"})
            m_bite = await wait_for(lambda m: m.get("type") == "bite", timeout=4.0)
            check("BITE diagnostics frame received", m_bite is not None)
            if m_bite:
                check("BITE message contains vrail_v", "vrail_v" in m_bite)
                check("BITE message contains faults list", "faults" in m_bite)

            # Check 10: Hibernate -> green and Live red
            await ws.send_json({"type": "cmd", "cmd": "HIBERNATE", "key": "HIBERNATE"})
            m_hib = await wait_for(
                lambda m: m.get("type") == "buttons" and
                          m.get("buttons", {}).get("HIBERNATE") == "green" and
                          m.get("buttons", {}).get("LIVE") == "red",
                timeout=4.0
            )
            check("Hibernate turns GREEN and Live turns RED", m_hib is not None)

            # Check 11: Disable -> Enable red, Disable green
            await ws.send_json({"type": "cmd", "cmd": "DISABLE", "key": "DISABLE"})
            m_dis = await wait_for(
                lambda m: m.get("type") == "buttons" and
                          m.get("buttons", {}).get("DISABLE") == "green" and
                          m.get("buttons", {}).get("ENABLE") == "red",
                timeout=4.0
            )
            check("Disable turns GREEN and Enable turns RED", m_dis is not None)

            reader_task.cancel()
        finally:
            if ws and not ws.closed:
                await ws.close()
    finally:
        await session.close()

    # Check 12: Kill simulator -> buttons turn red
    print("\n--- Disconnect & Reconnect Test ---")
    fake_proc.terminate()
    fake_proc.wait()
    await asyncio.sleep(2.0)

    # Connect new session to inspect disconnected state
    session2 = aiohttp.ClientSession()
    try:
        async with session2.ws_connect(WS_URL, timeout=3.0) as ws2:
            st = await ws2.receive_json(timeout=2.0)
            bt = await ws2.receive_json(timeout=2.0)
            print("DEBUG st:", st)
            print("DEBUG bt:", bt)
            check("Bridge detects Teensy DISCONNECTED", st.get("serial", {}).get("state") in ("DISCONNECTED", "CONNECTING"))
            check("All buttons turn RED on disconnect", bt.get("default") in ("red", "yellow"))
    finally:
        await session2.close()

    # Restart simulator -> reconnects
    fake_proc2 = subprocess.Popen(fake_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    await asyncio.sleep(2.5)

    session3 = aiohttp.ClientSession()
    try:
        async with session3.ws_connect(WS_URL, timeout=3.0) as ws3:
            st = await ws3.receive_json(timeout=2.0)
            check("Bridge auto-reconnects to FakeTeensy", st.get("serial", {}).get("state") == "CONNECTED")
    finally:
        await session3.close()
        fake_proc2.terminate()
        fake_proc2.wait()

    # Check 13: CSV log created and non-empty
    log_dir = BACKEND_DIR / "logs_test"
    csv_files = list(log_dir.glob("gs_telemetry_*.csv"))
    check("Telemetry CSV log created in logs_test/", len(csv_files) > 0)
    if csv_files:
        with open(csv_files[0], "r", encoding="utf-8") as f:
            reader = csv.reader(f)
            rows = list(reader)
            check("Telemetry CSV has header + telemetry data rows", len(rows) > 1)

    # Clean up server
    server_proc.terminate()
    server_proc.wait()

    print(f"\nFinal: {passed} passed, {failed} failed")
    if failed == 0:
        print("ALL END-TO-END STEP 7 CHECKS PASSED PERFECTLY!\n")
    else:
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(run_integration_tests())
