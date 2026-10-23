"""
backend/server.py
═════════════════
Teensy 4.1 Ground Station Bridge Server (Team 2026 IN-SPACe-010).

Connects to Teensy 4.1 over USB Serial (VID 0x16C0), handles auto-reconnect,
tracks command completion (<id>,<CMD>), computes button colours,
logs telemetry and events, and exposes WebSocket (ws://localhost:8765/ws)
and REST endpoints (/api/state, /api/ports, /api/cmd).
"""
from __future__ import annotations

import argparse
import asyncio
import csv
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

from aiohttp import web

try:
    import serial
    from serial.tools import list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    serial = None
    list_ports = None
    SERIAL_AVAILABLE = False

from protocol import decode_line, parse_telemetry, STATE_NAMES

ARM_TIMEOUT_S = 120.0
BUTTON_OK_HOLD_S = 2.0
BUTTON_FAILED_HOLD_S = 4.0
TELEM_TIMEOUT_S = 2.5
CMD_TIMEOUT_S = 18.0

ARM_REQUIRED_COMMANDS = {
    "UNLOCK", "SV", "PW", "TEST1", "TEST2", "TEST3",
    "NSTEP", "FSEQ", "SEP", "FPL"
}


class GroundStationBridge:
    def __init__(self, port: Optional[str] = None, baud: int = 115200, log_dir: str = "logs"):
        self.requested_port = port
        self.baud = baud
        self.log_dir = Path(__file__).resolve().parent / log_dir
        self.log_dir.mkdir(parents=True, exist_ok=True)

        self.serial_state = "DISCONNECTED"  # CONNECTED, CONNECTING, DISCONNECTED
        self.current_port: Optional[str] = None
        self.serial_info = "Not connected"
        self.serial_reader: Optional[asyncio.StreamReader] = None
        self.serial_writer: Optional[asyncio.StreamWriter] = None
        self.serial_conn: Any = None  # serial.Serial if sync thread

        # Subscriptions
        self.ws_clients: Set[web.WebSocketResponse] = set()

        # Operational state
        self.armed = False
        self.armed_time: float = 0.0
        self.target = "ROCKET"  # ROCKET or CANSAT

        # Radio & Ground state
        self.lora_enabled = False
        self.radio_ok = False
        self.channel = 0  # 0: Main (866.50MHz), 1: Backup (865.25MHz)
        self.scanning = False

        # Telemetry tracking
        self.last_telem_time: float = 0.0
        self.latest_telemetry: Optional[Dict[str, Any]] = None

        # Command tracking
        self.next_cmd_id = 1
        self.active_commands: Dict[int, Dict[str, Any]] = {}
        # Key -> state: {"status": "ok"|"yellow"|"failed"|"ready", "time": float, "reason": str}
        self.button_states: Dict[str, Dict[str, Any]] = {}
        self.channel_switch_in_progress = False

        # Logs
        session_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.telem_log_path = self.log_dir / f"gs_telemetry_{session_ts}.csv"
        self.event_log_path = self.log_dir / f"gs_events_{session_ts}.csv"

        self._init_logs()

    def _init_logs(self):
        if not self.telem_log_path.exists():
            with open(self.telem_log_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "timestamp", "mission_time_s", "packet_count", "altitude_m", "pressure_pa",
                    "temperature_c", "voltage_v", "gnss_time", "latitude", "longitude",
                    "gnss_altitude_m", "satellites", "acceleration_ms2", "spin_rate_dps",
                    "state", "state_name", "velocity_ms", "rssi", "snr"
                ])
        if not self.event_log_path.exists():
            with open(self.event_log_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["timestamp", "type", "raw"])

    def log_event(self, event_type: str, raw: str):
        try:
            with open(self.event_log_path, "a", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow([datetime.now().isoformat(), event_type, raw])
        except Exception as e:
            print(f"[ERR] log_event: {e}", file=sys.stderr)

    def log_telemetry(self, t: Dict[str, Any]):
        try:
            with open(self.telem_log_path, "a", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow([
                    datetime.now().isoformat(),
                    t.get("mission_time_s", 0),
                    t.get("packet_count", 0),
                    t.get("altitude_m", 0.0),
                    t.get("pressure_pa", 0),
                    t.get("temperature_c", 0.0),
                    t.get("voltage_v", 0.0),
                    t.get("gnss_s_of_hour", 0),
                    t.get("latitude_deg", 0.0),
                    t.get("longitude_deg", 0.0),
                    t.get("gnss_altitude_m", 0.0),
                    t.get("satellites", 0),
                    t.get("acceleration_ms2", 0.0),
                    t.get("spin_rate_dps", 0.0),
                    t.get("state", 0),
                    t.get("state_name", "UNKNOWN"),
                    t.get("velocity_ms", 0.0),
                    t.get("rssi", 0.0),
                    t.get("snr", 0.0),
                ])
        except Exception as e:
            print(f"[ERR] log_telemetry: {e}", file=sys.stderr)

    # ── Arming & Target ──────────────────────────────────────────────────

    def set_armed(self, armed: bool):
        self.armed = armed
        self.armed_time = time.time() if armed else 0.0
        asyncio.create_task(self.broadcast_state())
        asyncio.create_task(self.broadcast_buttons())

    def set_target(self, target: str):
        self.target = target.upper()
        asyncio.create_task(self.broadcast_state())
        asyncio.create_task(self.broadcast_buttons())

    def check_auto_disarm(self):
        if self.armed and (time.time() - self.armed_time > ARM_TIMEOUT_S):
            print("[INFO] Armed timeout reached (120 s) -> Auto-disarmed.")
            self.set_armed(False)

    # ── State and Button Colours ─────────────────────────────────────────

    def get_state_dict(self) -> Dict[str, Any]:
        return {
            "type": "state",
            "serial": {
                "state": self.serial_state,
                "port": self.current_port or self.requested_port or "auto",
                "info": self.serial_info,
            },
            "lora_enabled": self.lora_enabled,
            "radio_ok": self.radio_ok,
            "channel": self.channel,
            "armed": self.armed,
            "target": self.target,
            "telemetry_flowing": (time.time() - self.last_telem_time < TELEM_TIMEOUT_S) if self.last_telem_time else False,
            "last_telem_s_ago": round(time.time() - self.last_telem_time, 1) if self.last_telem_time else 999.0,
        }

    def compute_button_colors(self) -> Dict[str, Any]:
        now = time.time()
        buttons: Dict[str, str] = {}
        tooltips: Dict[str, str] = {}

        if self.serial_state == "DISCONNECTED":
            return {
                "type": "buttons",
                "default": "red",
                "buttons": {},
                "tooltips": {"default": "Backend offline / Teensy not connected"}
            }
        elif self.serial_state == "CONNECTING":
            return {
                "type": "buttons",
                "default": "yellow",
                "buttons": {},
                "tooltips": {"default": "Teensy connecting (handshake)..."}
            }

        # 1. LIVE button
        telem_flowing = (now - self.last_telem_time < TELEM_TIMEOUT_S) if self.last_telem_time else False
        live_rec = self.button_states.get("LIVE", {})
        if telem_flowing:
            buttons["LIVE"] = "green"
            tooltips["LIVE"] = "Telemetry flowing (< 2.5 s old)"
        elif live_rec.get("status") == "yellow" or self.scanning:
            buttons["LIVE"] = "yellow"
            tooltips["LIVE"] = "Live command sent / waiting for telemetry"
        else:
            buttons["LIVE"] = "red"
            tooltips["LIVE"] = live_rec.get("reason", "LoRa off or no telemetry")

        # 2. HIBERNATE button
        hib_rec = self.button_states.get("HIBERNATE", {})
        if hib_rec.get("status") == "yellow":
            buttons["HIBERNATE"] = "yellow"
            tooltips["HIBERNATE"] = "Hibernate command sent..."
        elif hib_rec.get("status") == "ok" and not telem_flowing:
            buttons["HIBERNATE"] = "green"
            tooltips["HIBERNATE"] = "Telemetry hibernated"
        else:
            buttons["HIBERNATE"] = "red"
            tooltips["HIBERNATE"] = hib_rec.get("reason", "Not hibernated")

        # 3. ENABLE button
        en_rec = self.button_states.get("ENABLE", {})
        if en_rec.get("status") == "yellow":
            buttons["ENABLE"] = "yellow"
            tooltips["ENABLE"] = "Radio initializing..."
        elif self.lora_enabled and self.radio_ok:
            buttons["ENABLE"] = "green"
            tooltips["ENABLE"] = "Ground LoRa enabled & radio OK"
        else:
            buttons["ENABLE"] = "red"
            tooltips["ENABLE"] = en_rec.get("reason", "LoRa disabled or radio error")

        # 4. DISABLE button
        dis_rec = self.button_states.get("DISABLE", {})
        if dis_rec.get("status") == "yellow":
            buttons["DISABLE"] = "yellow"
            tooltips["DISABLE"] = "Disabling radio..."
        elif not self.lora_enabled:
            buttons["DISABLE"] = "green"
            tooltips["DISABLE"] = "Ground LoRa disabled (confirmed)"
        else:
            buttons["DISABLE"] = "red"
            tooltips["DISABLE"] = "LoRa currently enabled"

        # 5. Rocket Original (FQ,0) / Backup (FQ,1)
        if self.channel_switch_in_progress:
            buttons["FQ,0"] = "yellow"
            buttons["FQ,1"] = "yellow"
            tooltips["FQ,0"] = "Channel switch in progress..."
            tooltips["FQ,1"] = "Channel switch in progress..."
        else:
            if self.channel == 0:
                buttons["FQ,0"] = "green"
                buttons["FQ,1"] = "red"
                tooltips["FQ,0"] = "Active channel: 866.50 MHz (Main)"
                tooltips["FQ,1"] = "Inactive channel (Backup)"
            else:
                buttons["FQ,0"] = "red"
                buttons["FQ,1"] = "green"
                tooltips["FQ,0"] = "Inactive channel (Main)"
                tooltips["FQ,1"] = "Active channel: 865.25 MHz (Backup)"

        # 6. CanSat Original / Backup (no uplink in firmware)
        buttons["CANSAT_ORIGINAL"] = "red"
        buttons["CANSAT_BACKUP"] = "red"
        tooltips["CANSAT_ORIGINAL"] = "CanSat link not implemented"
        tooltips["CANSAT_BACKUP"] = "CanSat link not implemented"

        # 7. Action / momentary command buttons (neutral when idle, yellow when pending, green when ok, red when failed)
        for key, rec in self.button_states.items():
            if key in ("LIVE", "HIBERNATE", "ENABLE", "DISABLE", "FQ,0", "FQ,1", "CANSAT_ORIGINAL", "CANSAT_BACKUP"):
                continue
            st = rec.get("status")
            t_event = rec.get("time", 0.0)
            if st == "yellow":
                buttons[key] = "yellow"
                tooltips[key] = rec.get("reason", "Waiting for rocket...")
            elif st == "ok" and (now - t_event < BUTTON_OK_HOLD_S):
                buttons[key] = "green"
                tooltips[key] = rec.get("reason", "Command OK")
            elif st == "failed" and (now - t_event < BUTTON_FAILED_HOLD_S):
                buttons[key] = "red"
                tooltips[key] = rec.get("reason", "Command failed")
            else:
                buttons[key] = "neutral"
                tooltips[key] = "Ready (click to send)" if self.lora_enabled else "LoRa disabled (send ENABLE)"

        default_color = "neutral"

        return {
            "type": "buttons",
            "default": default_color,
            "buttons": buttons,
            "tooltips": tooltips,
        }

    async def broadcast(self, msg: Dict[str, Any]):
        if not self.ws_clients:
            return
        payload = json.dumps(msg)
        dead = set()
        for ws in self.ws_clients:
            try:
                await ws.send_str(payload)
            except Exception:
                dead.add(ws)
        self.ws_clients.difference_update(dead)

    async def broadcast_state(self):
        await self.broadcast(self.get_state_dict())

    async def broadcast_buttons(self):
        await self.broadcast(self.compute_button_colors())

    # ── Command Pipeline ─────────────────────────────────────────────────

    def send_raw_line(self, line: str):
        clean = line.strip() + "\n"
        if self.serial_writer:
            try:
                self.serial_writer.write(clean.encode("utf-8"))
                asyncio.create_task(self.serial_writer.drain())
            except Exception as e:
                print(f"[WARN] write failed: {e}", file=sys.stderr)
        elif self.serial_conn:
            try:
                self.serial_conn.write(clean.encode("utf-8"))
            except Exception as e:
                print(f"[WARN] serial write failed: {e}", file=sys.stderr)
        else:
            print(f"[WARN] No serial connection to send: {clean.strip()}", file=sys.stderr)

    async def read_stream(self, reader: asyncio.StreamReader):
        while self.serial_state == "CONNECTED" and not reader.at_eof():
            try:
                line_bytes = await reader.readline()
                if not line_bytes:
                    break
                line = line_bytes.decode("utf-8", errors="replace").strip()
                if line:
                    await self.handle_incoming_line(line)
            except Exception as e:
                print(f"[ERR] read_stream: {e}", file=sys.stderr)
                break
        self.serial_state = "DISCONNECTED"
        self.serial_info = "Connection closed"
        self.serial_reader = None
        self.serial_writer = None
        await self.broadcast_state()
        await self.broadcast_buttons()

    def send_command(self, cmd: str, key: Optional[str] = None, seq: Optional[int] = None) -> Tuple[bool, str, int]:
        """Validate, assign ID, frame command, and send to Teensy."""
        key = key or cmd
        now = time.time()

        # Check serial connection
        if self.serial_state != "CONNECTED":
            reason = "FAILED: teensy_disconnected"
            self.button_states[key] = {"status": "failed", "time": now, "reason": reason}
            asyncio.create_task(self.broadcast_buttons())
            return False, reason, 0

        # Check vehicle target
        if self.target == "CANSAT" and not cmd.startswith("GF"):
            reason = "REJECTED: cansat_uplink_not_implemented"
            self.button_states[key] = {"status": "failed", "time": now, "reason": reason}
            asyncio.create_task(self.broadcast_buttons())
            return False, reason, 0

        # Check Arm requirement
        cmd_root = cmd.split(",")[0].upper()
        if cmd_root in ARM_REQUIRED_COMMANDS and not self.armed:
            reason = "REJECTED: not_armed"
            self.button_states[key] = {"status": "failed", "time": now, "reason": reason}
            asyncio.create_task(self.broadcast_buttons())
            return False, reason, 0

        # Check LoRa requirement
        local_cmds = {"ENABLE", "LINK", "DISABLE", "GS", "GF"}
        if cmd_root not in local_cmds and not self.lora_enabled:
            reason = "REJECTED: lora_disabled"
            self.button_states[key] = {"status": "failed", "time": now, "reason": reason}
            asyncio.create_task(self.broadcast_buttons())
            return False, reason, 0

        cmd_id = int(seq) if seq is not None and int(seq) > 0 else self.next_cmd_id
        if cmd_id >= self.next_cmd_id:
            self.next_cmd_id = cmd_id + 1

        wire_line = f"{cmd_id},{cmd}"
        self.active_commands[cmd_id] = {
            "id": cmd_id,
            "cmd": cmd,
            "key": key,
            "time": now,
            "tries": 0,
            "status": "QUEUED",
        }

        self.button_states[key] = {
            "status": "yellow",
            "time": now,
            "reason": f"Queued ({wire_line})",
        }

        if cmd.startswith("FQ,"):
            self.channel_switch_in_progress = True

        self.send_raw_line(wire_line)
        self.log_event("cmd_tx", wire_line)
        asyncio.create_task(self.broadcast_buttons())
        return True, "queued", cmd_id

    # ── Incoming Line Processing ─────────────────────────────────────────

    async def handle_incoming_line(self, line: str):
        if not line:
            return
        now = time.time()
        event = decode_line(line)
        self.log_event(event.get("type", "raw"), line)

        # Broadcast line and event
        event["raw"] = line
        await self.broadcast(event)

        etype = event.get("type")

        # 1. Telemetry
        if etype == "telemetry":
            self.last_telem_time = now
            self.latest_telemetry = event
            self.log_telemetry(event)
            # If Live was waiting, mark OK
            if "LIVE" in self.button_states:
                self.button_states["LIVE"] = {"status": "ok", "time": now, "reason": "Flowing"}
            await self.broadcast_buttons()

        # 2. Command progress ($G,CMD,<id>,<status>[,<info>])
        elif etype == "cmd":
            cmd_id = event.get("id", 0)
            status = event.get("status", "")
            info = event.get("info", "")

            rec = self.active_commands.get(cmd_id)
            if rec:
                key = rec["key"]
                if status == "QUEUED":
                    rec["status"] = "QUEUED"
                    self.button_states[key] = {"status": "yellow", "time": now, "reason": "QUEUED"}
                elif status == "SENT":
                    rec["status"] = "SENT"
                    rec["tries"] = int(info) if info.isdigit() else 1
                    self.button_states[key] = {"status": "yellow", "time": now, "reason": f"SENT try {info}"}
                elif status == "OK":
                    rec["status"] = "OK"
                    self.button_states[key] = {"status": "ok", "time": now, "reason": "OK"}
                    self.active_commands.pop(cmd_id, None)
                    if key.startswith("FQ,"):
                        self.channel_switch_in_progress = False
                elif status in ("REJECTED", "FAILED", "BADCMD", "BUSY"):
                    rec["status"] = status
                    reason = f"{status}: {info}" if info else status
                    self.button_states[key] = {"status": "failed", "time": now, "reason": reason}
                    self.active_commands.pop(cmd_id, None)
                    if key.startswith("FQ,"):
                        self.channel_switch_in_progress = False

            await self.broadcast_buttons()

        # 3. LoRa state ($G,LORA,ENABLED|DISABLED)
        elif etype == "lora":
            self.lora_enabled = event.get("enabled", False)
            if self.lora_enabled:
                self.button_states["ENABLE"] = {"status": "ok", "time": now, "reason": "Enabled"}
                self.button_states["DISABLE"] = {"status": "ready", "time": now, "reason": ""}
            else:
                self.button_states["DISABLE"] = {"status": "ok", "time": now, "reason": "Disabled"}
                self.button_states["ENABLE"] = {"status": "failed", "time": now, "reason": "LoRa off"}
                self.last_telem_time = 0.0
            await self.broadcast_state()
            await self.broadcast_buttons()

        # 4. Frequency status ($G,FREQ,<ch>,<MHz>,<what>)
        elif etype == "freq":
            self.channel = event.get("channel", 0)
            what = event.get("what", "")
            if what in ("CONFIRMED", "MANUAL", "FOUND"):
                self.channel_switch_in_progress = False
                self.button_states[f"FQ,{self.channel}"] = {"status": "ok", "time": now, "reason": what}
            elif what == "FAILED":
                self.channel_switch_in_progress = False
                self.button_states[f"FQ,{self.channel}"] = {"status": "failed", "time": now, "reason": "Switch failed"}
            await self.broadcast_state()
            await self.broadcast_buttons()

        # 5. Status ($G,STATUS,...)
        elif etype == "gs_status":
            self.lora_enabled = event.get("lora_enabled", False)
            self.radio_ok = event.get("radio_ok", False)
            self.channel = event.get("channel", 0)
            await self.broadcast_state()
            await self.broadcast_buttons()

        # 6. Replies ($H,ACK,... or $H,NACK,...)
        elif etype == "reply":
            cmd_name = event.get("name", "")
            ack = event.get("ack", False)
            detail = event.get("detail", {})

            # Match with command key
            key = cmd_name
            if cmd_name == "SD":
                key = "LIVE"
            elif cmd_name == "XD":
                key = "HIBERNATE"

            if ack:
                if key == "HIBERNATE":
                    self.last_telem_time = 0.0
                self.button_states[key] = {"status": "ok", "time": now, "reason": "Acknowledged"}
            else:
                reason = detail.get("reason", "NACK")
                self.button_states[key] = {"status": "failed", "time": now, "reason": f"NACK: {reason}"}

            await self.broadcast_buttons()

    # ── Background Loops ─────────────────────────────────────────────────

    async def connection_loop(self):
        """Continuously attempts to connect or reconnect to Teensy 4.1."""
        while True:
            self.check_auto_disarm()
            if self.serial_state != "CONNECTED":
                await self.try_connect()
            await asyncio.sleep(1.0)

    async def try_connect(self):
        # 1. TCP socket support for simulated testing (e.g. tcp://localhost:8766)
        if self.requested_port and self.requested_port.startswith("tcp://"):
            try:
                parts = self.requested_port[6:].split(":")
                host = parts[0]
                port = int(parts[1]) if len(parts) > 1 else 8766
                self.serial_state = "CONNECTING"
                self.current_port = self.requested_port
                self.serial_info = f"TCP socket {host}:{port}"
                await self.broadcast_state()
                await self.broadcast_buttons()

                reader, writer = await asyncio.open_connection(host, port)
                self.serial_reader = reader
                self.serial_writer = writer

                # Handshake: query ground status
                self.send_raw_line("GS")
                verified = False
                try:
                    line_bytes = await asyncio.wait_for(reader.readline(), timeout=3.0)
                    if line_bytes:
                        line = line_bytes.decode("utf-8", errors="replace").strip()
                        if line:
                            await self.handle_incoming_line(line)
                            if line.startswith(("$G,", "$T,", "$H,", "$B,")):
                                verified = True
                except asyncio.TimeoutError:
                    pass

                if not verified:
                    try:
                        writer.close()
                        await writer.wait_closed()
                    except Exception:
                        pass
                    raise RuntimeError("Device did not respond to handshake within 3s")

                self.serial_state = "CONNECTED"
                self.serial_info = f"Connected to FakeTeensy {host}:{port}"
                print(f"[INFO] Connected to {self.requested_port}")
                await self.broadcast_state()
                await self.broadcast_buttons()

                await self.read_stream(reader)
                return
            except Exception as e:
                self.serial_state = "DISCONNECTED"
                self.serial_info = f"Socket connection failed: {e}"
                if self.serial_writer:
                    try:
                        self.serial_writer.close()
                    except Exception:
                        pass
                self.serial_reader = None
                self.serial_writer = None
                await self.broadcast_state()
                await self.broadcast_buttons()
                return

        if not SERIAL_AVAILABLE:
            self.serial_state = "DISCONNECTED"
            self.serial_info = "pyserial not available"
            return

        # Find target port
        port_to_open = self.requested_port
        if not port_to_open:
            # Auto-detect Teensy (USB VID 0x16C0)
            for p in list_ports.comports():
                if getattr(p, "vid", None) == 0x16C0:
                    port_to_open = p.device
                    break

        if not port_to_open:
            self.serial_state = "DISCONNECTED"
            self.serial_info = "Scanning for Teensy 4.1 (VID 0x16C0)..."
            return

        try:
            self.serial_state = "CONNECTING"
            self.current_port = port_to_open
            self.serial_info = f"Connecting to {port_to_open}..."
            await self.broadcast_state()
            await self.broadcast_buttons()

            loop = asyncio.get_running_loop()
            ser = await loop.run_in_executor(
                None, lambda: serial.Serial(port_to_open, self.baud, timeout=1.0)
            )
            self.serial_conn = ser
            self.serial_info = f"Port {port_to_open} open, waiting for firmware handshake..."

            # Handshake: query ground status
            self.send_raw_line("GS")
            verified = False
            handshake_deadline = time.time() + 3.0
            while time.time() < handshake_deadline:
                line_bytes = await loop.run_in_executor(None, ser.readline)
                if line_bytes:
                    line = line_bytes.decode("utf-8", errors="replace").strip()
                    if line:
                        await self.handle_incoming_line(line)
                        if line.startswith(("$G,", "$T,", "$H,", "$B,")):
                            verified = True
                            break
                else:
                    await asyncio.sleep(0.02)

            if not verified:
                try:
                    ser.close()
                except Exception:
                    pass
                self.serial_conn = None
                raise RuntimeError(f"Device on {port_to_open} did not respond to handshake within 3s")

            self.serial_state = "CONNECTED"
            self.serial_info = f"Connected & verified: {port_to_open} @ {self.baud}"
            print(f"[INFO] Serial port opened and verified: {port_to_open}")
            await self.broadcast_state()
            await self.broadcast_buttons()

            # Reading loop
            while self.serial_state == "CONNECTED":
                line_bytes = await loop.run_in_executor(None, ser.readline)
                if line_bytes:
                    line = line_bytes.decode("utf-8", errors="replace").strip()
                    if line:
                        await self.handle_incoming_line(line)
                else:
                    await asyncio.sleep(0.01)

        except Exception as e:
            print(f"[WARN] Serial error on {port_to_open}: {e}")
            self.serial_state = "DISCONNECTED"
            self.serial_info = str(e)
            if self.serial_conn:
                try:
                    self.serial_conn.close()
                except Exception:
                    pass
                self.serial_conn = None
            await self.broadcast_state()
            await self.broadcast_buttons()

    async def read_stream(self, reader: asyncio.StreamReader):
        try:
            while not reader.at_eof():
                line_bytes = await reader.readline()
                if not line_bytes:
                    break
                line = line_bytes.decode("utf-8", errors="replace").strip()
                if line:
                    await self.handle_incoming_line(line)
        except Exception as e:
            print(f"[WARN] Stream read error: {e}")
        finally:
            self.serial_state = "DISCONNECTED"
            self.serial_info = "Stream disconnected"
            self.serial_reader = None
            self.serial_writer = None
            await self.broadcast_state()
            await self.broadcast_buttons()

    async def heartbeat_loop(self):
        """Periodic tick to update timers, check timeouts, and refresh UI button colors."""
        while True:
            await asyncio.sleep(0.5)
            # Timeout active commands
            now = time.time()
            to_fail = []
            for cid, rec in self.active_commands.items():
                if now - rec["time"] > CMD_TIMEOUT_S:
                    to_fail.append(cid)

            for cid in to_fail:
                rec = self.active_commands.pop(cid)
                key = rec["key"]
                self.button_states[key] = {
                    "status": "failed",
                    "time": now,
                    "reason": "FAILED: no_reply (timeout)"
                }
                if key.startswith("FQ,"):
                    self.channel_switch_in_progress = False

            await self.broadcast_buttons()


# ── REST & WebSocket Handlers ────────────────────────────────────────────

def create_app(bridge: GroundStationBridge) -> web.Application:
    @web.middleware
    async def cors_middleware(request: web.Request, handler):
        if request.method == "OPTIONS":
            response = web.Response(status=200)
        else:
            response = await handler(request)
        response.headers["Access-Control-Allow-Origin"] = "*"
        response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
        response.headers["Access-Control-Allow-Headers"] = "*"
        return response

    app = web.Application(middlewares=[cors_middleware])

    async def ws_handler(request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        bridge.ws_clients.add(ws)

        # Send initial state and button status
        await ws.send_str(json.dumps(bridge.get_state_dict()))
        await ws.send_str(json.dumps(bridge.compute_button_colors()))
        if bridge.latest_telemetry:
            await ws.send_str(json.dumps(bridge.latest_telemetry))

        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                        mtype = data.get("type")
                        if mtype == "cmd":
                            cmd = data.get("cmd", "")
                            key = data.get("key", cmd)
                            seq = data.get("seq")
                            bridge.send_command(cmd, key, seq=seq)
                        elif mtype == "set_armed":
                            bridge.set_armed(bool(data.get("armed", False)))
                        elif mtype == "set_target":
                            bridge.set_target(str(data.get("target", "ROCKET")))
                        elif mtype == "select_port":
                            bridge.requested_port = data.get("port")
                            bridge.serial_state = "DISCONNECTED"
                        elif mtype == "send_raw":
                            bridge.send_raw_line(data.get("text", ""))
                    except Exception as e:
                        print(f"[ERR] WS parse error: {e}", file=sys.stderr)
        finally:
            bridge.ws_clients.discard(ws)
        return ws

    async def api_state(request: web.Request) -> web.Response:
        return web.json_response(bridge.get_state_dict())

    async def api_ports(request: web.Request) -> web.Response:
        ports = []
        if SERIAL_AVAILABLE:
            for p in list_ports.comports():
                ports.append({
                    "device": p.device,
                    "description": p.description or "",
                    "vid": getattr(p, "vid", None),
                    "pid": getattr(p, "pid", None),
                })
        return web.json_response({"ports": ports})

    async def api_cmd(request: web.Request) -> web.Response:
        try:
            data = await request.json()
            cmd = data.get("cmd", "")
            key = data.get("key", cmd)
            seq = data.get("seq")
            ok, status, cid = bridge.send_command(cmd, key, seq=seq)
            return web.json_response({"ok": ok, "status": status, "id": cid})
        except Exception as e:
            return web.json_response({"ok": False, "error": str(e)}, status=400)

    # Routes
    app.router.add_get("/ws", ws_handler)
    app.router.add_get("/api/state", api_state)
    app.router.add_get("/api/ports", api_ports)
    app.router.add_post("/api/cmd", api_cmd)

    return app


async def main():
    parser = argparse.ArgumentParser(description="MACHZONE Ground Station Teensy 4.1 Bridge Server")
    parser.add_argument("--port", help="Serial port (COM7, /dev/ttyACM0, or tcp://localhost:8766)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    parser.add_argument("--host", default="0.0.0.0", help="HTTP/WS bind host")
    parser.add_argument("--ws-port", type=int, default=8765, help="HTTP/WS bind port (default 8765)")
    parser.add_argument("--log-dir", default="logs", help="Directory for logs (default logs)")
    args = parser.parse_args()

    bridge = GroundStationBridge(port=args.port, baud=args.baud, log_dir=args.log_dir)
    app = create_app(bridge)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, args.host, args.ws_port)
    await site.start()

    print(f"MACHZONE Ground Station Bridge running on http://{args.host}:{args.ws_port}")
    print(f"WebSocket available at ws://localhost:{args.ws_port}/ws")

    # Start connection and heartbeat tasks
    asyncio.create_task(bridge.connection_loop())
    asyncio.create_task(bridge.heartbeat_loop())

    # Keep running
    while True:
        await asyncio.sleep(3600)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nGround Station Bridge terminated by user.")
