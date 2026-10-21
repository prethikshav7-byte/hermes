"""
ground_station/backend_client.py
════════════════════════════════
Qt-native WebSocket client connecting to the backend bridge (ws://localhost:8765/ws).
All serial communication is handled by backend/server.py.
"""
from __future__ import annotations

import json
from typing import Any, Dict, Optional

from PyQt6.QtCore import QObject, QTimer, QUrl, pyqtSignal
from PyQt6.QtWebSockets import QWebSocket


class GroundStationBackendClient(QObject):
    connected = pyqtSignal()
    disconnected = pyqtSignal()
    state_changed = pyqtSignal(dict)
    buttons_changed = pyqtSignal(dict)
    telemetry_received = pyqtSignal(dict)
    link_received = pyqtSignal(dict)
    gs_status_received = pyqtSignal(dict)
    reply_received = pyqtSignal(dict)
    bite_received = pyqtSignal(dict)
    freq_received = pyqtSignal(dict)
    raw_message_received = pyqtSignal(str)
    notice = pyqtSignal(str)

    def __init__(self, url: str = "ws://localhost:8765/ws", parent=None):
        super().__init__(parent)
        self.url = url
        self.ws = QWebSocket()

        self.ws.connected.connect(self._on_connected)
        self.ws.disconnected.connect(self._on_disconnected)
        self.ws.textMessageReceived.connect(self._on_text_message)

        self._reconnect_timer = QTimer(self)
        self._reconnect_timer.setInterval(1500)
        self._reconnect_timer.timeout.connect(self._try_connect)

        self.state: Dict[str, Any] = {
            "serial": {"state": "DISCONNECTED", "port": "—", "info": "Connecting..."},
            "lora_enabled": False,
            "radio_ok": False,
            "channel": 0,
            "armed": False,
            "target": "ROCKET",
            "telemetry_flowing": False,
        }

        self.button_colors: Dict[str, str] = {}
        self.button_tooltips: Dict[str, str] = {}
        self.default_color = "red"
        self._is_connected = False

        self._try_connect()

    def _try_connect(self) -> None:
        if self._is_connected:
            self._reconnect_timer.stop()
            return
        self.default_color = "yellow"
        self.buttons_changed.emit({
            "default": "yellow",
            "buttons": self.button_colors,
            "tooltips": self.button_tooltips,
        })
        self.ws.open(QUrl(self.url))

    def _on_connected(self) -> None:
        self._is_connected = True
        self._reconnect_timer.stop()
        self.connected.emit()
        self.notice.emit("Connected to Ground Station Backend Bridge")

    def _on_disconnected(self) -> None:
        self._is_connected = False
        self.default_color = "red"
        self.state["serial"]["state"] = "DISCONNECTED"
        self.state_changed.emit(self.state)
        self.buttons_changed.emit({
            "default": "red",
            "buttons": {},
            "tooltips": {"default": "Backend offline / bridge disconnected"},
        })
        self.disconnected.emit()
        self._reconnect_timer.start()

    def _on_text_message(self, message: str) -> None:
        try:
            data = json.loads(message)
        except Exception:
            self.raw_message_received.emit(message)
            return

        mtype = data.get("type", "raw")
        raw = data.get("raw", "")
        if raw:
            self.raw_message_received.emit(raw)

        if mtype == "state":
            self.state = data
            self.state_changed.emit(data)
        elif mtype == "buttons":
            self.default_color = data.get("default", "neutral")
            self.button_colors = data.get("buttons", {})
            self.button_tooltips = data.get("tooltips", {})
            self.buttons_changed.emit(data)
        elif mtype == "telemetry":
            self.telemetry_received.emit(data)
        elif mtype == "link":
            self.link_received.emit(data)
        elif mtype == "gs_status":
            self.gs_status_received.emit(data)
        elif mtype == "reply":
            self.reply_received.emit(data)
        elif mtype == "bite":
            self.bite_received.emit(data)
        elif mtype == "freq":
            self.freq_received.emit(data)

    # ── Command & Control Methods ────────────────────────────────────────

    def send_command(self, cmd: str, key: Optional[str] = None, seq: Optional[int] = None) -> bool:
        if not self._is_connected:
            self.notice.emit("Cannot send command: Backend disconnected")
            return False
        payload = json.dumps({"type": "cmd", "cmd": cmd, "key": key or cmd, "seq": seq})
        self.ws.sendTextMessage(payload)
        return True

    def send_raw(self, line: str) -> bool:
        if not self._is_connected:
            return False
        payload = json.dumps({"type": "send_raw", "text": line})
        self.ws.sendTextMessage(payload)
        return True

    def set_armed(self, armed: bool) -> None:
        if not self._is_connected:
            return
        payload = json.dumps({"type": "set_armed", "armed": bool(armed)})
        self.ws.sendTextMessage(payload)

    def set_target(self, target: str) -> None:
        if not self._is_connected:
            return
        payload = json.dumps({"type": "set_target", "target": str(target)})
        self.ws.sendTextMessage(payload)

    def select_port(self, port: str) -> None:
        if not self._is_connected:
            return
        payload = json.dumps({"type": "select_port", "port": str(port)})
        self.ws.sendTextMessage(payload)

    def color_for(self, key: str) -> str:
        if not self._is_connected:
            return self.default_color or "red"
        return self.button_colors.get(key, self.default_color)

    def tooltip_for(self, key: str) -> str:
        return self.button_tooltips.get(key, self.button_tooltips.get("default", ""))
