/**
 * frontend/gsClient.js
 * Framework-free WebSocket client for the MACHZONE Teensy 4.1 Ground Station Bridge.
 */

class GroundStationClient {
  constructor(url) {
    this.url = url || `ws://${window.location.hostname || "localhost"}:8765/ws`;
    this.ws = null;
    this.listeners = new Map();
    this.reconnectTimer = null;
    this.reconnectInterval = 1500;
    this.isConnected = false;

    // Local cached state
    this.state = {
      serial: { state: "DISCONNECTED", port: "—", info: "" },
      lora_enabled: false,
      radio_ok: false,
      channel: 0,
      armed: false,
      target: "ROCKET",
      telemetry_flowing: false,
    };

    // Button colors & tooltips computed by backend
    this.buttonColors = {};
    this.buttonTooltips = {};
    this.defaultColor = "red";

    this.connect();
  }

  connect() {
    if (this.ws && (this.ws.readyState === WebSocket.OPEN || this.ws.readyState === WebSocket.CONNECTING)) {
      return;
    }

    try {
      this.ws = new WebSocket(this.url);
    } catch (err) {
      console.warn("[GS Client] Connection error:", err);
      this._scheduleReconnect();
      return;
    }

    this.ws.onopen = () => {
      console.log("[GS Client] Connected to bridge:", this.url);
      this.isConnected = true;
      if (this.reconnectTimer) {
        clearTimeout(this.reconnectTimer);
        this.reconnectTimer = null;
      }
      this._emit("connect", { url: this.url });
    };

    this.ws.onclose = () => {
      this.isConnected = false;
      this.defaultColor = "red";
      this.buttonColors = {};
      this.state.serial.state = "DISCONNECTED";
      this._emit("disconnect", {});
      this._emit("state", this.state);
      this._emit("buttons", { default: "red", buttons: {}, tooltips: {} });
      this._scheduleReconnect();
    };

    this.ws.onerror = (err) => {
      console.warn("[GS Client] WebSocket error:", err);
      this.ws.close();
    };

    this.ws.onmessage = (event) => {
      try {
        const msg = JSON.parse(event.data);
        this._handleMessage(msg);
      } catch (e) {
        console.error("[GS Client] Failed to parse message:", e, event.data);
      }
    };
  }

  _scheduleReconnect() {
    if (this.reconnectTimer) return;
    this.defaultColor = "yellow"; // Yellow while reconnecting
    this._emit("buttons", { default: "yellow", buttons: this.buttonColors, tooltips: {} });
    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null;
      this.connect();
    }, this.reconnectInterval);
  }

  _handleMessage(msg) {
    const type = msg.type || "raw";

    if (type === "state") {
      this.state = msg;
      this._emit("state", msg);
    } else if (type === "buttons") {
      this.defaultColor = msg.default || "red";
      this.buttonColors = msg.buttons || {};
      this.buttonTooltips = msg.tooltips || {};
      this._emit("buttons", msg);
      this._updateBoundButtons();
    } else if (type === "telemetry") {
      this._emit("telemetry", msg);
    } else if (type === "link") {
      this._emit("link", msg);
    } else if (type === "gs_status") {
      this._emit("gs_status", msg);
    } else if (type === "reply") {
      this._emit("reply", msg);
    } else if (type === "bite") {
      this._emit("bite", msg);
    } else if (type === "freq") {
      this._emit("freq", msg);
    }

    // Always emit type and wildcard
    this._emit(type, msg);
    this._emit("*", msg);
  }

  on(type, callback) {
    if (!this.listeners.has(type)) {
      this.listeners.set(type, new Set());
    }
    this.listeners.get(type).add(callback);
    return () => this.off(type, callback);
  }

  off(type, callback) {
    if (this.listeners.has(type)) {
      this.listeners.get(type).delete(callback);
    }
  }

  _emit(type, data) {
    if (this.listeners.has(type)) {
      for (const cb of this.listeners.get(type)) {
        try {
          cb(data);
        } catch (e) {
          console.error(`[GS Client] Error in listener for ${type}:`, e);
        }
      }
    }
  }

  // ── Actions ──────────────────────────────────────────────────────────

  send(cmd, key) {
    if (!this.isConnected || !this.ws) {
      console.warn("[GS Client] Cannot send command, client offline:", cmd);
      return false;
    }
    const payload = JSON.stringify({
      type: "cmd",
      cmd: cmd,
      key: key || cmd,
    });
    this.ws.send(payload);
    return true;
  }

  sendRaw(line) {
    if (!this.isConnected || !this.ws) return false;
    this.ws.send(JSON.stringify({ type: "send_raw", text: line }));
    return true;
  }

  setArmed(armed) {
    if (!this.isConnected || !this.ws) return;
    this.ws.send(JSON.stringify({ type: "set_armed", armed: Boolean(armed) }));
  }

  setTarget(target) {
    if (!this.isConnected || !this.ws) return;
    this.ws.send(JSON.stringify({ type: "set_target", target: target }));
  }

  selectPort(port) {
    if (!this.isConnected || !this.ws) return;
    this.ws.send(JSON.stringify({ type: "select_port", port: port }));
  }

  colorFor(key) {
    if (!this.isConnected) {
      return this.reconnectTimer ? "yellow" : "red";
    }
    return this.buttonColors[key] || this.defaultColor || "red";
  }

  tooltipFor(key) {
    return this.buttonTooltips[key] || (this.buttonTooltips.default || "");
  }

  // ── DOM Binding Utility ──────────────────────────────────────────────

  bindButtons(container) {
    const root = container || document;
    const btns = root.querySelectorAll("[data-cmd]");
    btns.forEach((btn) => {
      const cmd = btn.getAttribute("data-cmd");
      const key = btn.getAttribute("data-key") || cmd;
      btn.onclick = () => {
        const confirmMsg = btn.getAttribute("data-confirm");
        if (confirmMsg && !window.confirm(confirmMsg)) {
          return;
        }
        this.send(cmd, key);
      };
    });
    this._boundContainer = root;
    this._updateBoundButtons();
  }

  _updateBoundButtons() {
    if (!this._boundContainer) return;
    const btns = this._boundContainer.querySelectorAll("[data-cmd]");
    btns.forEach((btn) => {
      const key = btn.getAttribute("data-key") || btn.getAttribute("data-cmd");
      const color = this.colorFor(key);
      btn.classList.remove("gs-green", "gs-yellow", "gs-red");
      btn.classList.add(`gs-${color}`);
      const tip = this.tooltipFor(key);
      if (tip) {
        btn.title = tip;
      }
    });
  }
}

// Export for module systems or global
if (typeof module !== "undefined" && module.exports) {
  module.exports = { GroundStationClient };
} else {
  window.GroundStationClient = GroundStationClient;
}
