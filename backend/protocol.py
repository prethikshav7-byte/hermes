"""
backend/protocol.py
═══════════════════
Protocol definition and line decoders for Teensy 4.1 Ground Station & Rocket (Team 2026 IN-SPACe-010).

Decodes:
  - $T: Rocket telemetry packets
  - $H: Rocket replies (ACK / NACK)
  - $B: Rocket BITE diagnostic frames
  - $G: Ground station status, events, command tracking, link stats, and radio logs
"""
from __future__ import annotations

import re
from typing import Any, Dict, List, Optional, Tuple

STATE_NAMES = [
    "BOOT", "PRELAUNCH", "BOOST", "COAST",
    "APOGEE", "DESCENT", "LANDING", "RECOVERY"
]

BITE_BIT_NAMES = {
    0: ("BARO", "Barometer MS5611 failure", True),
    1: ("ACC", "High-g accelerometer ADXL345 failure", True),
    2: ("IMU", "IMU / Gyro BNO08x failure", False),
    3: ("INA", "Voltage / current monitor INA219 failure", False),
    4: ("TEMP", "Temperature sensor DS18B20 failure", False),
    5: ("PCA", "Servo driver PCA9685 failure", True),
    6: ("SD", "Onboard SD card failure", False),
    7: ("LORA", "Radio module Ra-01H failure", True),
    8: ("GNSS", "GNSS SkyTraq PX1125G failure / no fix", False),
    9: ("WDOG", "Watchdog triggered previously", False),
    10: ("BATT", "5V rail / battery out of range", True),
}


def parse_telemetry(line: str) -> Optional[Dict[str, Any]]:
    """Parse $T line into telemetry dictionary.
    Format:
    $T,2026 IN-SPACe-010,time_s,count,alt_0.1m,press_Pa,temp_0.1C,volt_0.01V,gnss_s_of_hour,lat_1e-4,lon_1e-4,gnss_alt_0.1m,sats,acc_0.1ms2,spin_0.1dps,state,vehicle,vel_ms
    """
    if not line.startswith("$T,"):
        return None
    parts = line.strip().split(",")
    if len(parts) < 18:
        return None
    try:
        team_id = parts[1].strip()
        time_s = int(parts[2])
        count = int(parts[3])
        alt_m = round(int(parts[4]) * 0.1, 2)
        press_pa = int(parts[5])
        temp_c = round(int(parts[6]) * 0.1, 2)
        volt_v = round(int(parts[7]) * 0.01, 2)
        gnss_time = int(parts[8])
        lat = round(int(parts[9]) * 1e-4, 5)
        lon = round(int(parts[10]) * 1e-4, 5)
        gnss_alt_m = round(int(parts[11]) * 0.1, 2)
        sats = int(parts[12])
        acc_ms2 = round(int(parts[13]) * 0.1, 2)
        spin_dps = round(int(parts[14]) * 0.1, 2)
        state_int = int(parts[15])
        state_name = STATE_NAMES[state_int] if 0 <= state_int < len(STATE_NAMES) else f"UNKNOWN({state_int})"
        vehicle_id = int(parts[16])
        vel_ms = round(float(parts[17]), 2)

        return {
            "type": "telemetry",
            "team_id": team_id,
            "mission_time_s": time_s,
            "packet_count": count,
            "altitude_m": alt_m,
            "pressure_pa": press_pa,
            "temperature_c": temp_c,
            "voltage_v": volt_v,
            "gnss_s_of_hour": gnss_time,
            "latitude_deg": lat,
            "longitude_deg": lon,
            "gnss_altitude_m": gnss_alt_m,
            "satellites": sats,
            "acceleration_ms2": acc_ms2,
            "spin_rate_dps": spin_dps,
            "state": state_int,
            "state_name": state_name,
            "vehicle_id": vehicle_id,
            "velocity_ms": vel_ms,
            "raw": line.strip(),
        }
    except Exception:
        return None


def parse_reply(line: str) -> Optional[Dict[str, Any]]:
    """Parse $H rocket reply lines ($H,ACK,... or $H,NACK,...)."""
    if not line.startswith("$H,"):
        return None
    parts = line.strip().split(",")
    if len(parts) < 3:
        return None
    ack_type = parts[1].strip()  # ACK or NACK
    cmd_name = parts[2].strip()
    args = [p.strip() for p in parts[3:]]

    detail: Dict[str, Any] = {}
    if ack_type == "NACK":
        detail["reason"] = args[0] if args else "unknown"
    else:
        # Decoded args for specific commands
        if cmd_name == "PG":
            detail["status"] = "pong"
        elif cmd_name == "LK":
            detail["status"] = "link_ok"
            if len(args) >= 2:
                detail["rssi"] = args[0]
                detail["snr"] = args[1]
        elif cmd_name == "ST":
            if len(args) >= 11:
                try:
                    detail["state"] = int(args[0])
                    detail["state_name"] = STATE_NAMES[int(args[0])] if int(args[0]) < len(STATE_NAMES) else "UNKNOWN"
                    detail["state_token"] = int(args[1])
                    detail["flags"] = args[2]
                    detail["servo_deg"] = [int(args[3]), int(args[4]), int(args[5])]
                    detail["servo_est"] = [int(args[6]), int(args[7]), int(args[8])]
                    detail["seq_mode"] = int(args[9])
                    detail["seq_step"] = int(args[10])
                except Exception:
                    pass
        elif cmd_name in ("CAL", "CALA", "CALI"):
            detail["cal_args"] = args
        elif cmd_name in ("LOCK", "UNLOCK", "SV", "PW"):
            if len(args) >= 2:
                detail["channel"] = args[0]
                detail["value"] = args[1]

    return {
        "type": "reply",
        "ack": ack_type == "ACK",
        "name": cmd_name,
        "args": args,
        "detail": detail,
        "raw": line.strip(),
    }


def parse_bite(line: str) -> Optional[Dict[str, Any]]:
    """Parse $B BITE frames:
    $B,<state>,<status>,<mask_hex>,<letters>,<vrail_V>,<sats>,<token>
    """
    if not line.startswith("$B,"):
        return None
    parts = line.strip().split(",")
    if len(parts) < 8:
        return None
    try:
        state_int = int(parts[1])
        status_code = int(parts[2])
        mask_hex = parts[3].strip()
        mask_val = int(mask_hex, 16) if mask_hex.startswith("0X") or mask_hex.startswith("0x") else int(mask_hex, 16)
        letters = parts[4].strip()
        vrail_v = round(float(parts[5]), 2)
        sats = int(parts[6])
        token = int(parts[7])

        faults = []
        has_critical = False
        for bit, (code, text, critical) in BITE_BIT_NAMES.items():
            if mask_val & (1 << bit):
                faults.append({
                    "bit": bit,
                    "code": code,
                    "text": text,
                    "critical": critical,
                })
                if critical:
                    has_critical = True

        state_name = STATE_NAMES[state_int] if 0 <= state_int < len(STATE_NAMES) else f"UNKNOWN({state_int})"

        return {
            "type": "bite",
            "clean": mask_val == 0,
            "no_critical": not has_critical,
            "fault_mask": mask_val,
            "fault_letters": letters,
            "faults": faults,
            "vrail_v": vrail_v,
            "satellites": sats,
            "state": state_int,
            "state_name": state_name,
            "state_token": token,
            "raw": line.strip(),
        }
    except Exception:
        return None


def parse_ground_msg(line: str) -> Optional[Dict[str, Any]]:
    """Parse $G ground station messages."""
    if not line.startswith("$G,"):
        return None
    parts = line.strip().split(",")
    if len(parts) < 2:
        return None
    sub = parts[1].strip()

    if sub == "CMD":
        # $G,CMD,<id>,<status>[,<info>]
        if len(parts) >= 4:
            try:
                cmd_id = int(parts[2])
            except ValueError:
                cmd_id = 0
            status = parts[3].strip()
            info = parts[4].strip() if len(parts) > 4 else ""
            return {
                "type": "cmd",
                "id": cmd_id,
                "status": status,
                "info": info,
                "raw": line.strip(),
            }

    elif sub == "FREQ":
        # $G,FREQ,<ch>,<MHz>,<what>
        if len(parts) >= 5:
            return {
                "type": "freq",
                "channel": int(parts[2]) if parts[2].isdigit() else 0,
                "mhz": float(parts[3]) if parts[3].replace(".", "", 1).isdigit() else 0.0,
                "what": parts[4].strip(),
                "raw": line.strip(),
            }

    elif sub == "LINK":
        # $G,LINK,<rssi>,<snr>,<good>,<crc_fail>,<lost>
        if len(parts) >= 7:
            try:
                return {
                    "type": "link",
                    "rssi": float(parts[2]),
                    "snr": float(parts[3]),
                    "good": int(parts[4]),
                    "crc_fail": int(parts[5]),
                    "lost": int(parts[6]),
                    "raw": line.strip(),
                }
            except ValueError:
                pass

    elif sub == "STATUS":
        # $G,STATUS,telem,status,crc,lost,rssi,snr,last_rx_ms,reinit,uplinks,link,radio,lora_enabled,channel
        if len(parts) >= 15:
            try:
                return {
                    "type": "gs_status",
                    "rx_telem": int(parts[2]),
                    "rx_status": int(parts[3]),
                    "rx_crc": int(parts[4]),
                    "lost": int(parts[5]),
                    "rssi": float(parts[6]),
                    "snr": float(parts[7]),
                    "last_rx_ms": int(parts[8]),
                    "reinits": int(parts[9]),
                    "uplinks": int(parts[10]),
                    "link_active": bool(int(parts[11])),
                    "radio_ok": bool(int(parts[12])),
                    "lora_enabled": bool(int(parts[13])),
                    "channel": int(parts[14]),
                    "raw": line.strip(),
                }
            except ValueError:
                pass

    elif sub == "LORA":
        # $G,LORA,ENABLED|DISABLED
        state = parts[2].strip() if len(parts) > 2 else ""
        return {
            "type": "lora",
            "state": state,
            "enabled": state == "ENABLED",
            "raw": line.strip(),
        }

    elif sub == "RADIO":
        return {
            "type": "radio",
            "detail": parts[2:],
            "raw": line.strip(),
        }

    elif sub == "CRC":
        return {
            "type": "crc",
            "count": int(parts[2]) if len(parts) > 2 and parts[2].isdigit() else 0,
            "rssi": float(parts[3]) if len(parts) > 3 else 0.0,
            "raw": line.strip(),
        }

    elif sub == "FOREIGN":
        return {
            "type": "foreign",
            "team": parts[2].strip() if len(parts) > 2 else "unknown",
            "raw": line.strip(),
        }

    return {
        "type": "ground",
        "sub": sub,
        "parts": parts[2:],
        "raw": line.strip(),
    }


def decode_line(line: str) -> Dict[str, Any]:
    """Decode any line from Teensy into an event dictionary."""
    clean = line.strip()
    if clean.startswith("$T,"):
        res = parse_telemetry(clean)
        if res:
            return res
    elif clean.startswith("$H,"):
        res = parse_reply(clean)
        if res:
            return res
    elif clean.startswith("$B,"):
        res = parse_bite(clean)
        if res:
            return res
    elif clean.startswith("$G,"):
        res = parse_ground_msg(clean)
        if res:
            return res

    return {
        "type": "raw",
        "raw": clean,
    }
