"""
backend/fake_teensy.py
══════════════════════
Simulated Teensy 4.1 Ground Station + Rocket for testing without physical hardware.

Emulates:
  - Ground Teensy: USB serial protocol (<id>,<CMD>), $G messages, LoRa state, frequency switching
  - Rocket: 1 Hz $T telemetry when LIVE, $H replies, $B BITE frames
"""
from __future__ import annotations

import argparse
import asyncio
import math
import random
import sys
import time
from typing import Optional

TEAM_ID = "2026 IN-SPACe-010"


class FakeTeensy:
    def __init__(self):
        self.lora_enabled = False
        self.radio_ok = True
        self.channel = 0
        self.telemetry_live = False
        self.telemetry_count = 0
        self.start_time = time.time()

        # Simulated flight state
        self.flight_state = 1  # 0=BOOT, 1=PRELAUNCH
        self.altitude = 0.0
        self.velocity = 0.0
        self.lat = 8.5241
        self.lon = 76.9366
        self.gnss_alt = 120.0
        self.temp_c = 28.5
        self.volt_v = 5.04
        self.press_pa = 101325
        self.sats = 11

        self.seq_step = 0
        self.seq_mode = 0  # 0=IDLE, 1=MANUAL

    def get_telemetry_line(self) -> str:
        self.telemetry_count += 1
        t_sec = int(time.time() - self.start_time)

        # Small physical jitter
        press = self.press_pa + random.randint(-15, 15)
        temp = round(self.temp_c + random.uniform(-0.2, 0.2), 1)
        volt = round(self.volt_v + random.uniform(-0.02, 0.02), 2)
        acc = 9.8 + random.uniform(-0.1, 0.1)
        spin = round(random.uniform(-0.5, 0.5), 1)
        sats = self.sats + random.choice([0, 0, 0, 1, -1])

        # Convert to protocol integer scaling
        alt_int = int(round(self.altitude * 10))
        temp_int = int(round(temp * 10))
        volt_int = int(round(volt * 100))
        lat_int = int(round(self.lat * 10000))
        lon_int = int(round(self.lon * 10000))
        gnss_alt_int = int(round(self.gnss_alt * 10))
        acc_int = int(round(acc * 10))
        spin_int = int(round(abs(spin) * 10))
        vel_int = int(round(abs(self.velocity)))

        # Format: $T,2026 IN-SPACe-010,time_s,count,alt_0.1m,press_Pa,temp_0.1C,volt_0.01V,gnss_s_of_hour,lat_1e-4,lon_1e-4,gnss_alt_0.1m,sats,acc_0.1ms2,spin_0.1dps,state,vehicle,vel_ms
        return (
            f"$T,{TEAM_ID},{t_sec},{self.telemetry_count},{alt_int},{press},"
            f"{temp_int},{volt_int},{t_sec % 3600},{lat_int},{lon_int},"
            f"{gnss_alt_int},{sats},{acc_int},{spin_int},{self.flight_state},1,{vel_int}"
        )

    def handle_line(self, line: str) -> list[str]:
        line = line.strip()
        if not line:
            return []

        out = []
        cmd_id = 0
        body = line

        if "," in line and line.split(",")[0].isdigit():
            parts = line.split(",", 1)
            cmd_id = int(parts[0])
            body = parts[1].strip()

        tokens = body.split(",")
        cmd = tokens[0].upper()

        # Local ground station commands
        if cmd == "GS":
            out.append(
                f"$G,STATUS,{self.telemetry_count},0,0,0,-64.0,9.5,100,0,1,1,"
                f"{1 if self.radio_ok else 0},{1 if self.lora_enabled else 0},{self.channel}"
            )
            if cmd_id:
                out.append(f"$G,CMD,{cmd_id},OK")
            return out

        if cmd in ("ENABLE", "LINK"):
            self.lora_enabled = True
            self.radio_ok = True
            out.append("$G,LORA,ENABLED")
            out.append(f"$G,FREQ,{self.channel},{'866.50' if self.channel == 0 else '865.25'},MANUAL")
            if cmd_id:
                out.append(f"$G,CMD,{cmd_id},OK")
            return out

        if cmd == "DISABLE":
            self.lora_enabled = False
            self.telemetry_live = False
            out.append("$G,LORA,DISABLED")
            if cmd_id:
                out.append(f"$G,CMD,{cmd_id},OK")
            return out

        if cmd == "GF":
            ch = int(tokens[1]) if len(tokens) > 1 and tokens[1].isdigit() else 0
            self.channel = ch
            out.append(f"$G,FREQ,{self.channel},{'866.50' if self.channel == 0 else '865.25'},MANUAL")
            if cmd_id:
                out.append(f"$G,CMD,{cmd_id},OK")
            return out

        # Rocket commands
        if not self.lora_enabled:
            if cmd_id:
                out.append(f"$G,CMD,{cmd_id},REJECTED,lora_disabled")
            return out

        # Acknowledge queueing
        if cmd_id:
            out.append(f"$G,CMD,{cmd_id},QUEUED")
            out.append(f"$G,CMD,{cmd_id},SENT,1")

        # Command behavior
        if cmd in ("LIVE", "SD"):
            self.telemetry_live = True
            out.append(f"$G,CMD,{cmd_id},OK")

        elif cmd in ("HIBERNATE", "XD"):
            self.telemetry_live = False
            out.append("$H,ACK,XD")
            out.append(f"$G,CMD,{cmd_id},OK")

        elif cmd == "PG":
            out.append("$H,ACK,PG")
            out.append(f"$G,CMD,{cmd_id},OK")

        elif cmd == "LK":
            out.append("$H,ACK,LK,-62,9.8")
            out.append(f"$G,CMD,{cmd_id},OK")

        elif cmd == "ST":
            out.append(f"$H,ACK,ST,{self.flight_state},101,0x00,0,0,0,0,0,0,{self.seq_mode},{self.seq_step}")
            out.append(f"$G,CMD,{cmd_id},OK")

        elif cmd == "BITE":
            out.append(f"$B,{self.flight_state},0,0x0000,OK,5.04,{self.sats},101")
            out.append(f"$G,CMD,{cmd_id},OK")

        elif cmd == "CALA":
            out.append("$H,ACK,CALA,1,101325,28.5,120.0")
            out.append(f"$G,CMD,{cmd_id},OK")

        elif cmd == "CALI":
            out.append("$H,ACK,CALI,1,0,1,0,0,0,0,1,0.0")
            out.append(f"$G,CMD,{cmd_id},OK")

        elif cmd == "CAL":
            out.append("$H,ACK,CAL,1,OK,OK")
            out.append(f"$G,CMD,{cmd_id},OK")

        elif cmd in ("LOCK", "UNLOCK"):
            ch = tokens[1] if len(tokens) > 1 else "0"
            ang = "50" if cmd == "LOCK" else "0"
            out.append(f"$H,ACK,{cmd},{ch},{ang}")
            out.append(f"$G,CMD,{cmd_id},OK")

        elif cmd in ("SV", "PW"):
            ch = tokens[1] if len(tokens) > 1 else "0"
            val = tokens[2] if len(tokens) > 2 else "0"
            out.append(f"$H,ACK,{cmd},{ch},{val}")
            out.append(f"$G,CMD,{cmd_id},OK")

        elif cmd == "LOCKALL":
            out.append("$H,ACK,LOCKALL")
            out.append(f"$G,CMD,{cmd_id},OK")

        elif cmd.startswith("TEST"):
            out.append(f"$H,ACK,{cmd},0,0")
            out.append(f"$G,CMD,{cmd_id},OK")

        elif cmd == "MSEQ":
            self.seq_mode = 1
            self.seq_step = 0
            out.append("$H,ACK,MSEQ,0")
            out.append(f"$G,CMD,{cmd_id},OK")

        elif cmd == "NSTEP":
            self.seq_step += 1
            out.append(f"$H,ACK,NSTEP,{self.seq_step},0")
            out.append(f"$G,CMD,{cmd_id},OK")

        elif cmd == "FSEQ":
            out.append("$H,ACK,FSEQ")
            out.append(f"$G,CMD,{cmd_id},OK")

        elif cmd == "SEQRST":
            self.seq_mode = 0
            self.seq_step = 0
            out.append("$H,ACK,SEQRST")
            out.append(f"$G,CMD,{cmd_id},OK")

        elif cmd == "SEP":
            out.append("$H,ACK,SEP,0x01")
            out.append(f"$G,CMD,{cmd_id},OK")

        elif cmd == "BZ":
            v = tokens[1] if len(tokens) > 1 else "0"
            out.append(f"$H,ACK,BZ,{v}")
            out.append(f"$G,CMD,{cmd_id},OK")

        elif cmd == "FQ":
            ch = int(tokens[1]) if len(tokens) > 1 and tokens[1].isdigit() else 0
            self.channel = ch
            out.append(f"$H,ACK,FQ,{ch}")
            out.append(f"$G,FREQ,{ch},{'866.50' if ch == 0 else '865.25'},CONFIRMED")
            out.append(f"$G,CMD,{cmd_id},OK")

        elif cmd == "TX":
            v = tokens[1] if len(tokens) > 1 else "1"
            out.append(f"$H,ACK,TX,{v}")
            out.append(f"$G,CMD,{cmd_id},OK")

        elif cmd == "RF":
            out.append("$H,ACK,RF")
            out.append(f"$G,CMD,{cmd_id},OK")

        elif cmd in ("GR", "LR", "FPL"):
            out.append(f"$H,ACK,{cmd}")
            out.append(f"$G,CMD,{cmd_id},OK")

        else:
            out.append(f"$G,CMD,{cmd_id},OK")

        return out


async def handle_client(reader: asyncio.StreamReader, writer: asyncio.StreamWriter, teensy: FakeTeensy):
    print("[FakeTeensy] Client connected.")

    # Periodic telemetry task
    async def telemetry_loop():
        while not writer.is_closing():
            if teensy.lora_enabled and teensy.telemetry_live:
                t_line = teensy.get_telemetry_line() + "\n"
                link_line = f"$G,LINK,-62.0,9.6,{teensy.telemetry_count},0,0\n"
                try:
                    writer.write(t_line.encode("utf-8"))
                    writer.write(link_line.encode("utf-8"))
                    await writer.drain()
                except Exception:
                    break
            await asyncio.sleep(1.0)

    telem_task = asyncio.create_task(telemetry_loop())

    try:
        while not reader.at_eof():
            line_bytes = await reader.readline()
            if not line_bytes:
                break
            line = line_bytes.decode("utf-8", errors="replace")
            replies = teensy.handle_line(line)
            for r in replies:
                writer.write((r + "\n").encode("utf-8"))
            await writer.drain()
    except Exception as e:
        print(f"[FakeTeensy] Connection error: {e}")
    finally:
        telem_task.cancel()
        writer.close()
        print("[FakeTeensy] Client disconnected.")


async def main():
    parser = argparse.ArgumentParser(description="Simulated Teensy 4.1 Ground Station & Rocket")
    parser.add_argument("--tcp", type=int, default=8766, help="TCP port to listen on (default 8766)")
    args = parser.parse_args()

    teensy = FakeTeensy()
    server = await asyncio.start_server(
        lambda r, w: handle_client(r, w, teensy),
        "0.0.0.0", args.tcp
    )
    print(f"FakeTeensy running on tcp://localhost:{args.tcp}")
    print("Run bridge with: python backend/server.py --port tcp://localhost:8766")

    async with server:
        await server.serve_forever()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nFakeTeensy stopped.")
