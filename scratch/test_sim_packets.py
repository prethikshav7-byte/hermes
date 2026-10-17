import sys
sys.path.insert(0, ".")

from ground_station.models import VehicleID, FlightState
from ground_station.graphs.trajectory import FlightProfile
from ground_station.simulator.engine import SimulatedFlight
from ground_station.codec import CsvCodec
from ground_station.parser import TelemetryDemux

def test_packet_generation():
    profile = FlightProfile()
    flight = SimulatedFlight(VehicleID.ROCKET, profile)
    codec = CsvCodec()
    demux = TelemetryDemux(codec)

    for i in range(1, 200):
        t = round(i * 0.1, 3)
        vals = flight.values(t)
        base = codec.encode(vals)
        line = base + f",RSSI={flight.rssi(t):.1f},SNR=8.5,SIM=1\n"
        res = demux.feed(line)
        assert res is not None, f"feed returned None for line: {line}"
        assert res.accepted, f"packet rejected: {res.rejected.reason} - {res.rejected.detail} on line: {line}"
        assert res.packet.packet_count == i
        assert res.packet.vehicle_id == VehicleID.ROCKET

    print("Rocket: 199 packets passed validation with 0 rejections!")

    flight_can = SimulatedFlight(VehicleID.CANSAT, profile)
    for i in range(1, 200):
        t = round(i * 0.1, 3)
        vals = flight_can.values(t)
        base = codec.encode(vals)
        line = base + f",RSSI={flight_can.rssi(t):.1f},SNR=8.5,SIM=1\n"
        res = demux.feed(line)
        assert res is not None, f"feed returned None for line: {line}"
        assert res.accepted, f"packet rejected: {res.rejected.reason} - {res.rejected.detail} on line: {line}"
        assert res.packet.packet_count == i
        assert res.packet.vehicle_id == VehicleID.CANSAT

    print("CanSat: 199 packets passed validation with 0 rejections!")

if __name__ == "__main__":
    test_packet_generation()
