import sys
sys.path.insert(0, ".")
from PyQt6.QtWidgets import QApplication
from ground_station.models import VehicleID, FlightState, TelemetryPacket
from ground_station.parser.parser import IngestResult, LossTier
from ground_station.pages.vehicle import VehiclePage
from ground_station.theme import ThemeManager

app = QApplication(sys.argv)
app.setStyleSheet(ThemeManager.main_stylesheet())

# Rocket Page
rocket_page = VehiclePage(VehicleID.ROCKET)
rocket_page.resize(1200, 800)
rocket_page.show()

pkt_rocket = TelemetryPacket(
    vehicle_id=VehicleID.ROCKET,
    mission_time=24.9,
    packet_count=100,
    state=FlightState.BOOT,
    altitude=950.30,
    velocity=-11.64,
    pressure=90381.2,
    temperature=15.73,
    accelerometer=1.56,
    accel_x=0.0,
    accel_y=0.0,
    accel_z=1.56,
    battery_voltage=8.27,
    gnss_satellites=0,
    extras={"RSSI": "-77"},
)
res = IngestResult(packet=pkt_rocket, tier=LossTier.NONE)
rocket_page.receive(pkt_rocket, res)
for g in rocket_page.gauges.values():
    g.digital._flush()

# 1. Default Rocket Digital Opening View (Matching Image 1)
rocket_page.grab().save("scratch/rocket_digital_open.png")

# 2. Rocket Analog View after clicking 'Show analog gauges' (Matching Image 3)
rocket_page._toggle_gauge_mode()
rocket_page.grab().save("scratch/rocket_analog_toggled.png")

# CanSat Page
cansat_page = VehiclePage(VehicleID.CANSAT)
cansat_page.resize(1200, 800)
cansat_page.show()

pkt_cansat = TelemetryPacket(
    vehicle_id=VehicleID.CANSAT,
    mission_time=45.1,
    packet_count=200,
    state=FlightState.BOOT,
    altitude=794.87,
    velocity=-8.45,
    pressure=92128.3,
    temperature=16.73,
    accel_x=0.0,
    accel_y=0.0,
    accel_z=1.30,
    battery_voltage=8.22,
    gnss_satellites=0,
    extras={"RSSI": "-80"},
)
res_cansat = IngestResult(packet=pkt_cansat, tier=LossTier.NONE)
cansat_page.receive(pkt_cansat, res_cansat)
for g in cansat_page.gauges.values():
    g.digital._flush()

# 3. Default CanSat Digital Opening View (Matching Image 2)
cansat_page.grab().save("scratch/cansat_digital_open.png")

# 4. CanSat Analog View after clicking 'Show analog gauges' (Matching Image 4)
cansat_page._toggle_gauge_mode()
cansat_page.grab().save("scratch/cansat_analog_toggled.png")

print("All 4 view screenshots successfully saved!")
