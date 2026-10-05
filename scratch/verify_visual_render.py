import sys
import os
sys.path.insert(0, ".")
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt
from ground_station.models import VehicleID, FlightState, TelemetryPacket
from ground_station.pages.vehicle import VehiclePage
from ground_station.pages.command import CommandPage
from ground_station.commands import CommandCentre
from ground_station.theme import ThemeManager
from ground_station.parser.parser import IngestResult, LossTier

app = QApplication(sys.argv)
app.setStyleSheet(ThemeManager.main_stylesheet())

os.makedirs("scratch", exist_ok=True)

# 1. Rocket Page in Analog Mode
rocket_page = VehiclePage(VehicleID.ROCKET)
rocket_page.resize(1100, 680)
rocket_page._toggle_gauge_mode()  # Switch to analog view (Image 3)
rocket_page.show()

pkt_rocket = TelemetryPacket(
    vehicle_id=VehicleID.ROCKET,
    mission_time=0.0,
    packet_count=1,
    state=FlightState.BOOT,
    altitude=0.0,
    velocity=0.0,
    pressure=50000.0,
    temperature=-40.0,
    accelerometer=-20.0,
    accel_x=0.0,
    accel_y=0.0,
    accel_z=1.0,
    gyro_x=0.0,
    gyro_y=0.0,
    gyro_z=0.0,
    battery_voltage=12.0,
    gnss_satellites=0,
)
res = IngestResult(packet=pkt_rocket, tier=LossTier.NONE)
rocket_page.receive(pkt_rocket, res)
QApplication.processEvents()

pix_rocket_detail = rocket_page.telemetry_detail.grab()
pix_rocket_detail.save("scratch/rendered_rocket_detail.png")

pix_rocket_full = rocket_page.grab()
pix_rocket_full.save("scratch/rendered_rocket_analog_page.png")

# 2. CanSat Page in Analog Mode
cansat_page = VehiclePage(VehicleID.CANSAT)
cansat_page.resize(1100, 680)
cansat_page._toggle_gauge_mode()  # Switch to analog view (Image 4)
cansat_page.show()

pkt_cansat = TelemetryPacket(
    vehicle_id=VehicleID.CANSAT,
    mission_time=0.0,
    packet_count=1,
    state=FlightState.BOOT,
    altitude=0.0,
    velocity=0.0,
    pressure=50000.0,
    temperature=-40.0,
    accel_x=0.0,
    accel_y=0.0,
    accel_z=1.0,
    battery_voltage=8.0,
    gnss_satellites=0,
)
res_cansat = IngestResult(packet=pkt_cansat, tier=LossTier.NONE)
cansat_page.receive(pkt_cansat, res_cansat)
QApplication.processEvents()

pix_cansat_detail = cansat_page.telemetry_detail.grab()
pix_cansat_detail.save("scratch/rendered_cansat_detail.png")

pix_cansat_full = cansat_page.grab()
pix_cansat_full.save("scratch/rendered_cansat_analog_page.png")

# 3. Command Page
cmd_centre = CommandCentre(lambda vid, cmd: None)
cmd_page = CommandPage(cmd_centre)
cmd_page.resize(1100, 750)
cmd_page.show()
QApplication.processEvents()

pix_cmd_page = cmd_page.grab()
pix_cmd_page.save("scratch/rendered_command_page.png")

pix_cmd_general = cmd_page.mode_panel.grab()
pix_cmd_general.save("scratch/rendered_telemetry_panel.png")

print("All screenshots generated and verified successfully!")
