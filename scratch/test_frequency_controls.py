import sys
import os
sys.path.insert(0, ".")
import pytest
from PyQt6.QtWidgets import QApplication
from ground_station.commands import CommandCentre
from ground_station.models import VehicleID
from ground_station.pages.command import CommandPage

@pytest.fixture(scope="session")
def qapp():
    app = QApplication.instance()
    if app is None:
        app = QApplication([])
    return app

def test_frequency_controls(qapp):
    sent_commands = []
    centre = CommandCentre(lambda vid, cmd, seq=0: sent_commands.append((vid, cmd)))
    page = CommandPage(centre)

    # Check Refresh Processor button is present
    assert "REFRESH_PROCESSOR" in page._command_buttons
    assert len(page._command_buttons) == 1

    # Check Rocket Frequency buttons
    assert "ORIGINAL" in page._rocket_freq_buttons
    assert "BACKUP" in page._rocket_freq_buttons
    assert page._rocket_freq_buttons["ORIGINAL"].isChecked()
    assert not page._rocket_freq_buttons["BACKUP"].isChecked()

    # Click Rocket Backup
    page._rocket_freq_buttons["BACKUP"].click()
    assert page._rocket_freq == "BACKUP"
    assert not page._rocket_freq_buttons["ORIGINAL"].isChecked()
    assert page._rocket_freq_buttons["BACKUP"].isChecked()
    assert (VehicleID.ROCKET, "FREQ:ROCKET:BACKUP") in sent_commands

    # Click CanSat Backup
    page._cansat_freq_buttons["BACKUP"].click()
    assert page._cansat_freq == "BACKUP"
    assert not page._cansat_freq_buttons["ORIGINAL"].isChecked()
    assert page._cansat_freq_buttons["BACKUP"].isChecked()
    assert (VehicleID.CANSAT, "FREQ:CANSAT:BACKUP") in sent_commands

    # Click Rocket Original
    page._rocket_freq_buttons["ORIGINAL"].click()
    assert page._rocket_freq == "ORIGINAL"
    assert page._rocket_freq_buttons["ORIGINAL"].isChecked()
    assert not page._rocket_freq_buttons["BACKUP"].isChecked()
    assert (VehicleID.ROCKET, "FREQ:ROCKET:ORIGINAL") in sent_commands
