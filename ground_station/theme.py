"""
ground_station/theme.py
═══════════════════════
Dark-mode colour palette and global stylesheet.

Every colour reference in the UI goes through ThemeManager.C() so that a
palette swap is a one-place edit and so that widgets can be built before
the QApplication stylesheet is applied.  Nothing here touches Qt directly
except the stylesheet generation, so the palette dataclass is importable
in tests.
"""
from __future__ import annotations

from dataclasses import dataclass


# ── Typographic scale (#2) ────────────────────────────────────────────────────
# Three levels, used everywhere a font size is needed.  Import these rather
# than hard-coding a number so the whole app stays on the same grid.
FS_HEADING: int = 11    # page titles, section headings
FS_BODY:    int = 9     # labels, button text, table cells
FS_CAPTION: int = 8     # dim secondary labels, timestamps, unit text
# ─────────────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class Palette:
    """The full colour palette used across all widgets."""
    # Backgrounds - deep aerospace navy and near-black surfaces
    BG:            str = "#080c14"   # window / outermost layer
    BG_CARD:       str = "#0d1420"   # card / panel backgrounds
    BG_CARD_HOVER: str = "#131f30"   # card hover / highlight
    BG_PANEL:      str = "#0a0f19"   # pinned strip backgrounds (health bar, header)
    BG_INPUT:      str = "#111a28"   # input fields, button backgrounds

    # Text - crisp, high-contrast engineering hierarchy
    TEXT:      str = "#f1f5f9"   # primary readable text
    TEXT_DIM:  str = "#94a3b8"   # labels, secondary text
    TEXT_MUTED:str = "#475569"   # disabled / placeholder text

    # Accents - MachZone brand and restrained telemetry signals
    CYAN:   str = "#38bdf8"   # primary highlight / telemetry cyan
    GREEN:  str = "#22c55e"   # success / connected / OK
    AMBER:  str = "#f59e0b"   # warning / pending
    RED:    str = "#ff3b30"   # MachZone brand red / error / danger / lost
    ORANGE: str = "#f97316"   # MachZone aerodynamic orange
    PURPLE: str = "#a855f7"   # telemetry acceleration purple
    MAGENTA:str = "#f43f5e"   # telemetry gyro spin rose/magenta
    BLUE:   str = "#3b82f6"   # telemetry pressure blue

    # Structural
    BORDER: str = "#1a2638"   # subtle blue-gray borders / grid lines
    BORDER_LIGHT: str = "#24364f" # lighter borders for hover / active elements

    # Gauge-specific
    GAUGE_BG:   str = "#111a28"   # dial arc background track
    GAUGE_TICK: str = "#475569"   # tick marks on the dial


class ThemeManager:
    """Central access point for the application palette and stylesheet.

    Usage::

        c = ThemeManager.C()
        widget.setStyleSheet(f"color: {c.CYAN};")

    The stylesheet is applied once at application start::

        app.setStyleSheet(ThemeManager.main_stylesheet())
    """

    _palette: Palette = Palette()

    @classmethod
    def C(cls) -> Palette:
        """Return the active colour palette."""
        return cls._palette

    @classmethod
    def sidebar_stylesheet(cls) -> str:
        """Stylesheet for the left navigation sidebar."""
        c = cls._palette
        return f"""
            QWidget {{
                background-color: {c.BG_PANEL};
                border-right: 1px solid {c.BORDER};
            }}
            QPushButton {{
                background-color: transparent;
                border: 1px solid transparent;
                border-radius: 6px;
                color: {c.TEXT_DIM};
                text-align: left;
                padding: 10px 14px;
                font-family: "Segoe UI", -apple-system, Arial, sans-serif;
                font-size: 10pt;
                font-weight: 500;
            }}
            QPushButton:hover {{
                background-color: {c.BG_CARD_HOVER};
                color: {c.TEXT};
                border-color: {c.BORDER};
            }}
            QPushButton:checked {{
                background-color: #0d2238;
                border: 1px solid #0284c7;
                color: #38bdf8;
                font-weight: 600;
            }}
            QPushButton:disabled {{
                color: {c.TEXT_MUTED};
            }}
        """

    @classmethod
    def graph_button_stylesheet(cls) -> str:
        """Stylesheet for the graph controls toolbar (reset, zoom in, zoom out)."""
        c = cls._palette
        return f"""
            QPushButton {{
                background-color: rgba(17, 26, 40, 0.85);
                color: {c.TEXT_DIM};
                border: 1px solid {c.BORDER};
                border-radius: 4px;
                padding: 1px 4px;
                font-family: "Segoe UI", Arial, sans-serif;
                font-size: 11px;
                font-weight: bold;
                min-width: 22px;
                max-width: 22px;
                min-height: 20px;
                max-height: 20px;
            }}
            QPushButton:hover {{
                background-color: {c.BG_CARD_HOVER};
                border-color: {c.CYAN};
                color: {c.CYAN};
            }}
            QPushButton:pressed {{
                background-color: {c.BG_CARD};
            }}
            QPushButton:disabled {{
                color: {c.TEXT_MUTED};
                border-color: {c.BORDER};
            }}
        """

    @classmethod
    def main_stylesheet(cls) -> str:
        """Global Qt stylesheet for the entire application."""
        c = cls._palette
        return f"""
            QMainWindow {{
                background-color: {c.BG};
                color: {c.TEXT};
                font-family: "Segoe UI", -apple-system, Arial, sans-serif;
                font-size: 10pt;
            }}

            QWidget {{
                color: {c.TEXT};
                font-family: "Segoe UI", -apple-system, Arial, sans-serif;
                font-size: 10pt;
            }}

            QScrollArea, QStackedWidget {{
                border: none;
                background-color: transparent;
            }}
            QScrollArea > QWidget > QWidget {{
                background-color: transparent;
            }}

            QScrollBar:vertical {{
                background: {c.BG};
                width: 6px;
                margin: 0;
            }}
            QScrollBar::handle:vertical {{
                background: {c.BORDER};
                border-radius: 3px;
                min-height: 24px;
            }}
            QScrollBar::handle:vertical:hover {{
                background: {c.BORDER_LIGHT};
            }}
            QScrollBar::add-line:vertical,
            QScrollBar::sub-line:vertical {{
                height: 0;
            }}

            QScrollBar:horizontal {{
                background: {c.BG};
                height: 6px;
                margin: 0;
            }}
            QScrollBar::handle:horizontal {{
                background: {c.BORDER};
                border-radius: 3px;
                min-width: 24px;
            }}
            QScrollBar::handle:horizontal:hover {{
                background: {c.BORDER_LIGHT};
            }}
            QScrollBar::add-line:horizontal,
            QScrollBar::sub-line:horizontal {{
                width: 0;
            }}

            QFrame#TelemetryCard {{
                background-color: {c.BG_CARD};
                border: 1px solid {c.BORDER};
                border-radius: 8px;
            }}
            QFrame#TelemetryCard:hover {{
                border-color: {c.BORDER_LIGHT};
            }}

            QTabWidget::pane {{
                border: 1px solid {c.BORDER};
                background-color: {c.BG_CARD};
            }}
            QTabBar::tab {{
                background-color: {c.BG_INPUT};
                color: {c.TEXT_DIM};
                border: 1px solid {c.BORDER};
                padding: 6px 14px;
                margin-right: 2px;
            }}
            QTabBar::tab:selected {{
                background-color: {c.BG_CARD};
                color: {c.CYAN};
                border-bottom: 2px solid {c.CYAN};
            }}
            QTabBar::tab:hover {{
                background-color: {c.BG_CARD_HOVER};
                color: {c.TEXT};
            }}

            QPushButton {{
                background-color: {c.BG_INPUT};
                color: {c.TEXT};
                border: 1px solid {c.BORDER};
                border-radius: 4px;
                padding: 5px 12px;
            }}
            QPushButton:hover {{
                background-color: {c.BG_CARD_HOVER};
                border-color: {c.BORDER_LIGHT};
            }}
            QPushButton:pressed {{
                background-color: {c.BG_CARD};
            }}
            QPushButton:disabled {{
                color: {c.TEXT_MUTED};
                border-color: {c.BG_INPUT};
            }}

            QComboBox {{
                background-color: {c.BG_INPUT};
                color: {c.TEXT};
                border: 1px solid {c.BORDER};
                border-radius: 4px;
                padding: 4px 8px;
            }}
            QComboBox::drop-down {{
                border: none;
            }}
            QComboBox QAbstractItemView {{
                background-color: {c.BG_CARD};
                color: {c.TEXT};
                selection-background-color: {c.BG_CARD_HOVER};
            }}

            QLabel {{
                background: transparent;
            }}

            QFrame[frameShape="4"],
            QFrame[frameShape="5"] {{
                color: {c.BORDER};
            }}

            QProgressBar {{
                background-color: {c.BG_INPUT};
                border: 1px solid {c.BORDER};
                border-radius: 3px;
                text-align: center;
                color: {c.TEXT};
            }}
            QProgressBar::chunk {{
                background-color: {c.CYAN};
                border-radius: 3px;
            }}

            QCheckBox {{
                color: {c.TEXT};
                spacing: 6px;
            }}
            QCheckBox::indicator {{
                width: 14px;
                height: 14px;
                border: 1px solid {c.BORDER};
                border-radius: 3px;
                background: {c.BG_INPUT};
            }}
            QCheckBox::indicator:checked {{
                background: {c.CYAN};
                border-color: {c.CYAN};
            }}

            QSplitter::handle {{
                background: {c.BORDER};
            }}
            QSplitter::handle:horizontal {{
                width: 1px;
            }}
            QSplitter::handle:vertical {{
                height: 1px;
            }}

            QStatusBar {{
                background-color: {c.BG_PANEL};
                color: {c.TEXT_DIM};
                border-top: 1px solid {c.BORDER};
                font-size: 9pt;
            }}

            QMenuBar {{
                background-color: {c.BG_CARD};
                color: {c.TEXT};
                border-bottom: 1px solid {c.BORDER};
            }}
            QMenuBar::item:selected {{
                background-color: {c.BG_CARD_HOVER};
            }}
            QMenu {{
                background-color: {c.BG_CARD};
                color: {c.TEXT};
                border: 1px solid {c.BORDER};
            }}
            QMenu::item:selected {{
                background-color: {c.BG_CARD_HOVER};
            }}

            QToolTip {{
                background-color: {c.BG_CARD};
                color: {c.TEXT};
                border: 1px solid {c.BORDER};
                padding: 4px;
            }}

            QLineEdit {{
                background-color: {c.BG_INPUT};
                color: {c.TEXT};
                border: 1px solid {c.BORDER};
                border-radius: 4px;
                padding: 4px 8px;
                selection-background-color: {c.CYAN};
                selection-color: {c.BG};
            }}
            QLineEdit:focus {{
                border-color: {c.CYAN};
            }}

            QTextEdit, QPlainTextEdit {{
                background-color: {c.BG_INPUT};
                color: {c.TEXT};
                border: 1px solid {c.BORDER};
                border-radius: 4px;
                selection-background-color: {c.CYAN};
                selection-color: {c.BG};
            }}
            QTextEdit:focus, QPlainTextEdit:focus {{
                border-color: {c.CYAN};
            }}
        """
