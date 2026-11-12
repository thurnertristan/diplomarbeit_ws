"""Zentrale Konstanten für die GUI - eigenes Modul, damit sowohl my_gui.py als auch die
Mixin-Module (style_mixin.py, jog_mixin.py, ...) darauf zugreifen können, ohne my_gui.py
selbst zu importieren (das würde einen Zirkelimport erzeugen)."""

import os
import pathlib

PROJECT_PATH = pathlib.Path(__file__).parent
PROJECT_UI = PROJECT_PATH / "my_gui.ui"
REPO_ROOT = PROJECT_PATH.parent

PROGRAM_GELENKPOSE = REPO_ROOT / "programs" / "beispiel_gelenkpose.json"
PROGRAM_TISCH = REPO_ROOT / "programs" / "beispiel_tisch.json"
PROGRAM_FIGUR1 = REPO_ROOT / "programs" / "beispiel_figur1.json"
PROGRAM_ECKEN = REPO_ROOT / "programs" / "beispiel_tisch_ecken.json"

JOG_STEP_M = 0.05
JOG_REPEAT_MS = 150

# Anzeigetext -> tatsächlicher Wert, für die Optionen-Comboboxen.
JOG_STEP_OPTIONS = {"1 mm": 0.001, "1 cm": 0.01, "5 cm": 0.05, "10 cm": 0.10}
BENDING_JOG_STEP_OPTIONS = {"1°": 1.0, "5°": 5.0, "10°": 10.0, "20°": 20.0}
FONT_SIZE_OPTIONS = {"Klein": 9, "Mittel": 11, "Groß": 14}
ZOOM_OPTIONS = {"100%": 1.0, "125%": 1.25, "150%": 1.5}
BASE_TK_SCALING = 1.3333  # Tcl/Tk-Basisskalierung bei 100% (96dpi) - ZOOM_OPTIONS multipliziert hierauf.

# Touchscreen-/Vollbildbetrieb (siehe display.py und StyleMixin._fit_window_to_screen):
# Alle Fenster laufen randlos im Vollbild auf dem Ziel-Monitor. Zum Entwickeln am normalen
# Monitor abschaltbar mit:  FR20_KIOSK=0 ./start_gui.sh
KIOSK_MODE = os.environ.get("FR20_KIOSK", "1") != "0"
# Name des Touchscreens laut `xrandr` (z.B. "HDMI-1"); leer = automatisch den kleinsten
# angeschlossenen Monitor nehmen. Überschreibbar mit:  FR20_MONITOR=HDMI-1 ./start_gui.sh
KIOSK_MONITOR = os.environ.get("FR20_MONITOR", "")
# Ab dieser Bildschirmhöhe (und darunter) werden Hauptfenster/Optionen in die Breite statt
# in die Höhe angeordnet - 10"-Touchscreens sind typischerweise 1280x800 oder 1024x600.
COMPACT_LAYOUT_MAX_HEIGHT = 900

# Firmenlogo unten rechts in jedem Fenster (siehe StyleMixin._add_logo).
LOGO_FILE = "dry_expert_logo.png"

# Kein echter Sicherheitsmechanismus - nur Schutz vor versehentlichem Aktivieren des Entwicklermodus.
DEV_MODE_PASSWORD = "bensen"

# Jog-Kreuz-Buttons (Achse, Vorzeichen, Pfeilsymbol) - identisch für alle drei Jog-Fenster
# (Kalibrierung/Programm/Manuell), siehe JogMixin._bind_jog_pad.
JOG_PAD_BUTTONS = (
    ("x", 1, "▲"), ("x", -1, "▼"),
    ("y", 1, "▶"), ("y", -1, "◀"),
    ("z", 1, "▲"), ("z", -1, "▼"),
    ("rot", 1, "⟳"), ("rot", -1, "⟲"),
)
