import collections
import pathlib
import subprocess
import sys
import threading
import traceback

import pygubu
import tkinter as tk

PROJECT_PATH = pathlib.Path(__file__).parent
REPO_ROOT = PROJECT_PATH.parent

# fr20_ceiling_gazebo/fr20_kinematics/robot_program/bending_head_motor liegen im
# Workspace-Root, nicht im gui/-Ordner - für Imports (hier und in den Mixin-Modulen,
# siehe unten) bekannt machen, BEVOR diese importiert werden.
sys.path.insert(0, str(REPO_ROOT))
# gui/ selbst ist bereits in sys.path (Skriptverzeichnis) - constants.py & die
# *_mixin.py-Module liegen dort und werden daher als einfache Top-Level-Module importiert.

from constants import PROJECT_UI, JOG_STEP_M, FONT_SIZE_OPTIONS, ZOOM_OPTIONS
from style_mixin import StyleMixin
from jog_mixin import JogMixin
from program_mixin import ProgramMixin
from bending_mixin import BendingMixin
from options_mixin import OptionsMixin
from log_mixin import LogMixin, _StdoutTee

import fr20_ceiling_gazebo as sim  # noqa: E402
import bending_head_motor as bhm  # noqa: E402


class MyApp(StyleMixin, JogMixin, ProgramMixin, BendingMixin, OptionsMixin, LogMixin):
    """Hauptfenster-Controller. Fachlogik ist auf Mixins verteilt (siehe *_mixin.py in
    diesem Ordner) - `MyApp` bleibt trotzdem EINE Klasseninstanz, damit pygubus
    `connect_callbacks(self)` alle im .ui-Designer benannten Callback-Methoden per
    Introspektion auf `self` findet, egal in welchem Mixin sie stecken."""

    def __init__(self, master=None):
        # Muss vor dem ersten _apply_style()-Aufruf gesetzt sein, da dieser sie liest.
        self._jog_step_m = JOG_STEP_M
        self._jog_rotation_step_deg = 5.0
        self._bending_jog_step_size = 5.0
        self._font_size = FONT_SIZE_OPTIONS["Mittel"]
        self._dark_mode = False
        self._zoom = ZOOM_OPTIONS["100%"]

        # Ringpuffer für die Log-Anzeige - begrenzt, damit ein lange laufendes Programm nicht
        # unbegrenzt Speicher frisst. _log_text_widget existiert erst nach dem ersten Öffnen.
        self._log_buffer: collections.deque[str] = collections.deque(maxlen=2000)
        self._log_text_widget: tk.Text | None = None
        sys.stdout = _StdoutTee(sys.stdout, self._on_log_line)

        self.builder = builder = pygubu.Builder()
        builder.add_resource_path(PROJECT_PATH)
        builder.add_from_file(PROJECT_UI)
        self.mainwindow = builder.get_object('mainwindow', master)
        self._init_display()
        self._apply_style()
        builder.connect_callbacks(self)
        self._images = {}

        # Buttons in window_testen/window_programm/... werden erst bei deren erstem Öffnen
        # gestylt (pygubu erzeugt sie erst dann).
        self._style_widget("label_title", "Title.TLabel")
        for label_id in ("label_figur1_caption", "label_testen_caption", "label_tischecken_caption", "label_platzhalter_caption"):
            self._style_widget(label_id, "Header.TLabel")
        for button_id in ("button_figur1", "button_testen", "button_tischecken", "button_platzhalter"):
            self._style_widget(button_id, "Card.TButton")
        self._style_widget("button_beenden", "Danger.TButton")
        self._style_widget("button_optionen", "Warning.TButton")

        # Kachel-Bilder liegen in voller Auflösung (270x420px) vor - verkleinert einsetzen,
        # sonst passt das Hauptfenster auf kleineren Bildschirmen nicht mehr komplett drauf.
        # Auf flachen Touchscreens stehen die Kacheln nebeneinander und werden so groß wie
        # die Bildschirmhöhe erlaubt.
        tile_h = self._apply_compact_layout_main() if self._compact else 233
        tile_size = (round(tile_h * 270 / 420), tile_h)
        self.set_image("button_figur1", "Form1r.png", size=tile_size)
        self.set_image("button_testen", "platzhalter_neu.png", size=tile_size)
        self.set_image("button_tischecken", "platzhalter_neu.png", size=tile_size)
        self.set_image("button_platzhalter", "platzhalter_neu.png", size=tile_size)

        self._make_responsive(self.mainwindow, self.builder.get_object("frame_main"))
        self._fit_window_to_screen(self.mainwindow)

        # Zustand der Gazebo-Simulation/Programmausführung.
        self._gz_proc: subprocess.Popen | None = None
        self._publishers: dict | None = None
        self._program_thread: threading.Thread | None = None
        self._stop_event: threading.Event | None = None
        # Verhindert doppelten gz-sim-Start durch Auto-Start (run()) + gleichzeitigen Button-Klick.
        self._sim_lock = threading.Lock()

        # Eigener Thread für den Biegekopf-Testlauf (nicht self._program_thread) - der
        # Biegekopf-Motor ist ein anderer Antrieb als der Arm und darf parallel dazu laufen.
        self._bending_thread: threading.Thread | None = None
        self._bending_stop_event: threading.Event | None = None
        self._bending_pause_event = threading.Event()
        self._bending_axis: "bhm.BendingHeadAxis | None" = None
        self._bending_current_position = 0.0
        self._bending_pose_lock = threading.Lock()
        self._bending_jog_after_id = None

        # Zuletzt gesendete Gelenkpose - Ausgangspunkt für den nächsten Jog-Schritt.
        # _pose_lock schützt gegen gleichzeitigen Zugriff vom Programm-Worker-Thread und Jog-Klicks.
        self._current_joints: dict | None = None
        self._pose_lock = threading.Lock()

        # Gesetzt = Programm pausiert - Jog-Buttons dürfen dann bewegen.
        self._pause_event = threading.Event()

        # Sanfter Abbruch: überspringt restliche Punkte, fährt aber noch zum Nullpunkt -
        # anders als self._stop_event (harter, sofortiger Stopp beim Ablösen durch ein anderes Programm).
        self._abort_event = threading.Event()

        # Entwicklermodus + Einzelschrittbetrieb: gesetzt = Worker wartet vor jedem Punkt auf
        # self._step_event statt direkt weiterzufahren.
        self._dev_mode = False
        self._single_step_var = tk.BooleanVar(value=False)
        self._step_event = threading.Event()

        self._jog_after_id = None

        # Läuft dauerhaft, aktualisiert aber nur Labels in bereits geöffneten Fenstern.
        self.mainwindow.after(300, self._update_position_readout)

    def on_beenden_pressed(self):
        """Button 'Beenden': dieselbe Aufräum-Logik wie beim Schließen über das Fenster-X."""
        self.on_close()

    def on_close(self):
        # zentrale Aufräum-Routine: Jog stoppen, Programm abbrechen, Gazebo beenden, Fenster schließen
        self._release_jog()
        self._release_bending_jog()
        self._stop_running_program()
        if self._bending_thread is not None and self._bending_thread.is_alive():
            self._bending_pause_event.clear()
            if self._bending_stop_event is not None:
                self._bending_stop_event.set()
            self._bending_thread.join(timeout=5.0)
        if self._gz_proc is not None:
            sim.shutdown_simulation(self._gz_proc)
        self.mainwindow.destroy()

    def run(self):
        self.mainwindow.protocol("WM_DELETE_WINDOW", self.on_close)

        # Kalibrierung zuerst zeigen - Hauptfenster bleibt verborgen, bis 'Weiter' dort
        # gedrückt wird, damit kein Programm ohne vorherige Kalibrierungsmöglichkeit startet.
        self._open_calibration_window()
        self.window_kalibrierung.protocol("WM_DELETE_WINDOW", self.on_close)

        # Gazebo direkt mit der GUI starten (Hintergrund-Thread, damit das Fenster nicht
        # einfriert) statt erst beim ersten Button-Klick; Fehler deutlich loggen statt den
        # Thread lautlos sterben zu lassen.
        def _auto_start():
            try:
                self._ensure_simulation_running()
            except Exception:
                print("Automatischer Gazebo-Start beim Öffnen ist fehlgeschlagen:")
                traceback.print_exc()

        threading.Thread(target=_auto_start, daemon=True).start()

        self.mainwindow.mainloop()

if __name__ == '__main__':
    app = MyApp()
    app.run()
