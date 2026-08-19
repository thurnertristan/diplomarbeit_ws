import collections         # begrenzter Ringpuffer für die Log-Anzeige
import math                 # Grad/Radiant für Gelenkwinkel-Anzeige und Werkzeug-Rotation
import pathlib            # Dateipfade
import subprocess         # Gazebo-Prozess
import sys                # sys.path anpassen für Imports aus dem Workspace-Root
import threading          # Hintergrund-Threads, Events, Lock
import traceback          # Fehler aus Hintergrund-Threads ausgeben

import pygubu              # lädt die .ui-Oberfläche
import tkinter as tk
from tkinter import messagebox, simpledialog, ttk   # Dialoge und gestylte Widgets
from PIL import Image, ImageTk         # Bilder laden/skalieren

PROJECT_PATH = pathlib.Path(__file__).parent
PROJECT_UI = PROJECT_PATH / "my_gui.ui"
REPO_ROOT = PROJECT_PATH.parent

# fr20_ceiling_gazebo/fr20_kinematics/robot_program liegen im Workspace-Root,
# nicht im gui/-Ordner - für den Import hier bekannt machen.
sys.path.insert(0, str(REPO_ROOT))

import fr20_ceiling_gazebo as sim  # noqa: E402  (Import erst nach sys.path-Anpassung möglich)
import fr20_kinematics as kin  # noqa: E402
import robot_program as rp  # noqa: E402

PROGRAM_GELENKPOSE = REPO_ROOT / "programs" / "beispiel_gelenkpose.json"
PROGRAM_TISCH = REPO_ROOT / "programs" / "beispiel_tisch.json"
PROGRAM_FIGUR1 = REPO_ROOT / "programs" / "beispiel_figur1.json"
PROGRAM_ECKEN = REPO_ROOT / "programs" / "beispiel_tisch_ecken.json"

# Schrittweite (Meter) pro Jog-Schritt (X+/X-/Y+/Y-) und Intervall (ms)
# zwischen den Schritten, solange der Button gehalten wird - Startwert für
# den in den Optionen einstellbaren self._jog_step_m (siehe combo_jogstep).
JOG_STEP_M = 0.05
JOG_REPEAT_MS = 150

# Auswahlwerte für die Optionen-Comboboxen (siehe on_optionen_pressed) -
# Anzeigetext -> tatsächlicher Wert. dict statt Liste, damit combo.get()
# direkt als Schlüssel für den passenden Wert dient.
JOG_STEP_OPTIONS = {"1 mm": 0.001, "1 cm": 0.01, "5 cm": 0.05, "10 cm": 0.10}
FONT_SIZE_OPTIONS = {"Klein": 9, "Mittel": 11, "Groß": 14}
ZOOM_OPTIONS = {"100%": 1.0, "125%": 1.25, "150%": 1.5}
# Tcl/Tk-Basis-Skalierung bei 100% (typischer Wert für 96dpi-Bildschirme) -
# ZOOM_OPTIONS-Faktoren werden hierauf multipliziert (siehe _on_zoom_selected).
BASE_TK_SCALING = 1.3333

# Passwort für den Entwicklermodus (siehe on_entwicklermodus_toggle_pressed) -
# kein echter Sicherheitsmechanismus, nur ein Schutz vor versehentlichem
# Aktivieren (Einzelschrittbetrieb u.ä. sind nichts für den Normalbetrieb).
DEV_MODE_PASSWORD = "bensen"


class _StdoutTee:
    """Leitet print()/stdout zusätzlich an die Log-Anzeige weiter (siehe
    on_log_pressed), ohne die normale Terminal-Ausgabe zu verlieren -
    Programme laufen in Hintergrund-Threads (siehe _run_program_async), die
    Callback-Funktion muss daher selbst threadsicher sein (schreibt nur in
    eine deque und plant ein after() auf dem Tkinter-Hauptthread ein, siehe
    MyApp._on_log_line - derselbe Cross-Thread-after()-Kniff wie beim
    bestehenden `on_done`-Callback in _run_program_async)."""

    def __init__(self, original, callback):
        self._original = original
        self._callback = callback

    def write(self, text: str) -> int:
        self._original.write(text)
        self._callback(text)
        return len(text)

    def flush(self) -> None:
        self._original.flush()


class MyApp:
    def __init__(self, master=None):
        # Einstellungen (siehe Optionen-Fenster) - MÜSSEN vor dem ersten
        # _apply_style()-Aufruf gesetzt sein, da dieser sie liest (siehe
        # _palette()/_apply_style unten).
        self._jog_step_m = JOG_STEP_M
        self._jog_rotation_step_deg = 5.0
        self._font_size = FONT_SIZE_OPTIONS["Mittel"]
        self._dark_mode = False
        self._zoom = ZOOM_OPTIONS["100%"]

        # Ringpuffer für die Log-Anzeige (siehe on_log_pressed) - begrenzt,
        # damit ein lange laufendes Programm nicht unbegrenzt Speicher
        # frisst. self._log_text_widget existiert erst, wenn das
        # Log-Fenster mind. einmal geöffnet wurde (siehe _on_log_line).
        self._log_buffer: collections.deque[str] = collections.deque(maxlen=2000)
        self._log_text_widget: tk.Text | None = None
        sys.stdout = _StdoutTee(sys.stdout, self._on_log_line)

        self.builder = builder = pygubu.Builder()
        builder.add_resource_path(PROJECT_PATH)
        builder.add_from_file(PROJECT_UI)
        self.mainwindow = builder.get_object('mainwindow', master)
        self._apply_style()
        builder.connect_callbacks(self)
        self._images = {}

        # Kachel-Überschriften ("Form Körper N") und der "Beenden"-Knopf
        # gehören zu mainwindow und sind damit schon jetzt gebaut - Buttons
        # in window_testen/window_programm werden erst bei deren erstem
        # Öffnen gestylt (siehe on_testen_pressed/on_figur1_pressed), da
        # pygubu sie vorher noch nicht erzeugt hat.
        self._style_widget("label_title", "Title.TLabel")
        for label_id in ("label_figur1_caption", "label_testen_caption", "label_tischecken_caption", "label_platzhalter_caption"):
            self._style_widget(label_id, "Header.TLabel")
        for button_id in ("button_figur1", "button_testen", "button_tischecken", "button_platzhalter"):
            self._style_widget(button_id, "Card.TButton")
        self._style_widget("button_beenden", "Danger.TButton")
        self._style_widget("button_optionen", "Warning.TButton")

        # Die Kachel-Bilder liegen in voller Auflösung vor (270x420 px) - zwei
        # Reihen davon plus Labels/Buttons sind zusammen deutlich höher als
        # 768px und lassen "Kalibrierung"/"Beenden" auf kleineren Bildschirmen
        # unten aus dem Fenster fallen. Deshalb hier verkleinert einsetzen,
        # damit das Hauptfenster komplett auf den Bildschirm passt.
        self.set_image("button_figur1", "Form1r.png", size=(150, 233))
        self.set_image("button_testen", "platzhalter_neu.png", size=(150, 233))
        self.set_image("button_tischecken", "platzhalter_neu.png", size=(150, 233))
        self.set_image("button_platzhalter", "platzhalter_neu.png", size=(150, 233))

        # Inhalt bleibt beim Vergrößern/Verkleinern als kompakter Block
        # mittig im Fenster stehen, statt sich zu verzerren (siehe
        # _make_responsive).
        self._make_responsive(self.mainwindow, self.builder.get_object("frame_main"))
        self._fit_window_to_screen(self.mainwindow)

        # Zustand der Gazebo-Simulation/Programmausführung, siehe
        # _ensure_simulation_running() und _run_program_async().
        self._gz_proc: subprocess.Popen | None = None
        self._publishers: dict | None = None
        self._program_thread: threading.Thread | None = None
        self._stop_event: threading.Event | None = None
        # Verhindert, dass der Auto-Start beim Öffnen der GUI (siehe run())
        # und ein gleichzeitiger Button-Klick gz-sim doppelt starten.
        self._sim_lock = threading.Lock()

        # Zuletzt gesendete Gelenkpose (Gelenkwinkel in rad) - wird von den
        # Jog-Buttons (X+/X-/Y+/Y-) als Ausgangspunkt für den nächsten
        # kleinen Schritt gebraucht. Wird beim Erreichen von
        # SAFE_STAGING_POSE und nach jedem Programm/Jog-Schritt
        # aktualisiert. _pose_lock schützt das gegen gleichzeitigen Zugriff
        # vom Programm-Worker-Thread und einem Jog-Klick auf dem Haupt-Thread.
        self._current_joints: dict | None = None
        self._pose_lock = threading.Lock()

        # Gesetzt = Programmausführung pausiert (siehe on_pause_pressed/
        # on_weiter_pressed sowie robot_program.run_program's pause_event).
        # Während der Pause dürfen
        # die Jog-Buttons den Roboter bewegen, ohne dass das Programm
        # gleichzeitig weiterläuft und sich mit ihnen in die Quere kommt.
        self._pause_event = threading.Event()

        # Gesetzt = sanfter Abbruch (siehe on_abbruch_pressed sowie
        # robot_program.run_program's abort_event): anders als
        # self._stop_event (harter, sofortiger Stopp z.B. beim Ablösen
        # durch ein anderes Programm oder beim Schließen der GUI, siehe
        # _stop_running_program) überspringt das nur die restlichen
        # Programmpunkte und lässt den Roboter zuerst noch zum Nullpunkt
        # fahren, bevor das Programm tatsächlich endet.
        self._abort_event = threading.Event()

        # Entwicklermodus (siehe on_entwicklermodus_toggle_pressed, Passwort
        # DEV_MODE_PASSWORD) + Einzelschrittbetrieb darin (siehe
        # on_einzelschritt_toggled/on_naechster_punkt_pressed). Gesetzt =
        # der laufende Programm-Worker wartet vor jedem Punkt auf
        # self._step_event (siehe robot_program.run_program's step_event)
        # statt direkt weiterzufahren - anders als _pause_event (komplett
        # anhalten) hier "genau einen Punkt weiterfahren und wieder warten".
        self._dev_mode = False
        self._single_step_var = tk.BooleanVar(value=False)
        self._step_event = threading.Event()

        # after()-Job-ID der laufenden Jog-Wiederholung (siehe _hold_jog/
        # _release_jog) - None, solange kein Jog-Button gehalten wird.
        self._jog_after_id = None

        # Live-Positionsanzeige (siehe _update_position_readout): läuft
        # dauerhaft im Hintergrund, aktualisiert aber nur Labels in bereits
        # geöffneten Fenstern (window_programm/_kalibrierung/_manuell) -
        # kostet im Ruhezustand (Fenster noch nicht geöffnet) praktisch
        # nichts, daher ein einziger zentraler Loop statt einem pro Fenster.
        self.mainwindow.after(300, self._update_position_readout)

    def set_image(self, widget_id, filename, size=(512, 512)):
        """Lädt ein Bild, skaliert es und setzt es auf ein Widget."""
        img = Image.open(PROJECT_PATH / filename).resize(size)
        photo = ImageTk.PhotoImage(img)
        self._images[widget_id] = photo                      # Referenz behalten!
        self.builder.get_object(widget_id).configure(image=photo)

    def _palette(self) -> dict[str, str]:
        """Farbpalette abhängig von self._dark_mode (siehe
        on_darkmode_toggle_pressed) - zentral an einer Stelle, damit
        _apply_style() und _refresh_toplevel_backgrounds() (für die
        Toplevel-Fenster selbst, die ihren Hintergrund per .configure()
        statt per ttk-Style bekommen) immer dieselben Farben verwenden."""
        if self._dark_mode:
            # Deutlich dunklerer, satterer Hintergrund als vorher (war
            # #1b1f27/#262c38 - Fenster und Karten lagen farblich zu nah
            # beieinander, wirkte flach) plus kräftigere, leicht
            # aufgehellte Akzent-/Statusfarben statt der 1:1 aus dem hellen
            # Design übernommenen (die auf dunklem Grund matt/schmutzig
            # wirkten) - jede Statusfarbe (Erfolg/Warnung/Gefahr/...) ist
            # bewusst eigens fürs dunkle Design abgestimmt, nicht nur
            # abgedunkelt.
            return {
                "bg": "#11141b",
                "card_bg": "#1c212c",
                "text": "#f1f3f9",
                "muted": "#9aa4b8",
                "accent": "#7c8bff",
                "accent_active": "#98a5ff",
                "border": "#333c4d",
                "jog_hover": "#2a3140",
                "success": "#34d399",
                "success_active": "#5fe3b2",
                "warning": "#fbbf24",
                "warning_active": "#fdd35a",
                "danger": "#f87171",
                "danger_active": "#fb9d9d",
                "secondary": "#5c6577",
                "secondary_active": "#707b90",
                "jog": "#3d4658",
                "jog_active": "#4b5670",
            }
        return {
            "bg": "#eef1f6",
            "card_bg": "#ffffff",
            "text": "#1b1f27",
            "muted": "#5b6472",
            "accent": "#3654d1",
            "accent_active": "#2842a8",
            "border": "#d7dbe3",
            "jog_hover": "#e7ecf7",
            "success": "#2f9e57",
            "success_active": "#22803f",
            "warning": "#d98c00",
            "warning_active": "#b57400",
            "danger": "#d9453f",
            "danger_active": "#b7332e",
            "secondary": "#8a93a0",
            "secondary_active": "#6f7784",
            "jog": "#586470",
            "jog_active": "#414a54",
        }

    def _apply_style(self) -> None:
        """Einheitliches, modernes Erscheinungsbild für die ganze GUI -
        bewusst hier in Python statt in der .ui-Datei: pygubu-Designer
        schreibt die .ui beim Speichern komplett neu und würde reine
        XML-Anpassungen sonst wieder verwerfen (mehrfach passiert), Styles
        aus Python bleiben davon unberührt.

        Wiederholt aufrufbar (siehe on_darkmode_toggle_pressed/
        _on_schriftgroesse_selected) - ttk-Styles aktualisieren beim erneuten
        style.configure() automatisch ALLE Widgets, die diesen Style
        verwenden, ganz ohne die Widgets selbst neu zu bauen. Nur die
        Toplevel-Fenster selbst (die ihren Hintergrund per .configure()
        statt per ttk-Style bekommen) müssen zusätzlich über
        _refresh_toplevel_backgrounds() aktualisiert werden.
        """
        style = ttk.Style(self.mainwindow)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass  # "clam" nicht verfügbar - beim Standard-Theme bleiben

        p = self._palette()
        bg, card_bg, text, muted, accent = p["bg"], p["card_bg"], p["text"], p["muted"], p["accent"]
        # Grundschriftgröße kommt aus den Optionen (siehe
        # _on_schriftgroesse_selected) - alle anderen Schriftgrößen unten
        # skalieren proportional dazu mit, statt fest zu bleiben.
        fs = self._font_size
        base_font = ("Helvetica", fs)

        self.mainwindow.configure(background=bg)
        style.configure("TFrame", background=bg)
        style.configure("TLabel", background=bg, foreground=text, font=base_font)
        style.configure("Header.TLabel", background=bg, foreground=muted, font=("Helvetica", fs + 1, "bold"))
        style.configure(
            "Title.TLabel", background=bg, foreground=accent, font=("Helvetica", fs + 9, "bold")
        )
        style.configure("TButton", font=base_font, padding=(14, 10))
        # Bildkacheln bekommen einen dezenten weißen "Karten"-Rahmen statt der
        # sonst unsichtbaren Standard-Button-Fläche - wirkt wie eine Karte,
        # auf der das Vorschaubild liegt.
        style.configure(
            "Card.TButton",
            background=card_bg,
            bordercolor=p["border"],
            relief="flat",
            padding=6,
        )
        style.map("Card.TButton", background=[("active", p["jog_hover"])])

        # Gruppierungs-Rahmen (ttk.Labelframe) für Jog-Pads und Programm-Karten
        # in window_testen/window_programm/window_kalibrierung - macht
        # zusammengehörige Buttons als eine sichtbare
        # Einheit erkennbar statt einer losen Button-Wolke. Titelzeile des
        # Labelframe braucht einen eigenen "<Style>.Label"-Sub-Style, sonst
        # bleibt sie beim Standard-Look (schwarz auf grau).
        style.configure(
            "Card.TLabelframe",
            background=card_bg,
            bordercolor=p["border"],
            relief="flat",
            padding=14,
        )
        style.configure(
            "Card.TLabelframe.Label",
            background=card_bg,
            foreground=muted,
            font=("Helvetica", fs, "bold"),
        )
        # Innerer Container für die ein-/ausklappbaren Jog-Pfeile (siehe
        # _bind_jog_toggle) - eigener Frame-Style, da ein normaler ttk.Frame
        # sonst den allgemeinen (grauen) Fensterhintergrund statt des
        # weißen Karten-Hintergrunds der umgebenden Card.TLabelframe zeigt.
        style.configure("Card.TFrame", background=card_bg)
        # Klickbare "Kopfzeile" oberhalb der Jog-Pfeile, blendet
        # frame_jog_controls_* ein/aus (siehe _bind_jog_toggle) - bewusst
        # unauffällig/flach wie ein Abschnittstitel, kein Action-Button.
        style.configure(
            "JogToggle.TButton",
            background=card_bg,
            foreground=muted,
            font=("Helvetica", fs, "bold"),
            anchor="w",
            relief="flat",
            padding=(4, 6),
        )
        style.map("JogToggle.TButton", background=[("active", p["jog_hover"])])
        # Live-Positionsanzeige (siehe _update_position_readout) - kleine,
        # monospace-artige Statuszeile unter dem Jog-Pad.
        style.configure(
            "Readout.TLabel", background=card_bg, foreground=muted, font=("Courier", fs - 1)
        )

        def colored_button(name: str, color: str, active: str, fg: str = "white") -> None:
            # legt einen benannten Button-Style mit Grundfarbe + Hover-Farbe an
            style.configure(name, background=color, foreground=fg, font=base_font, padding=(14, 10))
            style.map(name, background=[("active", active), ("disabled", "#c8ccd2")])

        # Button-Farben je nach Bedeutung (Start/Weiter/Pause/Abbruch/Jog) -
        # aus der Palette statt fest verdrahtet, damit sie im Dark Mode
        # eigens abgestimmte (nicht nur unverändert aus dem hellen Design
        # übernommene, dort auf dunklem Grund matt wirkende) Farben zeigen.
        # Die hellen Status-Akzentfarben (success/warning) sind im Dark Mode
        # kräftig-hell genug, dass dunkler statt weißer Buttontext lesbarer
        # ist (weißer Text auf z.B. hellem #fbbf24 hätte zu wenig Kontrast).
        dark_fg = "#12151c"
        colored_button("Primary.TButton", accent, p["accent_active"])
        colored_button("Success.TButton", p["success"], p["success_active"], fg=dark_fg if self._dark_mode else "white")
        colored_button("Warning.TButton", p["warning"], p["warning_active"], fg=dark_fg if self._dark_mode else "white")
        colored_button("Danger.TButton", p["danger"], p["danger_active"])
        colored_button("Secondary.TButton", p["secondary"], p["secondary_active"])
        colored_button("Jog.TButton", p["jog"], p["jog_active"])
        # Jog-Buttons bekommen eine einheitliche quadratische Mindestgröße
        # und ein großes reines Pfeilsymbol (▲▼◀▶, siehe _bind_jog_hold-
        # Aufrufe) - vorher standen dort Buchstabe+Pfeil-Mischungen (z.B.
        # "X ▸"), die an ihrer Position im Kreuz nicht zur Pfeilrichtung
        # passten (X+ oben zeigte einen Rechtspfeil). Jetzt zeigt jeder
        # Button unmissverständlich in seine eigene Fahrtrichtung, die
        # Achsen-Beschriftung steht separat als Kopfzeile über dem Kreuz.
        style.configure("Jog.TButton", width=3, font=("Helvetica", fs + 5, "bold"), padding=(4, 10))
        # Achsen-Kopfzeile ("Ebene (X / Y)"/"Höhe (Z)"/"Drehung") über den
        # Pads sowie der Mittelpunkt-Marker "•" der XY-Kreuzmitte - beide
        # rein informativ/nicht interaktiv, daher dezent statt wie ein Button.
        style.configure("JogAxis.TLabel", background=card_bg, foreground=muted, font=("Helvetica", fs - 1, "bold"))
        style.configure("JogCenter.TLabel", background=card_bg, foreground=p["border"], font=("Helvetica", fs + 3))

    def _refresh_toplevel_backgrounds(self) -> None:
        """Aktualisiert den Fensterhintergrund aller bereits gebauten
        Toplevel-Fenster nach einem Dark-Mode-Wechsel (siehe
        on_darkmode_toggle_pressed) - ttk-Styles (siehe _apply_style)
        aktualisieren sich zwar von selbst, die Toplevel-Fenster selbst
        haben ihren Hintergrund aber jeweils per direktem .configure(
        background=...) gesetzt (siehe on_testen_pressed etc.), das muss
        hier separat nachgezogen werden."""
        bg = self._palette()["bg"]
        for attr in (
            "mainwindow", "window_testen", "window_programm",
            "window_kalibrierung", "window_optionen", "window_manuell", "window_log",
        ):
            window = getattr(self, attr, None)
            if window is not None:
                window.configure(background=bg)

    def _make_responsive(self, window: tk.Toplevel, container: ttk.Frame) -> None:
        """Hält den Inhalt eines Fensters beim Verkleinern/Vergrößern
        zentriert, statt ihn zu verzerren.

        Frühere Variante gab den Grid-Zellen von `container` selbst Gewicht
        (columnconfigure/rowconfigure(weight=1)) - dadurch wuchsen Buttons/
        Kacheln ungleichmäßig auseinander und das Layout wirkte beim
        Maximieren zerrissen statt weiterhin wie eine kompakte, mittige
        Karte. Jetzt bekommt nur `window`s eigene Zelle Gewicht, `container`
        selbst bleibt in seiner natürlichen Größe (siehe .ui: OHNE
        sticky='nsew') und wird von Tkinter dadurch automatisch in der
        Fenstermitte zentriert - überschüssiger Platz sammelt sich
        gleichmäßig als Rand rundherum statt den Inhalt zu strecken."""
        window.columnconfigure(0, weight=1)
        window.rowconfigure(0, weight=1)

    def _fit_window_to_screen(self, window: tk.Toplevel, margin: int = 80) -> None:
        """Verkleinert und zentriert ein Fenster, falls es größer als der
        verfügbare Bildschirmplatz ist - verhindert, dass Buttons am unteren
        Rand (z.B. 'Beenden') auf kleineren Bildschirmen unsichtbar werden."""
        window.update_idletasks()
        req_w, req_h = window.winfo_reqwidth(), window.winfo_reqheight()
        screen_w, screen_h = window.winfo_screenwidth(), window.winfo_screenheight()
        w = min(req_w, screen_w - margin)
        h = min(req_h, screen_h - margin)
        x = max(0, (screen_w - w) // 2)
        y = max(0, (screen_h - h) // 2)
        window.geometry(f"{w}x{h}+{x}+{y}")

    def _style_widget(self, widget_id: str, style_name: str | None = None, text: str | None = None) -> None:
        """Setzt Style und/oder Text eines schon gebauten Widgets - kleiner
        Helfer, um Wiederholung bei den vielen Button-Anpassungen zu sparen."""
        widget = self.builder.get_object(widget_id)
        if style_name is not None:
            widget.configure(style=style_name)
        if text is not None:
            widget.configure(text=text)

    def _ensure_simulation_running(self) -> dict:
        """Startet die Gazebo-Simulation einmalig (siehe run(): passiert bereits
        automatisch beim Öffnen der GUI, nicht erst beim ersten Button-Klick).

        Läuft bereits eine Simulation, wird diese wiederverwendet, statt ein
        zweites gz-sim-Fenster zu öffnen. Gibt die gz-transport-Publisher pro
        Gelenk zurück. Durch _sim_lock gegen gleichzeitigen Aufruf (Auto-Start
        beim Öffnen + direkter Button-Klick) abgesichert.
        """
        with self._sim_lock:
            if self._gz_proc is not None and self._gz_proc.poll() is None and self._publishers is not None:
                return self._publishers

            if self._gz_proc is not None and self._gz_proc.poll() is None:
                # gz-sim läuft noch, aber self._publishers wurde nie gesetzt -
                # ein vorheriger Versuch (z.B. der Auto-Start beim Öffnen der
                # GUI) ist zwischen Prozessstart und Publisher-Aufbau
                # gescheitert (z.B. Timeout in sim.wait_for_world auf einem
                # langsameren Rechner). Ohne diesen Check würde hier "None"
                # als Publisher zurückgegeben und ein Programm später mit
                # "'NoneType' object is not subscriptable" abstürzen. Statt
                # mit dem kaputten Rest weiterzumachen: alten Prozess sauber
                # beenden und neu aufsetzen.
                print("Vorheriger Gazebo-Start war unvollständig, versuche es erneut ...")
                sim.shutdown_simulation(self._gz_proc)
                self._gz_proc = None
                self._publishers = None

            print("Starte Gazebo (FR20, transparente Wände) ...")
            self._gz_proc, self._publishers = sim.launch_simulation()

            # Roboter spawnt physikalisch in der Null-Stellung - zunächst die
            # kollisionsfreie Zwischenpose anfahren, bevor irgendein Button
            # ein Programm/Zielpunkt sendet (siehe SAFE_STAGING_POSE in
            # fr20_ceiling_gazebo.py: ein direkter Sprung Null -> Zielpunkt
            # würde mitten durch den Tisch schwingen).
            sim.send_to_safe_staging_pose(self._publishers)
            with self._pose_lock:
                self._current_joints = dict(sim.SAFE_STAGING_POSE)

            print("Gazebo bereit.")
            return self._publishers

    def _stop_running_program(self, graceful: bool = False) -> bool:
        """Bricht ein evtl. gerade laufendes Punktprogramm ab - der
        gemeinsame Kern hinter on_abbruch_pressed, on_kalibrierung_weiter_pressed,
        on_close und dem Ablöse-Schritt in _run_program_async. Hebt eine evtl.
        aktive Pause zuerst auf, damit run_program() nicht dort auf das
        gleich folgende Abbruch-Signal wartet, statt sofort zu reagieren
        (siehe _wait_while in robot_program.py). Gibt True zurück, wenn
        tatsächlich ein Programm lief und abgebrochen wurde - False, wenn
        ohnehin nichts lief.

        `graceful=False` (Standard, harter Stopp über self._stop_event):
        bricht SOFORT ab, ohne Nullpunkt-Rückfahrt - richtig zum Ablösen
        durch ein neues Programm oder beim Schließen der GUI, wo ein
        zusätzlicher Umweg nur stören würde. Wartet hier auf das
        Thread-Ende (join), da der Aufrufer i.d.R. direkt danach selbst
        etwas an denselben Publishern sendet.

        `graceful=True` (button 'Abbruch', siehe on_abbruch_pressed): setzt
        stattdessen self._abort_event (siehe robot_program.run_program's
        abort_event) - der Worker-Thread überspringt die restlichen
        Programmpunkte, fährt aber noch zum Nullpunkt, bevor er sich ganz
        normal beendet (inkl. des schon bestehenden on_done-Callbacks, der
        z.B. window_programm wieder schließt). Kein join() hier - das
        würde die GUI für die gesamte Nullpunkt-Rückfahrt einfrieren."""
        if self._program_thread is None or not self._program_thread.is_alive():
            return False
        self._pause_event.clear()
        if graceful:
            self._abort_event.set()
        else:
            self._stop_event.set()
            self._program_thread.join(timeout=5.0)
        return True

    def _run_program_async(
        self,
        program_source: "pathlib.Path | rp.Program",
        on_done=None,
        home_at_end: bool = True,
    ) -> None:
        """Startet ein Punktprogramm (siehe robot_program.py) in einem eigenen
        Thread, damit die GUI währenddessen bedienbar bleibt.

        `program_source` ist entweder ein Pfad zu einer Programmdatei (wird
        hier geladen) oder bereits ein fertiges Program-Objekt (z.B.
        rp.build_calibration_program() für den Kalibrierungs-Button) - beide
        laufen über denselben stop_event/self._program_thread-Mechanismus,
        damit sich AUCH die automatische Kalibrierfahrt sauber von einem
        anderen Programm ablösen lässt bzw. selbst abgelöst werden kann,
        statt als nicht-abbrechbarer Sonderfall nebenher zu laufen.

        Läuft bereits ein anderes Programm, wird es zuerst über stop_event
        abgebrochen (spätestens am Ende des gerade angefahrenen Punkts) -
        so kann man z.B. mitten in "Tisch" auf "Gelenkpose" klicken und der
        Arm wechselt sofort die Route, statt erst das alte Programm zu Ende
        abzufahren.

        `on_done` (optional) wird aufgerufen, sobald DIESER Lauf regulär
        endet oder mit einem Fehler abbricht - aber NICHT, wenn er durch
        einen neuen Button-Klick (stop_event) abgebrochen wurde, da dann
        ein anderer Ablauf die GUI übernimmt. Läuft im Hintergrund-Thread,
        wird aber über `mainwindow.after(0, ...)` auf den Tkinter-Hauptthread
        umgeleitet (Tkinter-Widgets dürfen nicht direkt aus einem anderen
        Thread angefasst werden).
        """
        self._stop_running_program()

        stop_event = threading.Event()
        self._stop_event = stop_event
        abort_event = threading.Event()
        self._abort_event = abort_event
        self._pause_event.clear()  # ein neu gestartetes Programm startet nie pausiert
        self._step_event.clear()  # ein neu gestartetes Programm wartet nicht sofort

        # nur im Entwicklermodus MIT aktiviertem Einzelschrittbetrieb wird
        # tatsächlich vor jedem Punkt gewartet (siehe robot_program.run_program).
        step_event = self._step_event if (self._dev_mode and self._single_step_var.get()) else None

        def get_current() -> dict:
            with self._pose_lock:
                return self._current_joints

        def set_current(target: dict) -> None:
            with self._pose_lock:
                self._current_joints = target

        def set_step_waiting(waiting: bool) -> None:
            self.mainwindow.after(0, lambda: self._set_naechster_punkt_enabled(waiting))

        def worker() -> None:
            program_name = "?"
            try:
                publishers = self._ensure_simulation_running()
                program = (
                    rp.Program.from_file(program_source)
                    if isinstance(program_source, pathlib.Path)
                    else program_source
                )
                program_name = program.name
                print(f"Starte Programm '{program.name}' ...")
                rp.run_program(
                    program,
                    publishers,
                    stop_event=stop_event,
                    abort_event=abort_event,
                    pause_event=self._pause_event,
                    current=get_current(),
                    get_current=get_current,
                    on_target_sent=set_current,
                    home_at_end=home_at_end,
                    step_event=step_event,
                    on_step_wait=set_step_waiting,
                )
                if not stop_event.is_set():
                    print(f"Programm '{program.name}' beendet.")
            except Exception as exc:  # noqa: BLE001 - im Hintergrund-Thread sichtbar loggen
                print(f"Fehler beim Ausführen von '{program_name}': {exc}")
                traceback.print_exc()
            finally:
                if on_done is not None and not stop_event.is_set():
                    self.mainwindow.after(0, on_done)

        self._program_thread = threading.Thread(target=worker, daemon=True)
        self._program_thread.start()

    def on_gelenkpose_pressed(self):
        """Button 'Start' (Gelenkpose, in window_testen): fährt
        programs/beispiel_gelenkpose.json ab."""
        self._run_program_async(PROGRAM_GELENKPOSE)

    def on_tisch_pressed(self):
        """Button 'Start' (Tisch, in window_testen): fährt
        programs/beispiel_tisch.json ab."""
        self._run_program_async(PROGRAM_TISCH)

    def _jog(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, drot_deg: float = 0.0) -> bool:
        """Bewegt den Arm um einen kleinen Schritt in X/Y/Z (Schrittweite:
        self._jog_step_m, siehe combo_jogstep in den Optionen) oder dreht
        das Werkzeug (j6) um drot_deg (siehe combo_jogstep_rotation), beides
        ausgehend von der zuletzt bekannten Pose (self._current_joints).
        Gibt True zurück, wenn der Schritt gesendet wurde - False, wenn er
        verweigert wurde oder fehlschlug (dann bricht _hold_jog das
        wiederholte Fahren ab, siehe unten). dx/dy/dz und drot_deg schließen
        sich gegenseitig aus (ein Aufruf bewegt entweder linear oder dreht).

        Läuft bewusst NICHT über die Rückzug/Verfahren/Absenken-Logik aus
        robot_program.py - Jog-Schritte sind klein genug (Standard 5cm),
        dass eine direkte kontinuitätsbasierte IK (resolve_point, ohne den
        breiten Fallback - der wäre für ein reaktionsschnelles "gehalten =
        durchfahren" zu langsam) ohne Höhenumweg schnell und in der Praxis
        sicher bleibt, solange man von einer bereits sicheren Pose aus
        startet. Die Werkzeug-Rotation (drot_deg) braucht gar keine IK -
        sie ändert direkt und ausschließlich j6, alle anderen Gelenke (und
        damit die Werkzeugposition) bleiben unverändert.

        Läuft gerade ein Programm UND ist es pausiert (siehe
        on_pause_pressed), wird einfach mitbewegt - genau dafür ist Pause
        da. Läuft ein Programm und ist es NICHT pausiert, wird der Jog
        verweigert (nicht abgebrochen!): gleichzeitiges Schreiben auf
        dieselben Publisher von Programm und Jog würde sich in die Quere
        kommen. Erst "Pause" drücken, dann jogt es.
        """
        if (
            self._program_thread is not None
            and self._program_thread.is_alive()
            and not self._pause_event.is_set()
        ):
            print("Jog ignoriert: Programm läuft gerade - erst 'Pause' drücken.")
            return False

        with self._pose_lock:
            current = self._current_joints

        if current is None or self._publishers is None:
            print("Jog ignoriert: Simulation ist noch nicht bereit.")
            return False

        Double = sim.import_gz("msgs", "double_pb2").Double

        if drot_deg:
            solution = dict(current)
            solution["j6"] = current["j6"] + math.radians(drot_deg)
            msg = Double()
            msg.data = solution["j6"]
            self._publishers["j6"].publish(msg)
            with self._pose_lock:
                self._current_joints = solution
            return True

        pose = kin.forward_kinematics(current)
        x, y, z = pose[:3, 3]
        target = rp.Point(name="Jog", xyz=(x + dx, y + dy, z + dz))
        try:
            solution = rp.resolve_point(target, current, allow_fallback=False)
        except (ValueError, RuntimeError) as exc:
            print(f"Jog fehlgeschlagen: {exc}")
            return False

        for joint, angle in solution.items():
            msg = Double()
            msg.data = angle
            self._publishers[joint].publish(msg)

        with self._pose_lock:
            self._current_joints = solution
        return True

    def _hold_jog(self, axis: str, sign: float) -> None:
        """Wird beim Drücken eines Jog-Buttons aufgerufen und fährt sofort
        einen Schritt, plant sich danach per `after()` selbst alle
        JOG_REPEAT_MS erneut ein - so bewegt sich der Arm statisch weiter,
        solange der Button gehalten wird. `_release_jog` (beim Loslassen)
        bricht die Kette wieder ab. Bricht auch von selbst ab, wenn `_jog`
        fehlschlägt (z.B. Reichweitenende erreicht), statt sinnlos weiter
        zu versuchen.

        `axis` ist 'x'/'y'/'z'/'rot', die aktuelle Schrittweite
        (self._jog_step_m bzw. self._jog_rotation_step_deg) wird bei JEDEM
        Aufruf frisch gelesen statt beim ersten Drücken fest eingefroren zu
        werden - ändert man die Schrittweite in den Optionen während der
        Arm gerade durchgehend jogt, wirkt das dadurch sofort auf den
        nächsten Schritt."""
        kwargs = {}
        if axis == "x":
            kwargs["dx"] = sign * self._jog_step_m
        elif axis == "y":
            kwargs["dy"] = sign * self._jog_step_m
        elif axis == "z":
            kwargs["dz"] = sign * self._jog_step_m
        elif axis == "rot":
            kwargs["drot_deg"] = sign * self._jog_rotation_step_deg
        if not self._jog(**kwargs):
            self._jog_after_id = None
            return
        self._jog_after_id = self.mainwindow.after(JOG_REPEAT_MS, lambda: self._hold_jog(axis, sign))

    def _release_jog(self) -> None:
        """Beendet die per _hold_jog gestartete Wiederholung (Loslassen des
        Jog-Buttons)."""
        if self._jog_after_id is not None:
            self.mainwindow.after_cancel(self._jog_after_id)
            self._jog_after_id = None

    def _bind_jog_hold(self, widget_id: str, axis: str, sign: float, symbol: str) -> None:
        """Verbindet einen Jog-Button mit Drücken-halten-loslassen statt
        eines einfachen Klicks (siehe _hold_jog/_release_jog), und stylt ihn
        einheitlich mit einem Pfeilsymbol statt "x+"/"y-" o.ä. Wird beim
        erstmaligen Öffnen von window_programm/_kalibrierung/_manuell
        aufgerufen - vorher existiert das Widget in pygubu noch nicht.

        `axis` ('x'/'y'/'z'/'rot') und `sign` (+1/-1) statt fertiger dx/dy/dz-
        Werte, damit _hold_jog die aktuell in den Optionen eingestellte
        Schrittweite (self._jog_step_m/_jog_rotation_step_deg) bei jedem
        Tastendruck neu abfragt, statt einen beim GUI-Start fest gebundenen
        Wert zu verwenden (siehe _hold_jog)."""
        widget = self.builder.get_object(widget_id, self.mainwindow)
        widget.bind("<ButtonPress-1>", lambda event: self._hold_jog(axis, sign))
        widget.bind("<ButtonRelease-1>", lambda event: self._release_jog())
        widget.configure(style="Jog.TButton", text=symbol)

    def _bind_jog_toggle(self, toggle_button_id: str, controls_frame_id: str) -> None:
        """Macht die Kopfzeile eines Jog-Bereichs klickbar: blendet die
        Pfeil-Buttons darunter (frame_jog_controls_programm bzw.
        _kalibrierung) per grid()/grid_remove() ein und aus, statt sie fest
        immer sichtbar zu lassen - praktisch, wenn man die manuelle
        Steuerung gerade nicht braucht und das Fenster kompakter haben will.
        Der Pfeil im Button-Text (▾ = eingeblendet, ▸ = ausgeblendet) zeigt
        den aktuellen Zustand an. Standardzustand beim Öffnen: eingeblendet
        (wie zuvor, als es noch keinen Ein-/Ausblende-Mechanismus gab)."""
        button = self.builder.get_object(toggle_button_id, self.mainwindow)
        controls = self.builder.get_object(controls_frame_id, self.mainwindow)
        window = button.winfo_toplevel()

        def toggle():
            if controls.winfo_ismapped():
                controls.grid_remove()
                button.configure(text=button.cget("text").replace("▾", "▸"))
            else:
                controls.grid()
                button.configure(text=button.cget("text").replace("▸", "▾"))
            # Fenster war zuvor per _fit_window_to_screen() auf eine feste
            # Größe gesetzt (geometry("WxH+X+Y")) - ohne das hier zu
            # aktualisieren, bliebe nach dem Einklappen eine leere weiße
            # Fläche stehen, wo die Jog-Pfeile waren, statt dass sich das
            # Fenster verkleinert. update_idletasks() lässt Tkinter zuerst
            # die neue, kleinere/größere benötigte Größe berechnen; die
            # Fensterposition (x/y) bleibt dabei unverändert, da geometry()
            # hier bewusst ohne "+x+y" aufgerufen wird.
            window.update_idletasks()
            window.geometry(f"{window.winfo_reqwidth()}x{window.winfo_reqheight()}")

        button.configure(command=toggle, style="JogToggle.TButton")

    def _update_position_readout(self) -> None:
        """Aktualisiert die Live-Positionsanzeige (TCP-Position + Werkzeug-
        Drehung j6 - nicht alle 6 Gelenkwinkel, das würde die Fenster nur
        unnötig aufblähen und interessiert beim Jogging ohnehin kaum; j1-j5
        ergeben sich implizit aus X/Y/Z, nur j6 (Werkzeug-Rotation) hat mit
        den Dreh-Buttons einen eigenen, nicht aus X/Y/Z ablesbaren Wert) in
        allen bereits geöffneten Jog-Fenstern und plant sich selbst alle
        300ms erneut ein (gestartet einmalig in __init__) - ein zentraler
        Loop statt einem pro Fenster, da er ohnehin nur dort etwas tut, wo
        das jeweilige Fenster (und damit sein Label) schon existiert."""
        with self._pose_lock:
            current = self._current_joints
        if current is not None:
            pose = kin.forward_kinematics(current)
            x, y, z = pose[:3, 3]
            text = f"TCP  x={x:+.3f}  y={y:+.3f}  z={z:+.3f} m   j6={math.degrees(current['j6']):+.1f}°"
            for window_attr, label_id in (
                ("window_programm", "label_position_readout_programm"),
                ("window_kalibrierung", "label_position_readout_kalibrierung"),
                ("window_manuell", "label_position_readout_manuell"),
            ):
                if getattr(self, window_attr, None) is not None:
                    self.builder.get_object(label_id, self.mainwindow).configure(text=text)
        self.mainwindow.after(300, self._update_position_readout)

    def on_pause_pressed(self):
        """Button 'Pause' (button_pause): pausiert das laufende Punktprogramm,
        damit die Jog-Buttons (X+/X-/Y+/Y-) den Roboter übernehmen können.
        Läuft gerade kein Programm, passiert nichts (Pause wirkt beim
        nächsten Programmstart einfach nicht, da _run_program_async sie
        ohnehin zurücksetzt)."""
        if self._program_thread is None or not self._program_thread.is_alive():
            print("Kein Programm läuft gerade - Pause hat nichts zu tun.")
            return
        self._pause_event.set()
        print("Programm pausiert - Jog-Buttons sind jetzt aktiv.")

    def on_weiter_pressed(self):
        """Button 'Weiter' (button_weiter_programm): hebt die Pause auf, das
        Programm macht ab der zuletzt (ggf. per Jog geänderten) Position
        weiter."""
        if not self._pause_event.is_set():
            print("Programm ist nicht pausiert - nichts zu tun.")
            return
        self._pause_event.clear()
        print("Programm wird fortgesetzt.")

    def on_abbruch_pressed(self):
        """Button 'Abbruch' (button_abbruch, nur in window_programm
        gebunden): bricht beim ERSTEN Klick die restlichen Programmpunkte ab
        (nicht nur pausieren), lässt den Roboter aber zuerst noch zum
        Nullpunkt fahren, bevor das Programm endet (siehe
        _stop_running_program(graceful=True) sowie robot_program.run_program's
        abort_event) - Gazebo bleibt dabei offen.

        Anders als beim harten Stopp (self._stop_event, z.B. beim Ablösen
        durch ein neues Programm) wird `_run_program_async`'s `on_done`
        (bei Figur-Programmen: back_to_main) hier GANZ NORMAL aufgerufen,
        sobald der Worker-Thread die Nullpunkt-Rückfahrt beendet hat - kein
        eigenes Verstecken/Zeigen der Fenster nötig, das übernimmt back_to_main
        wie bei einem normal durchgelaufenen Programm auch.

        Ein ZWEITER Klick, während diese Nullpunkt-Rückfahrt noch läuft
        (self._abort_event bereits gesetzt), eskaliert zum sofortigen harten
        Stopp (graceful=False) - für den Fall, dass der Bedienstand den
        Roboter JETZT anhalten will und nicht erst nach der Rückfahrt zum
        Nullpunkt warten kann. Dabei bleibt kein on_done-Aufruf mehr übrig
        (siehe _run_program_async - bei stop_event wird on_done bewusst
        übersprungen), daher hier wie früher manuell window_programm
        verstecken und das Hauptfenster zeigen."""
        if self._program_thread is None or not self._program_thread.is_alive():
            print("Kein Programm läuft gerade - Abbruch hat nichts zu tun.")
            return

        if self._abort_event.is_set():
            print("Abbruch erneut gedrückt - Sofort-Stopp (keine Nullpunkt-Rückfahrt mehr).")
            self._stop_running_program()  # graceful=False: harter, sofortiger Stopp
            self.window_programm.withdraw()
            self.mainwindow.deiconify()
            return

        self._stop_running_program(graceful=True)
        print("Abbruch angefordert - Roboter fährt noch zum Nullpunkt, danach endet das Programm.")

    def _open_calibration_window(self):
        """Öffnet das Kalibrierungs-Fenster (window_kalibrierung) - einmalig
        automatisch beim GUI-Start (siehe run()), bevor das Hauptfenster mit
        den Programm-Buttons überhaupt sichtbar wird, und danach jederzeit
        erneut über den Button 'Kalibrierung' im Hauptfenster (siehe
        on_kalibrierung_pressed), falls der Tisch später nochmal
        verrutscht. Baut die Jog-Buttons beim allerersten Öffnen (siehe
        _bind_jog_hold) - vorher existieren die Widgets in pygubu noch
        nicht."""
        self.mainwindow.withdraw()
        if getattr(self, 'window_kalibrierung', None) is None:
            self.window_kalibrierung = self.builder.get_object('window_kalibrierung', self.mainwindow)
            self.builder.connect_callbacks(self)
            self.window_kalibrierung.configure(background=self._palette()["bg"])
            self._bind_jog_hold('button_jog_xplus_kalibrierung', axis="x", sign=1, symbol="▲")
            self._bind_jog_hold('button_jog_xminus_kalibrierung', axis="x", sign=-1, symbol="▼")
            self._bind_jog_hold('button_jog_yplus_kalibrierung', axis="y", sign=1, symbol="▶")
            self._bind_jog_hold('button_jog_yminus_kalibrierung', axis="y", sign=-1, symbol="◀")
            self._bind_jog_hold('button_jog_zplus_kalibrierung', axis="z", sign=1, symbol="▲")
            self._bind_jog_hold('button_jog_zminus_kalibrierung', axis="z", sign=-1, symbol="▼")
            self._bind_jog_hold('button_jog_rotplus_kalibrierung', axis="rot", sign=1, symbol="⟳")
            self._bind_jog_hold('button_jog_rotminus_kalibrierung', axis="rot", sign=-1, symbol="⟲")
            self._bind_jog_toggle('button_toggle_jog_kalibrierung', 'frame_jog_controls_kalibrierung')
            self._style_widget("label_kalibrierung_title", "Title.TLabel")
            self._style_widget("frame_jog_kalibrierung", "Card.TLabelframe")
            self._style_widget("frame_jog_controls_kalibrierung", "Card.TFrame")
            self._style_widget("label_jog_xy_header_kalibrierung", "JogAxis.TLabel")
            self._style_widget("label_jog_z_header_kalibrierung", "JogAxis.TLabel")
            self._style_widget("label_jog_rot_header_kalibrierung", "JogAxis.TLabel")
            self._style_widget("label_jog_center_kalibrierung", "JogCenter.TLabel")
            self._style_widget("label_position_readout_kalibrierung", "Readout.TLabel")
            self._style_widget("button_kalibrieren", "Primary.TButton")
            self._style_widget("button_kalibrierung_weiter", "Success.TButton")
            self._make_responsive(self.window_kalibrierung, self.builder.get_object("frame_kalibrierung"))
            self._fit_window_to_screen(self.window_kalibrierung)
        else:
            self.window_kalibrierung.deiconify()
        self.window_kalibrierung.lift()
        self.window_kalibrierung.attributes('-topmost', True)
        self.window_kalibrierung.update()
        self.window_kalibrierung.attributes('-topmost', False)
        self.window_kalibrierung.focus_force()

    def on_kalibrierung_pressed(self):
        """Button 'Kalibrierung' im Optionen-Fenster (button_optionen_kalibrierung):
        öffnet das Kalibrierungs-Fenster erneut, falls der Tisch nach dem
        ersten Kalibrieren beim GUI-Start nochmal verrutscht."""
        self._open_calibration_window()

    def on_kalibrierung_weiter_pressed(self):
        """Button 'Weiter' im Kalibrierungs-Fenster (button_kalibrierung_weiter): schließt die
        Kalibrierung ab - mit oder ohne tatsächliches Neukalibrieren (siehe
        on_kalibrieren_pressed), ein zuvor gespeicherter Tisch-Offset gilt
        einfach unverändert weiter - und zeigt das Hauptfenster mit den
        Programm-Buttons. Vor dem allerersten Aufruf dieses Buttons ist das
        Hauptfenster nicht erreichbar (siehe run()), es kann also kein
        Programm gestartet werden, bevor der Nutzer diese Kalibrierung
        gesehen/bestätigt hat.

        Bricht zuerst eine noch laufende automatische Kalibrierfahrt ab
        (siehe on_kalibrieren_pressed/_run_program_async), falls "Weiter"
        geklickt wird, bevor der Roboter den Kalibrierpunkt erreicht hat -
        sonst würde die Fahrt im Hintergrund weiterlaufen und
        _confirm_and_calibrate später unerwartet über dem schon sichtbaren
        Hauptfenster aufpoppen, statt gar nicht mehr zu erscheinen."""
        self._stop_running_program()
        self._release_jog()
        self.window_kalibrierung.withdraw()
        self.mainwindow.deiconify()
        self.mainwindow.lift()
        self.mainwindow.focus_force()

    def on_kalibrieren_pressed(self):
        """Button 'Nullpunkt kalibrieren' (button_kalibrieren im Kalibrierungs-Fenster):
        fährt den Roboter zunächst automatisch auf den markanten Kalibrierpunkt
        (sim.CALIBRATION_REFERENCE_XYZ, siehe Kommentar dort - ein Punkt auf
        der Baugruppe statt einer bloßen Tischkante), damit der Nutzer nicht
        mehr den kompletten Weg per Jog manuell zurücklegen muss. Erst danach
        erscheint der Bestätigungsdialog: von dort kann per Jog noch
        feinjustiert werden (falls die Baugruppe leicht verrutscht ist),
        bevor bestätigt wird - dann wird aus der aktuellen Pose der neue
        Tisch-Offset berechnet und gespeichert (siehe
        robot_program.calibrate_table_offset) und ab dem nächsten gesendeten
        Punkt automatisch auf alle kartesischen Programmziele angewendet
        (siehe robot_program.run_program).

        Läuft über denselben _run_program_async-Mechanismus wie die anderen
        Programm-Buttons (Gelenkpose/Tisch/Figur1) - wichtig, damit diese
        Fahrt genau wie jedes andere Programm über stop_event abgebrochen
        bzw. von einem anderen Button sauber abgelöst werden kann, statt als
        nicht-unterbrechbarer Sonderfall mit einem eigenen, losgelösten
        Thread zu laufen (der z.B. beim Wechsel auf 'Figur1' mitten in der
        Kalibrierfahrt hängen bliebe und dessen Gelenkbefehle sich dann mit
        denen des neuen Programms überschneiden würden).

        home_at_end=False: der Kalibrierpunkt IST bereits der Nullpunkt, ein
        automatisches Nachfahren dorthin (siehe robot_program.run_program)
        wäre hier nur ein unnötiger Zusatzschritt."""
        self._run_program_async(
            rp.build_calibration_program(),
            on_done=self._confirm_and_calibrate,
            home_at_end=False,
        )

    def _confirm_and_calibrate(self) -> None:
        """Zweiter Teil von on_kalibrieren_pressed, nachdem der Roboter den
        Kalibrierpunkt automatisch angefahren hat (läuft auf dem
        Tkinter-Hauptthread, siehe mainwindow.after in on_kalibrieren_pressed)."""
        with self._pose_lock:
            current = self._current_joints
        if current is None:
            print("Kalibrierung nicht möglich: Simulation ist noch nicht bereit.")
            return

        if not messagebox.askyesno(
            "Nullpunkt kalibrieren",
            "Der Roboter wurde automatisch auf den Kalibrierpunkt gefahren. "
            "Steht der Werkzeugmittelpunkt exakt auf der markanten Stelle "
            "auf der Baugruppe? Falls nicht, jetzt per Jog feinjustieren, "
            "dann bestätigen.\n\n"
            "Der neue Tisch-Offset wird sofort gespeichert und auf alle "
            "künftigen Programmläufe angewendet.",
        ):
            return

        dx, dy, dz = rp.calibrate_table_offset(current)
        print(f"Tisch-Offset kalibriert: dx={dx:+.4f} dy={dy:+.4f} dz={dz:+.4f} m")
        messagebox.showinfo(
            "Nullpunkt kalibriert",
            f"Neuer Tisch-Offset gespeichert:\ndx = {dx:+.4f} m\ndy = {dy:+.4f} m\ndz = {dz:+.4f} m",
        )

    def _run_figure_program(self, program_path: pathlib.Path) -> None:
        """Zeigt window_programm ('Programm läuft...') statt des
        Hauptfensters und fährt die übergebene Programmdatei ab. Sobald das
        Programm fertig ist (oder abbricht), schließt sich window_programm
        wieder und das Hauptfenster erscheint erneut. Gemeinsamer Kern
        hinter allen 'Form Körper N'-Buttons im Hauptfenster.

        window_programm enthält (im Designer so angelegt) auch die
        Jog-Buttons (X+/X-/Y+/Y-/Z+/Z-) sowie Pause/Weiter/Abbruch - macht
        Sinn, da diese nur Sinn ergeben, während gerade ein Programm
        läuft."""
        self.mainwindow.withdraw()
        if getattr(self, 'window_programm', None) is None:
            self.window_programm = self.builder.get_object('window_programm', self.mainwindow)
            self.builder.connect_callbacks(self)
            self.window_programm.configure(background=self._palette()["bg"])
            # Jog-Buttons auf "gedrückt halten" statt Einzelklick umstellen
            # (siehe _bind_jog_hold) - erst hier möglich, da die Widgets in
            # window_programm vorher noch nicht existieren.
            self._bind_jog_hold('button_jog_xplus_programm', axis="x", sign=1, symbol="▲")
            self._bind_jog_hold('button_jog_xminus_programm', axis="x", sign=-1, symbol="▼")
            self._bind_jog_hold('button_jog_yplus_programm', axis="y", sign=1, symbol="▶")
            self._bind_jog_hold('button_jog_yminus_programm', axis="y", sign=-1, symbol="◀")
            self._bind_jog_hold('button_jog_zplus_programm', axis="z", sign=1, symbol="▲")
            self._bind_jog_hold('button_jog_zminus_programm', axis="z", sign=-1, symbol="▼")
            self._bind_jog_hold('button_jog_rotplus_programm', axis="rot", sign=1, symbol="⟳")
            self._bind_jog_hold('button_jog_rotminus_programm', axis="rot", sign=-1, symbol="⟲")
            self._bind_jog_toggle('button_toggle_jog_programm', 'frame_jog_controls_programm')
            self._style_widget("label_programm_status", "Title.TLabel")
            self._style_widget("frame_jog_programm", "Card.TLabelframe")
            self._style_widget("frame_jog_controls_programm", "Card.TFrame")
            self._style_widget("frame_ablauf_programm", "Card.TLabelframe")
            self._style_widget("label_jog_xy_header_programm", "JogAxis.TLabel")
            self._style_widget("label_jog_z_header_programm", "JogAxis.TLabel")
            self._style_widget("label_jog_rot_header_programm", "JogAxis.TLabel")
            self._style_widget("label_jog_center_programm", "JogCenter.TLabel")
            self._style_widget("label_position_readout_programm", "Readout.TLabel")
            self._style_widget("button_pause", "Warning.TButton")
            self._style_widget("button_weiter_programm", "Success.TButton")
            self._style_widget("button_abbruch", "Danger.TButton")
            self._style_widget("button_naechster_punkt", "Primary.TButton")
            # startet deaktiviert - wird erst per on_step_wait-Callback aktiv
            # (siehe _set_naechster_punkt_enabled/_run_program_async), wenn
            # der Worker tatsächlich auf 'Nächster Punkt' wartet, statt
            # dauerhaft klickbar (aber wirkungslos) zu sein.
            self.builder.get_object("button_naechster_punkt", self.mainwindow).state(["disabled"])
            self._make_responsive(self.window_programm, self.builder.get_object("frame_programm"))
            self._fit_window_to_screen(self.window_programm)
        else:
            self.window_programm.deiconify()

        def back_to_main():
            self.window_programm.withdraw()
            self.mainwindow.deiconify()

        self._run_program_async(program_path, on_done=back_to_main)

    def on_figur1_pressed(self):
        """Button 'Form Körper 1': fährt programs/beispiel_figur1.json ab."""
        self._run_figure_program(PROGRAM_FIGUR1)

    def on_tischecken_pressed(self):
        """Button 'Tisch-Ecken': fährt programs/beispiel_tisch_ecken.json ab -
        der Arm fährt jede der vier Tischecken mit 0,5 m Höhenabstand über
        der Tischplatte an und dreht dort einmal den Werkzeugkopf (yaw 360°)."""
        self._run_figure_program(PROGRAM_ECKEN)

    def on_testen_pressed(self):
        """Button 'Testen' im Hauptfenster: öffnet window_testen (analog zu
        on_figur1_pressed, aber ohne direkten Programmstart - dort liegen
        die separaten Start-Buttons für Gelenkpose/Tisch)."""
        self.mainwindow.withdraw()
        if getattr(self, 'window_testen', None) is None:
            self.window_testen = self.builder.get_object('window_testen', self.mainwindow)
            self.builder.connect_callbacks(self)   # <-- Callbacks für window_testen verbinden!
            self.window_testen.configure(background=self._palette()["bg"])
            self._style_widget("label_testen_title", "Title.TLabel")
            self._style_widget("card_gelenkpose", "Card.TLabelframe")
            self._style_widget("card_tisch", "Card.TLabelframe")
            self._style_widget("button_start_gelenkpose", "Primary.TButton")
            self._style_widget("button_start_tisch", "Primary.TButton")
            self._style_widget("button_testen_zurueck", "Secondary.TButton")
            self._make_responsive(self.window_testen, self.builder.get_object("frame_testen"))
            self._fit_window_to_screen(self.window_testen)
        else:
            self.window_testen.deiconify()

    def on_testen_zurueck_pressed(self):
        """Button '◂ Zurück' in window_testen: zurück zum Hauptfenster."""
        self.window_testen.withdraw()
        self.mainwindow.deiconify()

    def on_optionen_pressed(self):
        """Button 'Optionen' im Hauptfenster: öffnet window_optionen -
        anders als alle übrigen Fenster wird das Hauptfenster dabei bewusst
        NICHT versteckt (kein mainwindow.withdraw()), da die Optionen
        (Kalibrierung, Manuelle Steuerung) parallel zum Hauptfenster
        nutzbar bleiben sollen, statt es komplett zu ersetzen."""
        if getattr(self, 'window_optionen', None) is None:
            self.window_optionen = self.builder.get_object('window_optionen', self.mainwindow)
            self.builder.connect_callbacks(self)
            self.window_optionen.configure(background=self._palette()["bg"])
            self._style_widget("label_optionen_title", "Title.TLabel")
            self._style_widget("button_optionen_kalibrierung", "Primary.TButton")
            self._style_widget("button_optionen_manuell", "Primary.TButton")
            self._style_widget("button_optionen_log", "Primary.TButton")
            self._style_widget("frame_einstellungen", "Card.TLabelframe")
            self._style_widget("button_optionen_schliessen", "Secondary.TButton")

            # Einstellungen-Comboboxen (Jog-Schrittweite/Schriftgröße/Zoom) -
            # "readonly" verhindert Freitext-Eingabe, es dürfen nur die
            # vorgegebenen values gewählt werden. Aktueller Wert (self._...)
            # wird als Startauswahl gesetzt, damit die Anzeige stimmt, auch
            # wenn die Optionen nicht die erste geöffnete Ansicht sind.
            combo_jogstep = self.builder.get_object("combo_jogstep", self.mainwindow)
            combo_jogstep["values"] = list(JOG_STEP_OPTIONS.keys())
            combo_jogstep.state(["readonly"])
            combo_jogstep.set(next(k for k, v in JOG_STEP_OPTIONS.items() if v == self._jog_step_m))
            combo_jogstep.bind("<<ComboboxSelected>>", lambda e: self._on_jogstep_selected(combo_jogstep.get()))

            combo_schrift = self.builder.get_object("combo_schriftgroesse", self.mainwindow)
            combo_schrift["values"] = list(FONT_SIZE_OPTIONS.keys())
            combo_schrift.state(["readonly"])
            combo_schrift.set(next(k for k, v in FONT_SIZE_OPTIONS.items() if v == self._font_size))
            combo_schrift.bind("<<ComboboxSelected>>", lambda e: self._on_schriftgroesse_selected(combo_schrift.get()))

            combo_zoom = self.builder.get_object("combo_zoom", self.mainwindow)
            combo_zoom["values"] = list(ZOOM_OPTIONS.keys())
            combo_zoom.state(["readonly"])
            combo_zoom.set(next(k for k, v in ZOOM_OPTIONS.items() if v == self._zoom))
            combo_zoom.bind("<<ComboboxSelected>>", lambda e: self._on_zoom_selected(combo_zoom.get()))

            self._style_widget(
                "button_darkmode_toggle",
                "Secondary.TButton",
                text="☀ Helles Design" if self._dark_mode else "🌙 Dunkles Design",
            )

            # Entwicklermodus (siehe on_entwicklermodus_toggle_pressed) -
            # Checkbutton startet deaktiviert, solange der Entwicklermodus
            # selbst noch nicht per Passwort freigeschaltet wurde.
            self._style_widget("frame_entwicklermodus", "Card.TLabelframe")
            self._style_widget(
                "button_entwicklermodus_toggle",
                "Secondary.TButton",
                text="🔧 Entwicklermodus deaktivieren" if self._dev_mode else "🔧 Entwicklermodus aktivieren",
            )
            checkbutton_einzelschritt = self.builder.get_object("checkbutton_einzelschritt", self.mainwindow)
            checkbutton_einzelschritt.configure(variable=self._single_step_var)
            checkbutton_einzelschritt.state(["!disabled"] if self._dev_mode else ["disabled"])

            self._make_responsive(self.window_optionen, self.builder.get_object("frame_optionen"))
            self._fit_window_to_screen(self.window_optionen)
        else:
            self.window_optionen.deiconify()
        self.window_optionen.lift()
        self.window_optionen.focus_force()

    def on_optionen_schliessen_pressed(self):
        """Button '◂ Schließen' in window_optionen: blendet nur dieses
        Fenster aus - das Hauptfenster war nie versteckt (siehe
        on_optionen_pressed), muss also nicht extra wieder gezeigt werden."""
        self.window_optionen.withdraw()

    def _on_jogstep_selected(self, label: str) -> None:
        """Callback der combo_jogstep-Combobox (siehe on_optionen_pressed) -
        self._jog_step_m wird von _hold_jog bei jedem Tastendruck frisch
        gelesen, wirkt also sofort auf den nächsten Jog-Schritt."""
        self._jog_step_m = JOG_STEP_OPTIONS[label]
        print(f"Jog-Schrittweite: {label}")

    def _on_schriftgroesse_selected(self, label: str) -> None:
        """Callback der combo_schriftgroesse-Combobox: setzt die
        Grundschriftgröße und wendet sie sofort auf alle offenen Fenster an
        (siehe _apply_style - ttk-Styles aktualisieren sich live)."""
        self._font_size = FONT_SIZE_OPTIONS[label]
        self._apply_style()
        print(f"Schriftgröße: {label}")

    def _on_zoom_selected(self, label: str) -> None:
        """Callback der combo_zoom-Combobox: setzt den Tcl/Tk-Skalierungsfaktor
        für die gesamte Oberfläche - wirkt auf neu gebaute/neu geöffnete
        Fenster sofort; bereits offene Fenster zeigen den neuen Zoom
        spätestens nach einem Schließen/Wiederöffnen vollständig (Tk
        berechnet Fenstergrößen einiger schon bestehender Widgets nicht
        rückwirkend neu)."""
        self._zoom = ZOOM_OPTIONS[label]
        self.mainwindow.tk.call("tk", "scaling", self._zoom * BASE_TK_SCALING)
        print(f"Zoom: {label}")

    def on_darkmode_toggle_pressed(self):
        """Button 'Dunkles Design'/'Helles Design' in window_optionen:
        wechselt die Farbpalette (siehe _palette()) und wendet sie sofort
        auf alle ttk-Styles (_apply_style) sowie auf die Hintergrundfarbe
        aller bereits offenen Toplevel-Fenster an (_refresh_toplevel_backgrounds,
        siehe dortigen Kommentar dazu, warum das zusätzlich nötig ist)."""
        self._dark_mode = not self._dark_mode
        self._apply_style()
        self._refresh_toplevel_backgrounds()
        self._style_widget(
            "button_darkmode_toggle",
            "Secondary.TButton",
            text="☀ Helles Design" if self._dark_mode else "🌙 Dunkles Design",
        )

    def on_entwicklermodus_toggle_pressed(self):
        """Button 'Entwicklermodus aktivieren/deaktivieren' in
        window_optionen: zum Aktivieren muss DEV_MODE_PASSWORD eingegeben
        werden (simpledialog.askstring mit show='*', kein echter
        Sicherheitsmechanismus - nur ein Schutz vor versehentlichem
        Aktivieren). Deaktivieren braucht kein Passwort.

        Deaktivieren schaltet auch den Einzelschrittbetrieb (siehe
        on_einzelschritt_toggled) mit ab und gibt ein evtl. gerade
        wartendes Programm sofort frei (self._step_event.set()) - sonst
        bliebe ein laufendes Programm, das im Einzelschrittbetrieb gerade
        auf 'Nächster Punkt' wartet, nach dem Verlassen des
        Entwicklermodus für immer hängen, da der Button dafür dann nicht
        mehr bedienbar ist."""
        if not self._dev_mode:
            pw = simpledialog.askstring(
                "Entwicklermodus",
                "Passwort eingeben:",
                show="*",
                parent=self.window_optionen,
            )
            if pw != DEV_MODE_PASSWORD:
                if pw is not None:  # None = Dialog abgebrochen, keine Meldung nötig
                    messagebox.showerror("Entwicklermodus", "Falsches Passwort.")
                return
            self._dev_mode = True
            print("Entwicklermodus aktiviert.")
        else:
            self._dev_mode = False
            self._single_step_var.set(False)
            self._step_event.set()  # evtl. wartendes Programm freigeben
            print("Entwicklermodus deaktiviert.")

        self._style_widget(
            "button_entwicklermodus_toggle",
            "Secondary.TButton",
            text="🔧 Entwicklermodus deaktivieren" if self._dev_mode else "🔧 Entwicklermodus aktivieren",
        )
        checkbutton = self.builder.get_object("checkbutton_einzelschritt", self.mainwindow)
        checkbutton.state(["!disabled"] if self._dev_mode else ["disabled"])

    def on_einzelschritt_toggled(self):
        """Checkbutton 'Einzelschrittbetrieb' in window_optionen (nur im
        Entwicklermodus bedienbar, siehe on_entwicklermodus_toggle_pressed):
        self._single_step_var wird von _run_program_async beim nächsten
        Programmstart gelesen (siehe dortiges step_event). Wird es
        WÄHREND ein Programm bereits darauf wartet ausgeschaltet, gibt
        self._step_event.set() dieses Warten sofort frei, statt den
        Roboter bis zum nächsten Klick auf 'Nächster Punkt' stehen zu
        lassen."""
        if not self._single_step_var.get():
            self._step_event.set()
        print(f"Einzelschrittbetrieb: {'ein' if self._single_step_var.get() else 'aus'}")

    def on_naechster_punkt_pressed(self):
        """Button 'Nächster Punkt' in window_programm (nur im
        Einzelschrittbetrieb relevant): gibt den aktuell wartenden Punkt
        frei (siehe robot_program.run_program's step_event/_wait_for_step).
        Ohne aktives Warten ist der Klick wirkungslos (das Event wird beim
        nächsten Punkt ohnehin sofort wieder gebraucht/gelöscht)."""
        self._step_event.set()

    def _set_naechster_punkt_enabled(self, enabled: bool) -> None:
        """Schaltet den 'Nächster Punkt'-Button (nur) genau dann aktiv, wenn
        robot_program.run_program gerade wirklich darauf wartet (siehe
        dessen on_step_wait-Callback, hier in _run_program_async verdrahtet)
        - sonst bliebe er dauerhaft klickbar, obwohl ein Klick oft
        wirkungslos wäre (z.B. während der Roboter noch zum vorherigen
        Punkt unterwegs ist). Wird über mainwindow.after(0, ...) auf den
        Tkinter-Hauptthread umgeleitet, da der Aufrufer (Programm-Worker)
        in einem Hintergrund-Thread läuft. Tut nichts, falls window_programm
        noch gar nicht gebaut wurde (z.B. bei Gelenkpose/Tisch, die nicht
        über window_programm laufen)."""
        if getattr(self, "window_programm", None) is None:
            return
        button = self.builder.get_object("button_naechster_punkt", self.mainwindow)
        button.state(["!disabled"] if enabled else ["disabled"])

    def _on_log_line(self, text: str) -> None:
        """Wird von _StdoutTee bei JEDEM print()/stdout-Write aufgerufen -
        auch aus Hintergrund-Threads (z.B. während ein Programm läuft, siehe
        _run_program_async). Deshalb: nur threadsicher in die deque
        schreiben, das eigentliche Text-Widget-Update über after(0, ...) auf
        den Tkinter-Hauptthread verlagern (derselbe Kniff wie beim
        bestehenden on_done-Callback in _run_program_async)."""
        self._log_buffer.append(text)
        if self._log_text_widget is not None:
            self.mainwindow.after(0, self._append_log_widget, text)

    def _append_log_widget(self, text: str) -> None:
        widget = self._log_text_widget
        if widget is None or not widget.winfo_exists():
            return
        widget.configure(state="normal")
        widget.insert("end", text)
        widget.see("end")
        widget.configure(state="disabled")

    def on_log_pressed(self):
        """Button 'Log anzeigen' (in window_optionen): öffnet window_log mit
        einer laufend aktualisierten Kopie der Konsolenausgabe (siehe
        _StdoutTee/_on_log_line) - praktisch, um Programmstart/-fehler und
        Kalibrierergebnisse zu sehen, ohne ein Terminal danebenzuhaben."""
        if getattr(self, 'window_log', None) is None:
            self.window_log = self.builder.get_object('window_log', self.mainwindow)
            self.builder.connect_callbacks(self)
            self.window_log.configure(background=self._palette()["bg"])
            self._style_widget("label_log_title", "Title.TLabel")
            self._style_widget("button_log_schliessen", "Secondary.TButton")

            text_widget = self.builder.get_object("text_log", self.mainwindow)
            scrollbar = self.builder.get_object("scroll_log", self.mainwindow)
            text_widget.configure(
                yscrollcommand=scrollbar.set,
                wrap="word",
                background="#12161d",
                foreground="#d7dbe3",
                insertbackground="#d7dbe3",
                font=("Courier", 10),
                relief="flat",
            )
            scrollbar.configure(command=text_widget.yview)

            # bisherigen Puffer einmalig eintragen, danach hält _on_log_line
            # das Widget automatisch auf dem laufenden Stand
            text_widget.insert("end", "".join(self._log_buffer))
            text_widget.see("end")
            text_widget.configure(state="disabled")
            self._log_text_widget = text_widget

            self._make_responsive(self.window_log, self.builder.get_object("frame_log"))
            self._fit_window_to_screen(self.window_log)
        else:
            self.window_log.deiconify()
        self.window_log.lift()
        self.window_log.focus_force()

    def on_log_schliessen_pressed(self):
        """Button '◂ Schließen' in window_log: blendet nur dieses Fenster
        aus - _log_text_widget bleibt gesetzt, _on_log_line schreibt beim
        nächsten print() also weiterhin (unsichtbar) mit, bis das Fenster
        erneut geöffnet wird."""
        self.window_log.withdraw()

    def on_manuell_pressed(self):
        """Button 'Manuelle Steuerung' (in window_optionen): öffnet
        window_manuell - freie Jog-Steuerung des Roboters (X/Y/Z) ganz ohne
        laufendes Punktprogramm, z.B. um kurz von Hand nachzujustieren.
        Nutzt denselben _jog-Mechanismus wie die Jog-Pads in
        window_programm/window_kalibrierung (siehe _bind_jog_hold) - _jog
        verweigert von sich aus, wenn gerade ein Programm läuft und nicht
        pausiert ist, das gilt hier unverändert mit."""
        if getattr(self, 'window_manuell', None) is None:
            self.window_manuell = self.builder.get_object('window_manuell', self.mainwindow)
            self.builder.connect_callbacks(self)
            self.window_manuell.configure(background=self._palette()["bg"])
            self._bind_jog_hold('button_jog_xplus_manuell', axis="x", sign=1, symbol="▲")
            self._bind_jog_hold('button_jog_xminus_manuell', axis="x", sign=-1, symbol="▼")
            self._bind_jog_hold('button_jog_yplus_manuell', axis="y", sign=1, symbol="▶")
            self._bind_jog_hold('button_jog_yminus_manuell', axis="y", sign=-1, symbol="◀")
            self._bind_jog_hold('button_jog_zplus_manuell', axis="z", sign=1, symbol="▲")
            self._bind_jog_hold('button_jog_zminus_manuell', axis="z", sign=-1, symbol="▼")
            self._bind_jog_hold('button_jog_rotplus_manuell', axis="rot", sign=1, symbol="⟳")
            self._bind_jog_hold('button_jog_rotminus_manuell', axis="rot", sign=-1, symbol="⟲")
            self._bind_jog_toggle('button_toggle_jog_manuell', 'frame_jog_controls_manuell')
            self._style_widget("label_manuell_title", "Title.TLabel")
            self._style_widget("frame_jog_manuell", "Card.TLabelframe")
            self._style_widget("frame_jog_controls_manuell", "Card.TFrame")
            self._style_widget("label_jog_xy_header_manuell", "JogAxis.TLabel")
            self._style_widget("label_jog_z_header_manuell", "JogAxis.TLabel")
            self._style_widget("label_jog_rot_header_manuell", "JogAxis.TLabel")
            self._style_widget("label_jog_center_manuell", "JogCenter.TLabel")
            self._style_widget("label_position_readout_manuell", "Readout.TLabel")
            self._style_widget("button_manuell_schliessen", "Secondary.TButton")
            self._make_responsive(self.window_manuell, self.builder.get_object("frame_manuell"))
            self._fit_window_to_screen(self.window_manuell)
        else:
            self.window_manuell.deiconify()
        self.window_manuell.lift()
        self.window_manuell.focus_force()

    def on_manuell_schliessen_pressed(self):
        """Button '◂ Schließen' in window_manuell: blendet nur dieses
        Fenster aus, Haupt- und Optionen-Fenster bleiben unverändert."""
        self._release_jog()
        self.window_manuell.withdraw()

    def on_beenden_pressed(self):
        """Button 'Beenden': beendet das Programm richtig - bricht ein
        laufendes Punktprogramm ab, fährt Gazebo sauber herunter und
        schließt die GUI (dieselbe Aufräum-Logik wie beim Schließen über
        das Fenster-X, siehe on_close/WM_DELETE_WINDOW)."""
        self.on_close()

    def on_close(self):
        # zentrale Aufräum-Routine: Jog stoppen, Programm abbrechen, Gazebo beenden, Fenster schließen
        self._release_jog()
        self._stop_running_program()
        if self._gz_proc is not None:
            sim.shutdown_simulation(self._gz_proc)
        self.mainwindow.destroy()

    def run(self):
        # Aufräum-Routine beim Schließen des Fensters (X-Button) aufrufen
        self.mainwindow.protocol("WM_DELETE_WINDOW", self.on_close)

        # Zuerst die Kalibrierung zeigen (siehe _open_calibration_window) -
        # das Hauptfenster mit den Programm-Buttons bleibt verborgen, bis
        # "Weiter" dort gedrückt wird (siehe on_kalibrierung_weiter_pressed).
        # So kann kein Programm gestartet werden, ohne vorher zumindest die
        # Möglichkeit zur Kalibrierung gesehen zu haben.
        self._open_calibration_window()
        self.window_kalibrierung.protocol("WM_DELETE_WINDOW", self.on_close)

        # Gazebo direkt mit der GUI zusammen starten, statt erst beim ersten
        # Button-Klick - läuft im Hintergrund-Thread, damit das Fenster
        # währenddessen nicht einfriert. Fehler hier abfangen und deutlich
        # loggen (nicht einfach den Thread lautlos sterben lassen) - sonst
        # bleibt z.B. bei einem Timeout auf einem langsamen Rechner ein
        # halbfertiger Zustand zurück, der erst beim nächsten Button-Klick
        # mit einer verwirrenden Fehlermeldung auffällt.
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