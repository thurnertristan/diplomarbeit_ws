import pathlib
import subprocess
import sys
import threading
import traceback

import pygubu
import tkinter as tk
from tkinter import messagebox, ttk
from PIL import Image, ImageTk

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

# Schrittweite (Meter) pro Jog-Schritt (X+/X-/Y+/Y-) und Intervall (ms)
# zwischen den Schritten, solange der Button gehalten wird.
JOG_STEP_M = 0.05
JOG_REPEAT_MS = 150


class MyApp:
    def __init__(self, master=None):
        self.builder = builder = pygubu.Builder()
        builder.add_resource_path(PROJECT_PATH)
        builder.add_from_file(PROJECT_UI)
        self.mainwindow = builder.get_object('mainwindow', master)
        self._apply_style()
        builder.connect_callbacks(self)
        self._images = {}

        # Kachel-Überschriften ("Form Körper N") und der "Beenden"-Knopf
        # gehören zu mainwindow und sind damit schon jetzt gebaut - Buttons
        # in window2/window3 werden erst bei deren erstem Öffnen gestylt
        # (siehe on_Fabipressed/on_button2pressed), da pygubu sie vorher
        # noch nicht erzeugt hat.
        for label_id in ("label1", "label2", "label4", "label5"):
            self._style_widget(label_id, "Header.TLabel")
        self._style_widget("button5", "Danger.TButton")
        self._style_widget("button18", "Warning.TButton")

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

        # after()-Job-ID der laufenden Jog-Wiederholung (siehe _hold_jog/
        # _release_jog) - None, solange kein Jog-Button gehalten wird.
        self._jog_after_id = None


    def set_image(self, widget_id, filename, size=(512, 512)):
        """Lädt ein Bild, skaliert es und setzt es auf ein Widget."""
        img = Image.open(PROJECT_PATH / filename).resize(size)
        photo = ImageTk.PhotoImage(img)
        self._images[widget_id] = photo                      # Referenz behalten!
        self.builder.get_object(widget_id).configure(image=photo)

    def _apply_style(self) -> None:
        """Einheitliches, modernes Erscheinungsbild für die ganze GUI -
        bewusst hier in Python statt in der .ui-Datei: pygubu-Designer
        schreibt die .ui beim Speichern komplett neu und würde reine
        XML-Anpassungen sonst wieder verwerfen (mehrfach passiert), Styles
        aus Python bleiben davon unberührt.
        """
        style = ttk.Style(self.mainwindow)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass  # "clam" nicht verfügbar - beim Standard-Theme bleiben

        bg = "#f2f4f7"
        text = "#20242b"
        base_font = ("Helvetica", 11)

        self.mainwindow.configure(background=bg)
        style.configure("TFrame", background=bg)
        style.configure("TLabel", background=bg, foreground=text, font=base_font)
        style.configure("Header.TLabel", background=bg, foreground=text, font=("Helvetica", 13, "bold"))
        style.configure("TButton", font=base_font, padding=(14, 10))

        def colored_button(name: str, color: str, active: str) -> None:
            style.configure(name, background=color, foreground="white", font=base_font, padding=(14, 10))
            style.map(name, background=[("active", active), ("disabled", "#c8ccd2")])

        colored_button("Primary.TButton", "#2f6fed", "#245ac9")   # Start
        colored_button("Success.TButton", "#2f9e57", "#22803f")   # Weiter
        colored_button("Warning.TButton", "#d98c00", "#b57400")   # Pause
        colored_button("Danger.TButton", "#d9453f", "#b7332e")    # Beenden/Abbruch
        colored_button("Jog.TButton", "#586470", "#414a54")       # X/Y/Z-Jog

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

    def _stop_running_program(self) -> bool:
        """Bricht ein evtl. gerade laufendes Punktprogramm ab und wartet,
        bis der Worker-Thread beendet ist - der gemeinsame Kern hinter
        on_abbruch_pressed, on_kalibrierung_weiter_pressed, on_close und dem
        Ablöse-Schritt in _run_program_async. Hebt eine evtl. aktive Pause
        zuerst auf, damit run_program() nicht dort auf das gleich folgende
        stop_event wartet, statt sofort abzubrechen (siehe _wait_while in
        robot_program.py). Gibt True zurück, wenn tatsächlich ein Programm
        lief und gestoppt wurde - False, wenn ohnehin nichts lief."""
        if self._program_thread is None or not self._program_thread.is_alive():
            return False
        self._pause_event.clear()
        self._stop_event.set()
        self._program_thread.join(timeout=5.0)
        return True

    def _run_program_async(self, program_source: "pathlib.Path | rp.Program", on_done=None) -> None:
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
        self._pause_event.clear()  # ein neu gestartetes Programm startet nie pausiert

        def get_current() -> dict:
            with self._pose_lock:
                return self._current_joints

        def set_current(target: dict) -> None:
            with self._pose_lock:
                self._current_joints = target

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
                    pause_event=self._pause_event,
                    current=get_current(),
                    get_current=get_current,
                    on_target_sent=set_current,
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

    def on_btgelenk(self):
        """Button 'Gelenkpose': fährt programs/beispiel_gelenkpose.json ab."""
        self._run_program_async(PROGRAM_GELENKPOSE)

    def on_bttisch(self):
        """Button 'Tisch': fährt programs/beispiel_tisch.json ab."""
        self._run_program_async(PROGRAM_TISCH)

    def _jog(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> bool:
        """Bewegt den Arm um einen kleinen Schritt (JOG_STEP_M) in X/Y/Z,
        ausgehend von der zuletzt bekannten Pose (self._current_joints).
        Gibt True zurück, wenn der Schritt gesendet wurde - False, wenn er
        verweigert wurde oder fehlschlug (dann bricht _hold_jog das
        wiederholte Fahren ab, siehe unten).

        Läuft bewusst NICHT über die Rückzug/Verfahren/Absenken-Logik aus
        robot_program.py - Jog-Schritte sind klein genug (Standard 5cm),
        dass eine direkte kontinuitätsbasierte IK (resolve_point, ohne den
        breiten Fallback - der wäre für ein reaktionsschnelles "gehalten =
        durchfahren" zu langsam) ohne Höhenumweg schnell und in der Praxis
        sicher bleibt, solange man von einer bereits sicheren Pose aus
        startet.

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

        pose = kin.forward_kinematics(current)
        x, y, z = pose[:3, 3]
        target = rp.Point(name="Jog", xyz=(x + dx, y + dy, z + dz))
        try:
            solution = rp.resolve_point(target, current, allow_fallback=False)
        except (ValueError, RuntimeError) as exc:
            print(f"Jog fehlgeschlagen: {exc}")
            return False

        Double = sim.import_gz("msgs", "double_pb2").Double
        for joint, angle in solution.items():
            msg = Double()
            msg.data = angle
            self._publishers[joint].publish(msg)

        with self._pose_lock:
            self._current_joints = solution
        return True

    def _hold_jog(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> None:
        """Wird beim Drücken eines Jog-Buttons aufgerufen und fährt sofort
        einen Schritt, plant sich danach per `after()` selbst alle
        JOG_REPEAT_MS erneut ein - so bewegt sich der Arm statisch weiter,
        solange der Button gehalten wird. `_release_jog` (beim Loslassen)
        bricht die Kette wieder ab. Bricht auch von selbst ab, wenn `_jog`
        fehlschlägt (z.B. Reichweitenende erreicht), statt sinnlos weiter
        zu versuchen."""
        if not self._jog(dx=dx, dy=dy, dz=dz):
            self._jog_after_id = None
            return
        self._jog_after_id = self.mainwindow.after(JOG_REPEAT_MS, lambda: self._hold_jog(dx, dy, dz))

    def _release_jog(self) -> None:
        """Beendet die per _hold_jog gestartete Wiederholung (Loslassen des
        Jog-Buttons)."""
        if self._jog_after_id is not None:
            self.mainwindow.after_cancel(self._jog_after_id)
            self._jog_after_id = None

    def _bind_jog_hold(
        self, widget_id: str, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, symbol: str = ""
    ) -> None:
        """Verbindet einen Jog-Button mit Drücken-halten-loslassen statt
        eines einfachen Klicks (siehe _hold_jog/_release_jog), und stylt ihn
        einheitlich mit einem Pfeilsymbol statt "x+"/"y-" o.ä. Wird beim
        erstmaligen Öffnen von window3 aufgerufen (siehe on_button2pressed) -
        vorher existiert das Widget in pygubu noch nicht."""
        widget = self.builder.get_object(widget_id, self.mainwindow)
        widget.bind("<ButtonPress-1>", lambda event: self._hold_jog(dx, dy, dz))
        widget.bind("<ButtonRelease-1>", lambda event: self._release_jog())
        widget.configure(style="Jog.TButton", text=symbol)

    def on_pause_pressed(self):
        """Button 'Pause' (button16): pausiert das laufende Punktprogramm,
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
        """Button 'Weiter' (button17): hebt die Pause auf, das Programm
        macht ab der zuletzt (ggf. per Jog geänderten) Position weiter."""
        if not self._pause_event.is_set():
            print("Programm ist nicht pausiert - nichts zu tun.")
            return
        self._pause_event.clear()
        print("Programm wird fortgesetzt.")

    def on_abbruch_pressed(self):
        """Button 'Abbruch' (button19): bricht ein laufendes Punktprogramm
        sofort und endgültig ab (nicht nur pausieren) - Gazebo und die GUI
        bleiben dabei offen."""
        if not self._stop_running_program():
            print("Kein Programm läuft gerade - Abbruch hat nichts zu tun.")
            return
        print("Programm abgebrochen.")

    def _open_calibration_window(self):
        """Öffnet das Kalibrierungs-Fenster (window4) - einmalig automatisch
        beim GUI-Start (siehe run()), bevor das Hauptfenster mit den
        Programm-Buttons überhaupt sichtbar wird, und danach jederzeit
        erneut über den Button 'Kalibrierung' im Hauptfenster (siehe
        on_kalibrierung_pressed), falls der Tisch später nochmal
        verrutscht. Baut die Jog-Buttons beim allerersten Öffnen (siehe
        _bind_jog_hold) - vorher existieren die Widgets in pygubu noch
        nicht."""
        self.mainwindow.withdraw()
        if getattr(self, 'window4', None) is None:
            self.window4 = self.builder.get_object('window4', self.mainwindow)
            self.builder.connect_callbacks(self)
            self._bind_jog_hold('button23', dx=JOG_STEP_M, symbol="X ▸")
            self._bind_jog_hold('button24', dx=-JOG_STEP_M, symbol="◂ X")
            self._bind_jog_hold('button25', dy=JOG_STEP_M, symbol="Y ▴")
            self._bind_jog_hold('button26', dy=-JOG_STEP_M, symbol="Y ▾")
            self._bind_jog_hold('button27', dz=JOG_STEP_M, symbol="Z ▴")
            self._bind_jog_hold('button28', dz=-JOG_STEP_M, symbol="Z ▾")
            self._style_widget("button29", "Primary.TButton")
            self._style_widget("button30", "Success.TButton")
        else:
            self.window4.deiconify()
        self.window4.lift()
        self.window4.attributes('-topmost', True)
        self.window4.update()
        self.window4.attributes('-topmost', False)
        self.window4.focus_force()

    def on_kalibrierung_pressed(self):
        """Button 'Kalibrierung' im Hauptfenster (button18): öffnet das
        Kalibrierungs-Fenster erneut, falls der Tisch nach dem ersten
        Kalibrieren beim GUI-Start nochmal verrutscht."""
        self._open_calibration_window()

    def on_kalibrierung_weiter_pressed(self):
        """Button 'Weiter' im Kalibrierungs-Fenster (button30): schließt die
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
        self.window4.withdraw()
        self.mainwindow.deiconify()
        self.mainwindow.lift()
        self.mainwindow.focus_force()

    def on_kalibrieren_pressed(self):
        """Button 'Nullpunkt kalibrieren' (button29 im Kalibrierungs-Fenster):
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
        denen des neuen Programms überschneiden würden)."""
        self._run_program_async(rp.build_calibration_program(), on_done=self._confirm_and_calibrate)

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

    def on_button2pressed(self):
        """Button 'Form Körper 1': zeigt window3 ('Programm läuft...') statt
        des Hauptfensters und fährt programs/beispiel_figur1.json ab. Sobald
        das Programm fertig ist (oder abbricht), schließt sich window3
        wieder und das Hauptfenster erscheint erneut.

        window3 enthält (im Designer so angelegt) auch die Jog-Buttons
        (x+/x-/y+/y-) sowie Pause/Weiter/Abbruch - macht Sinn, da diese nur
        Sinn ergeben, während gerade ein Programm läuft."""
        self.mainwindow.withdraw()
        if getattr(self, 'window3', None) is None:
            self.window3 = self.builder.get_object('window3', self.mainwindow)
            self.builder.connect_callbacks(self)
            # Jog-Buttons auf "gedrückt halten" statt Einzelklick umstellen
            # (siehe _bind_jog_hold) - erst hier möglich, da die Widgets in
            # window3 vorher noch nicht existieren.
            self._bind_jog_hold('button6', dx=JOG_STEP_M, symbol="X ▸")
            self._bind_jog_hold('button15', dx=-JOG_STEP_M, symbol="◂ X")
            self._bind_jog_hold('button14', dy=JOG_STEP_M, symbol="Y ▴")
            self._bind_jog_hold('button11', dy=-JOG_STEP_M, symbol="Y ▾")
            self._bind_jog_hold('button20', dz=JOG_STEP_M, symbol="Z ▴")
            self._bind_jog_hold('button21', dz=-JOG_STEP_M, symbol="Z ▾")
            self._style_widget("button16", "Warning.TButton")
            self._style_widget("button17", "Success.TButton")
            self._style_widget("button19", "Danger.TButton")
            self._style_widget("button22", "Primary.TButton")
        else:
            self.window3.deiconify()

        def back_to_main():
            self.window3.withdraw()
            self.mainwindow.deiconify()

        self._run_program_async(PROGRAM_FIGUR1, on_done=back_to_main)

    def on_Fabipressed(self):
        self.mainwindow.withdraw()
        if getattr(self, 'window2', None) is None:
            self.window2 = self.builder.get_object('window2', self.mainwindow)
            self.builder.connect_callbacks(self)   # <-- Callbacks für window2 verbinden!
            self.window2.configure(background="#f2f4f7")
            self._style_widget("button12", "Primary.TButton")
            self._style_widget("button13", "Primary.TButton")
        else:
            self.window2.deiconify()

    def on_sebblpressed(self):
        self.window2.withdraw()
        self.mainwindow.deiconify()

    def on_beenden_pressed(self):
        """Button 'Beenden': beendet das Programm richtig - bricht ein
        laufendes Punktprogramm ab, fährt Gazebo sauber herunter und
        schließt die GUI (dieselbe Aufräum-Logik wie beim Schließen über
        das Fenster-X, siehe on_close/WM_DELETE_WINDOW)."""
        self.on_close()

    def on_close(self):
        self._release_jog()
        self._stop_running_program()
        if self._gz_proc is not None:
            sim.shutdown_simulation(self._gz_proc)
        self.mainwindow.destroy()

    def run(self):
        self.mainwindow.protocol("WM_DELETE_WINDOW", self.on_close)

        # Zuerst die Kalibrierung zeigen (siehe _open_calibration_window) -
        # das Hauptfenster mit den Programm-Buttons bleibt verborgen, bis
        # "Weiter" dort gedrückt wird (siehe on_kalibrierung_weiter_pressed).
        # So kann kein Programm gestartet werden, ohne vorher zumindest die
        # Möglichkeit zur Kalibrierung gesehen zu haben.
        self._open_calibration_window()
        self.window4.protocol("WM_DELETE_WINDOW", self.on_close)

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