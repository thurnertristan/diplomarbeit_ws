"""Punktprogramm-Ausführung: Gazebo-Start, Programm-Worker-Thread, Kalibrierung, die
'Form Körper N'-Fenster und Pause/Weiter/Abbruch. Der Biegekopf-Testlauf hat sein eigenes
Mixin (bending_mixin.py), da er unabhängig vom Arm-Programm läuft."""

import pathlib
import threading
import traceback
from tkinter import messagebox

import fr20_ceiling_gazebo as sim
import robot_program as rp

from constants import PROGRAM_ECKEN, PROGRAM_FIGUR1, PROGRAM_GELENKPOSE, PROGRAM_TISCH


class ProgramMixin:
    def _ensure_simulation_running(self) -> dict:
        """Startet Gazebo einmalig (auch schon beim GUI-Öffnen, siehe run()). Läuft bereits
        eine Simulation, wird sie wiederverwendet. _sim_lock schützt gegen gleichzeitigen
        Aufruf (Auto-Start + Button-Klick)."""
        with self._sim_lock:
            if self._gz_proc is not None and self._gz_proc.poll() is None and self._publishers is not None:
                return self._publishers

            if self._gz_proc is not None and self._gz_proc.poll() is None:
                # gz-sim läuft noch, aber Publisher-Aufbau ist beim letzten Versuch gescheitert -
                # sauber beenden und neu aufsetzen statt mit kaputtem Rest weiterzumachen.
                print("Vorheriger Gazebo-Start war unvollständig, versuche es erneut ...")
                sim.shutdown_simulation(self._gz_proc)
                self._gz_proc = None
                self._publishers = None

            print("Starte Gazebo (FR20, transparente Wände) ...")
            self._gz_proc, self._publishers = sim.launch_simulation()

            # Roboter spawnt in der physikalischen Null-Stellung - erst die kollisionsfreie
            # Zwischenpose anfahren (Null -> Zielpunkt würde direkt durch den Tisch schwingen).
            sim.send_to_safe_staging_pose(self._publishers)
            with self._pose_lock:
                self._current_joints = dict(sim.SAFE_STAGING_POSE)

            print("Gazebo bereit.")
            return self._publishers

    def _stop_running_program(self, graceful: bool = False) -> bool:
        """Bricht ein laufendes Punktprogramm ab - gemeinsamer Kern hinter on_abbruch_pressed,
        on_kalibrierung_weiter_pressed, on_close, _run_program_async. Gibt True zurück, wenn
        tatsächlich etwas lief.

        `graceful=False`: harter, sofortiger Stopp ohne Nullpunkt-Rückfahrt (Ablösen durch neues
        Programm, GUI schließen); wartet per join(), da der Aufrufer meist direkt danach selbst
        an denselben Publishern sendet.

        `graceful=True` (Button 'Abbruch'): setzt self._abort_event - Worker fährt noch zum
        Nullpunkt, bevor er sich normal (inkl. on_done) beendet; kein join(), sonst friert die
        GUI für die Rückfahrt ein."""
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
        """Startet ein Punktprogramm in einem eigenen Thread, damit die GUI bedienbar bleibt.

        `program_source`: Pfad zu einer Programmdatei oder bereits ein Program-Objekt (z.B.
        rp.build_calibration_program()). Läuft bereits ein anderes Programm, wird es zuerst
        abgebrochen (stop_event) - so übernimmt ein neuer Button-Klick sofort die Route.

        `on_done`: wird aufgerufen, sobald DIESER Lauf regulär endet oder fehlschlägt - nicht
        bei Abbruch durch einen neuen Button-Klick. Läuft im Worker-Thread, wird per
        mainwindow.after(0, ...) auf den Tkinter-Hauptthread umgeleitet."""
        self._stop_running_program()

        stop_event = threading.Event()
        self._stop_event = stop_event
        abort_event = threading.Event()
        self._abort_event = abort_event
        self._pause_event.clear()
        self._step_event.clear()

        # Nur im Entwicklermodus MIT Einzelschrittbetrieb wird vor jedem Punkt gewartet.
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
        """Button 'Start' (Gelenkpose, in window_testen)."""
        self._run_program_async(PROGRAM_GELENKPOSE)

    def on_tisch_pressed(self):
        """Button 'Start' (Tisch, in window_testen)."""
        self._run_program_async(PROGRAM_TISCH)

    def on_pause_pressed(self):
        """Button 'Pause': pausiert das laufende Punktprogramm, damit die Jog-Buttons übernehmen können."""
        if self._program_thread is None or not self._program_thread.is_alive():
            print("Kein Programm läuft gerade - Pause hat nichts zu tun.")
            return
        self._pause_event.set()
        print("Programm pausiert - Jog-Buttons sind jetzt aktiv.")

    def on_weiter_pressed(self):
        """Button 'Weiter': hebt die Pause auf, Programm macht ab der aktuellen Position weiter."""
        if not self._pause_event.is_set():
            print("Programm ist nicht pausiert - nichts zu tun.")
            return
        self._pause_event.clear()
        print("Programm wird fortgesetzt.")

    def on_abbruch_pressed(self):
        """Button 'Abbruch' (window_programm): ERSTER Klick bricht die restlichen Punkte ab,
        fährt aber noch zum Nullpunkt (_stop_running_program(graceful=True)), on_done läuft
        normal danach. ZWEITER Klick (während der Rückfahrt) eskaliert zum sofortigen harten
        Stopp - dabei entfällt on_done, daher hier manuell zum Hauptfenster zurück."""
        if self._program_thread is None or not self._program_thread.is_alive():
            print("Kein Programm läuft gerade - Abbruch hat nichts zu tun.")
            return

        if self._abort_event.is_set():
            print("Abbruch erneut gedrückt - Sofort-Stopp (keine Nullpunkt-Rückfahrt mehr).")
            self._stop_running_program()
            self.window_programm.withdraw()
            self.mainwindow.deiconify()
            return

        self._stop_running_program(graceful=True)
        print("Abbruch angefordert - Roboter fährt noch zum Nullpunkt, danach endet das Programm.")

    def _open_calibration_window(self):
        """Öffnet window_kalibrierung - automatisch beim GUI-Start (siehe run(), bevor das
        Hauptfenster erscheint) und jederzeit erneut über 'Kalibrierung' in den Optionen.
        Baut die Jog-Buttons nur beim allerersten Öffnen."""
        self.mainwindow.withdraw()
        if getattr(self, 'window_kalibrierung', None) is None:
            self.window_kalibrierung = self.builder.get_object('window_kalibrierung', self.mainwindow)
            self.builder.connect_callbacks(self)
            self.window_kalibrierung.configure(background=self._palette()["bg"])
            self._bind_jog_pad("kalibrierung")
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
        """Button 'Kalibrierung' in den Optionen: öffnet das Kalibrierungs-Fenster erneut."""
        self._open_calibration_window()

    def on_kalibrierung_weiter_pressed(self):
        """Button 'Weiter' im Kalibrierungs-Fenster: schließt die Kalibrierung ab (mit oder
        ohne tatsächliches Neukalibrieren) und zeigt das Hauptfenster - bricht zuerst eine
        evtl. noch laufende automatische Kalibrierfahrt ab."""
        self._stop_running_program()
        self._release_jog()
        self.window_kalibrierung.withdraw()
        self.mainwindow.deiconify()
        self.mainwindow.lift()
        self.mainwindow.focus_force()

    def on_kalibrieren_pressed(self):
        """Button 'Nullpunkt kalibrieren': fährt den Roboter automatisch auf den
        Kalibrierpunkt (sim.CALIBRATION_REFERENCE_XYZ), danach folgt der Bestätigungsdialog
        zum Feinjustieren. home_at_end=False, da der Kalibrierpunkt bereits der Nullpunkt ist."""
        self._run_program_async(
            rp.build_calibration_program(),
            on_done=self._confirm_and_calibrate,
            home_at_end=False,
        )

    def _confirm_and_calibrate(self) -> None:
        """Zweiter Teil von on_kalibrieren_pressed, nachdem der Kalibrierpunkt angefahren wurde."""
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
        """Zeigt window_programm statt des Hauptfensters und fährt die Programmdatei ab -
        gemeinsamer Kern hinter allen 'Form Körper N'-Buttons. Enthält auch die Jog-Buttons
        sowie Pause/Weiter/Abbruch."""
        self.mainwindow.withdraw()
        if getattr(self, 'window_programm', None) is None:
            self.window_programm = self.builder.get_object('window_programm', self.mainwindow)
            self.builder.connect_callbacks(self)
            self.window_programm.configure(background=self._palette()["bg"])
            self._bind_jog_pad("programm")
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
            # startet deaktiviert - wird erst aktiv, wenn der Worker wirklich auf 'Nächster Punkt' wartet.
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
        """Button 'Form Körper 1'."""
        self._run_figure_program(PROGRAM_FIGUR1)

    def on_tischecken_pressed(self):
        """Button 'Tisch-Ecken': fährt alle vier Tischecken 0,5m über der Tischplatte an und
        dreht dort einmal den Werkzeugkopf (yaw 360°)."""
        self._run_figure_program(PROGRAM_ECKEN)

    def on_naechster_punkt_pressed(self):
        """Button 'Nächster Punkt' (nur im Einzelschrittbetrieb relevant)."""
        self._step_event.set()

    def _set_naechster_punkt_enabled(self, enabled: bool) -> None:
        """Aktiviert 'Nächster Punkt' nur, wenn robot_program.run_program wirklich darauf
        wartet. Tut nichts, falls window_programm noch nicht gebaut wurde."""
        if getattr(self, "window_programm", None) is None:
            return
        button = self.builder.get_object("button_naechster_punkt", self.mainwindow)
        button.state(["!disabled"] if enabled else ["disabled"])
