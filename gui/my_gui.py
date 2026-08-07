import pathlib
import subprocess
import sys
import threading
import time
import traceback

import pygubu
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

PROGRAM_GELENKPOSE = REPO_ROOT / "programs" / "beispiel_drehung.json"
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
        builder.connect_callbacks(self)
        self._images = {}

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
        # on_weiter_pressed/_wait_while_paused). Während der Pause dürfen
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




    def on_btpressed(self):
        print("Button wurde geklickt!")

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
                self._gz_proc.terminate()
                try:
                    self._gz_proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    self._gz_proc.kill()
                self._gz_proc = None
                self._publishers = None

            for tool in ("gz", "ros2"):
                sim.check_tool(tool)
            sim.isolate_gz_partition()

            sim.WORK_DIR.mkdir(exist_ok=True)
            urdf_path = sim.WORK_DIR / f"{sim.MODEL_NAME}.urdf"
            world_path = sim.WORK_DIR / "world.sdf"

            print("Starte Gazebo (FR20, transparente Wände) ...")
            sim.write_mount_urdf(urdf_path)
            sim.append_joint_position_controllers(urdf_path)
            robot_model_sdf = sim.urdf_to_sdf_model(urdf_path)
            world_path.write_text(sim.build_world_sdf(robot_model_sdf))

            self._gz_proc = subprocess.Popen(
                ["gz", "sim", "-r", str(world_path)], env=sim.gazebo_env_with_fr20_meshes()
            )
            sim.wait_for_world(sim.WORLD_NAME)
            time.sleep(2.0)  # bis die Gelenk-Controller aktiv sind

            Node = sim.import_gz("transport").Node
            Double = sim.import_gz("msgs", "double_pb2").Double

            node = Node()
            self._publishers = {
                j: node.advertise(f"/{sim.MODEL_NAME}/{j}/cmd_pos", Double) for j in sim.JOINT_NAMES
            }

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

    def _run_program_async(self, program_path: pathlib.Path, on_done=None) -> None:
        """Startet ein Punktprogramm (siehe robot_program.py) in einem eigenen
        Thread, damit die GUI währenddessen bedienbar bleibt.

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
        if self._program_thread is not None and self._program_thread.is_alive():
            self._stop_event.set()
            self._program_thread.join(timeout=5.0)

        stop_event = threading.Event()
        self._stop_event = stop_event
        self._pause_event.clear()  # ein neu gestartetes Programm startet nie pausiert

        def worker() -> None:
            try:
                publishers = self._ensure_simulation_running()
                program = rp.Program.from_file(program_path)
                print(f"Starte Programm '{program.name}' ...")
                self._run_points(program, publishers, stop_event)
                if not stop_event.is_set():
                    print(f"Programm '{program.name}' beendet.")
            except Exception as exc:  # noqa: BLE001 - im Hintergrund-Thread sichtbar loggen
                print(f"Fehler beim Ausführen von '{program_path.name}': {exc}")
                traceback.print_exc()
            finally:
                if on_done is not None and not stop_event.is_set():
                    self.mainwindow.after(0, on_done)

        self._program_thread = threading.Thread(target=worker, daemon=True)
        self._program_thread.start()

    def _wait_while_paused(self, stop_event: threading.Event) -> None:
        """Blockiert, solange self._pause_event gesetzt ist (siehe
        on_pause_pressed), kehrt aber sofort zurück, falls das Programm
        währenddessen komplett abgebrochen wird (stop_event)."""
        while self._pause_event.is_set() and not stop_event.is_set():
            time.sleep(0.1)

    def _run_points(self, program: rp.Program, publishers: dict, stop_event: threading.Event) -> None:
        """Fährt die Punkte eines Programms ab - eigene, GUI-spezifische
        Schleife statt robot_program.run_program(), aus zwei Gründen:

        1. Pausierbar (on_pause_pressed/on_weiter_pressed): vor jedem
           Zwischenschritt wird gewartet, solange _pause_event gesetzt ist.
        2. Nutzt self._current_joints (mit self._pose_lock) als gemeinsamen
           Positions-Zustand mit den Jog-Buttons - wird während einer Pause
           gejoggt, macht das Programm beim Fortsetzen genau dort weiter,
           wo der Jog den Arm hingefahren hat, statt die alte Position vor
           der Pause anzunehmen.
        """
        Double = sim.import_gz("msgs", "double_pb2").Double

        def send(joint_angles: dict) -> None:
            for joint, angle in joint_angles.items():
                msg = Double()
                msg.data = angle
                publishers[joint].publish(msg)

        for point in program.points:
            self._wait_while_paused(stop_event)
            if stop_event.is_set():
                print("Programm abgebrochen.")
                return
            print(f"-> Punkt '{point.name}'")
            with self._pose_lock:
                current = self._current_joints
            waypoints = rp._build_travel_waypoints(point, current)
            for i, target in enumerate(waypoints):
                self._wait_while_paused(stop_event)
                if stop_event.is_set():
                    print("Programm abgebrochen.")
                    return
                send(target)
                with self._pose_lock:
                    self._current_joints = target
                is_last = i == len(waypoints) - 1
                rp._interruptible_sleep(point.dwell_s if is_last else 0.8, stop_event)

    def on_btgelenk(self):
        """Button 'Gelenkpose': fährt programs/beispiel_gelenkpose.json ab."""
        self._run_program_async(PROGRAM_GELENKPOSE)

    def on_bttisch(self):
        """Button 'Tisch': fährt programs/beispiel_tisch.json ab."""
        self._run_program_async(PROGRAM_TISCH)

    def _jog(self, dx: float = 0.0, dy: float = 0.0) -> bool:
        """Bewegt den Arm um einen kleinen Schritt (JOG_STEP_M) in X/Y,
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
        target = rp.Point(name="Jog", xyz=(x + dx, y + dy, z))
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

    def _hold_jog(self, dx: float = 0.0, dy: float = 0.0) -> None:
        """Wird beim Drücken eines Jog-Buttons aufgerufen und fährt sofort
        einen Schritt, plant sich danach per `after()` selbst alle
        JOG_REPEAT_MS erneut ein - so bewegt sich der Arm statisch weiter,
        solange der Button gehalten wird. `_release_jog` (beim Loslassen)
        bricht die Kette wieder ab. Bricht auch von selbst ab, wenn `_jog`
        fehlschlägt (z.B. Reichweitenende erreicht), statt sinnlos weiter
        zu versuchen."""
        if not self._jog(dx=dx, dy=dy):
            self._jog_after_id = None
            return
        self._jog_after_id = self.mainwindow.after(JOG_REPEAT_MS, lambda: self._hold_jog(dx, dy))

    def _release_jog(self) -> None:
        """Beendet die per _hold_jog gestartete Wiederholung (Loslassen des
        Jog-Buttons)."""
        if self._jog_after_id is not None:
            self.mainwindow.after_cancel(self._jog_after_id)
            self._jog_after_id = None

    def _bind_jog_hold(self, widget_id: str, dx: float = 0.0, dy: float = 0.0) -> None:
        """Verbindet einen Jog-Button mit Drücken-halten-loslassen statt
        eines einfachen Klicks (siehe _hold_jog/_release_jog). Wird beim
        erstmaligen Öffnen von window2 aufgerufen (siehe on_Fabipressed) -
        vorher existiert das Widget in pygubu noch nicht."""
        widget = self.builder.get_object(widget_id, self.mainwindow)
        widget.bind("<ButtonPress-1>", lambda event: self._hold_jog(dx, dy))
        widget.bind("<ButtonRelease-1>", lambda event: self._release_jog())

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
        bleiben dabei offen. Hebt eine evtl. aktive Pause zuerst auf, damit
        _wait_while_paused nicht auf das gleich folgende stop_event wartet,
        statt sofort abzubrechen."""
        if self._program_thread is None or not self._program_thread.is_alive():
            print("Kein Programm läuft gerade - Abbruch hat nichts zu tun.")
            return
        self._pause_event.clear()
        self._stop_event.set()
        self._program_thread.join(timeout=5.0)
        print("Programm abgebrochen.")

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
            self._bind_jog_hold('button6', dx=JOG_STEP_M)
            self._bind_jog_hold('button15', dx=-JOG_STEP_M)
            self._bind_jog_hold('button14', dy=JOG_STEP_M)
            self._bind_jog_hold('button11', dy=-JOG_STEP_M)
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
        if self._stop_event is not None:
            self._stop_event.set()
        if self._program_thread is not None and self._program_thread.is_alive():
            self._program_thread.join(timeout=5.0)
        if self._gz_proc is not None and self._gz_proc.poll() is None:
            self._gz_proc.terminate()
            try:
                self._gz_proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self._gz_proc.kill()
        self.mainwindow.destroy()

    def run(self):
        self.mainwindow.protocol("WM_DELETE_WINDOW", self.on_close)
        self.mainwindow.deiconify()
        self.mainwindow.lift()
        self.mainwindow.attributes('-topmost', True)
        self.mainwindow.update()
        self.mainwindow.attributes('-topmost', False)
        self.mainwindow.focus_force()

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