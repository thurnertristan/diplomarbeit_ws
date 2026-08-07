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
import robot_program as rp  # noqa: E402

PROGRAM_GELENKPOSE = REPO_ROOT / "programs" / "beispiel_drehung.json"
PROGRAM_TISCH = REPO_ROOT / "programs" / "beispiel_tisch.json"
PROGRAM_FIGUR1 = REPO_ROOT / "programs" / "beispiel_figur1.json"


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

            from gz.transport import Node
            from gz.msgs.double_pb2 import Double

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

        def worker() -> None:
            try:
                publishers = self._ensure_simulation_running()
                program = rp.Program.from_file(program_path)
                print(f"Starte Programm '{program.name}' ...")
                rp.run_program(program, publishers, stop_event=stop_event)
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

    def on_btgelenk(self):
        """Button 'Gelenkpose': fährt programs/beispiel_gelenkpose.json ab."""
        self._run_program_async(PROGRAM_GELENKPOSE)

    def on_bttisch(self):
        """Button 'Tisch': fährt programs/beispiel_tisch.json ab."""
        self._run_program_async(PROGRAM_TISCH)

    def on_button2pressed(self):
        """Button 'Form Körper 1': zeigt window3 ('Programm läuft...') statt
        des Hauptfensters und fährt programs/beispiel_figur1.json ab. Sobald
        das Programm fertig ist (oder abbricht), schließt sich window3
        wieder und das Hauptfenster erscheint erneut."""
        self.mainwindow.withdraw()
        if getattr(self, 'window3', None) is None:
            self.window3 = self.builder.get_object('window3', self.mainwindow)
            self.builder.connect_callbacks(self)
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

    def on_close(self):
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