#!/usr/bin/env python3
"""Vorlage: Punktprogramme für den deckenmontierten FR20 abfahren.

Ein "Programm" ist eine benannte Liste von Punkten (siehe `programs/*.json`
für Beispiele). Jeder Punkt ist entweder

  - eine kartesische Zielposition {"x", "y", "z"} (optional "rpy" für die
    Werkzeugorientierung, Grad, Standard: senkrecht nach unten wie
    `fr20_ceiling_gazebo.LOOK_AT_TABLE_POSE`), die per inverser Kinematik
    (`fr20_kinematics`) in Gelenkwinkel umgerechnet wird, oder
  - eine Gelenkpose {"joints": {"j1": ..., ..., "j6": ...}} in Grad, die
    direkt verwendet wird.

Damit lassen sich beliebig viele Programme (= Punktlisten) austauschen,
ohne Code zu ändern: einfach eine neue JSON-Datei nach dem Schema unten
anlegen und über --program auswählen.

Start (Beispiel):
    source /opt/ros/jazzy/setup.bash
    python3 robot_program.py programs/beispiel_tisch.json
    python3 robot_program.py programs/beispiel_wave.json --loop

Voraussetzungen: siehe Docstring von fr20_ceiling_gazebo.py.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

import fr20_ceiling_gazebo as sim
import fr20_kinematics as kin

PROGRAMS_DIR = Path(__file__).resolve().parent / "programs"

# Wie lange (Sekunden) auf jedem Punkt gewartet wird, bevor der nächste
# angefahren wird - Platzhalterwert, falls ein Punkt kein eigenes "dwell"
# angibt.
DEFAULT_DWELL_S = 1.0


# ---------------------------------------------------------------------------
# Datenmodell
# ---------------------------------------------------------------------------

@dataclass
class Point:
    """Ein einzelner Zielpunkt eines Programms."""

    name: str
    joints_deg: dict[str, float] | None = None   # direkte Gelenkpose (Grad)
    xyz: tuple[float, float, float] | None = None  # kartesisches Ziel (m)
    rpy_deg: tuple[float, float, float] | None = None  # Werkzeugorientierung (Grad)
    dwell_s: float = DEFAULT_DWELL_S

    @staticmethod
    def from_dict(d: dict) -> "Point":
        joints_deg = d.get("joints")
        xyz = d.get("xyz") or (
            (d["x"], d["y"], d["z"]) if all(k in d for k in ("x", "y", "z")) else None
        )
        rpy = d.get("rpy")
        return Point(
            name=d.get("name", "?"),
            joints_deg=joints_deg,
            xyz=tuple(xyz) if xyz else None,
            rpy_deg=tuple(rpy) if rpy else None,
            dwell_s=float(d.get("dwell_s", DEFAULT_DWELL_S)),
        )


@dataclass
class Program:
    """Eine benannte Liste von Punkten, die nacheinander angefahren werden."""

    name: str
    points: list[Point] = field(default_factory=list)

    @staticmethod
    def from_file(path: Path) -> "Program":
        data = json.loads(path.read_text())
        points = [Point.from_dict(p) for p in data["points"]]
        return Program(name=data.get("name", path.stem), points=points)


def list_programs() -> list[Path]:
    """Alle verfügbaren Programmdateien im `programs/`-Ordner."""
    if not PROGRAMS_DIR.exists():
        return []
    return sorted(PROGRAMS_DIR.glob("*.json"))


# ---------------------------------------------------------------------------
# Punkt -> Gelenkwinkel
# ---------------------------------------------------------------------------

# Default-Werkzeugorientierung, falls ein kartesischer Punkt kein "rpy" hat:
# Werkzeug-Z-Achse zeigt senkrecht nach unten (wie beim "Blick auf Tisch").
_DEFAULT_TOOL_R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float)


def _rpy_deg_to_matrix(rpy_deg: tuple[float, float, float]) -> np.ndarray:
    r, p, y = (math.radians(v) for v in rpy_deg)
    return kin._rpy_to_matrix(r, p, y)


def resolve_point(point: Point, current_joints_rad: dict[str, float]) -> dict[str, float]:
    """Berechnet die Ziel-Gelenkwinkel (rad) für einen Punkt.

    Für kartesische Punkte wird die numerische IK aus `fr20_kinematics`
    verwendet, mit der aktuellen Pose als Startwert (Seed) - dadurch bleibt
    die gefundene Lösung nahe an der vorherigen Konfiguration, statt
    zwischen mehreren möglichen Ellbogen-Stellungen zu springen.
    """
    if point.joints_deg is not None:
        return {j: math.radians(deg) for j, deg in point.joints_deg.items()}

    if point.xyz is None:
        raise ValueError(f"Punkt '{point.name}' hat weder 'joints' noch 'xyz'.")

    target = np.eye(4)
    target[:3, :3] = (
        _rpy_deg_to_matrix(point.rpy_deg) if point.rpy_deg is not None else _DEFAULT_TOOL_R
    )
    target[:3, 3] = point.xyz

    solution = kin.inverse_kinematics(target, current_joints_rad)
    if solution is None:
        raise RuntimeError(
            f"Keine IK-Lösung für Punkt '{point.name}' ({point.xyz}) gefunden - "
            f"Punkt evtl. außerhalb der Reichweite."
        )
    return solution


# ---------------------------------------------------------------------------
# Programm ausführen
# ---------------------------------------------------------------------------

def _interruptible_sleep(seconds: float, stop_event: "threading.Event | None") -> None:
    """Wie time.sleep(), kehrt aber sofort zurück, sobald stop_event gesetzt wird.

    Wird gebraucht, damit ein per Button gestartetes Programm nicht erst die
    volle dwell_s eines Punkts abwarten muss, bevor ein Abbruch (z.B. durch
    Klick auf einen anderen Programm-Button) greift.
    """
    if stop_event is None:
        time.sleep(seconds)
        return
    deadline = time.time() + seconds
    while time.time() < deadline and not stop_event.is_set():
        time.sleep(min(0.05, deadline - time.time()))


def run_program(
    program: Program,
    publishers: dict,
    loop: bool = False,
    stop_event: "threading.Event | None" = None,
) -> None:
    """Fährt alle Punkte eines Programms nacheinander an.

    `publishers` sind gz-transport-Publisher pro Gelenk, wie sie
    fr20_ceiling_gazebo.send_look_at_table_pose() bzw. robot_gui.py sie
    aufbauen (ein Double-Topic pro Gelenkname aus sim.JOINT_NAMES).

    Ist `stop_event` gesetzt (z.B. threading.Event), wird nach dem aktuellen
    Punkt abgebrochen, sobald das Event gesetzt wird - damit sich zwei
    Programme, die denselben Roboter ansteuern, sauber ablösen können (siehe
    gui/my_gui.py: ein Button-Klick während ein anderes Programm läuft
    bricht es ab und fährt stattdessen sofort die neue Route).
    """
    from gz.msgs.double_pb2 import Double

    # SAFE_STAGING_POSE als Annahme für die aktuelle Pose: gz-sim spawnt den
    # Roboter physikalisch immer in der Null-Stellung, und jeder Aufrufer
    # (fr20_ceiling_gazebo.main(), robot_gui.py, gui/my_gui.py) fährt vor dem
    # ersten Programm/Zielpunkt zunächst dorthin (siehe SAFE_STAGING_POSE in
    # fr20_ceiling_gazebo.py) - ein direkter Sprung Null -> Zielpunkt würde
    # mitten durch den Tisch schwingen.
    current = dict(sim.SAFE_STAGING_POSE)

    def send(joint_angles: dict[str, float]) -> None:
        for joint, angle in joint_angles.items():
            msg = Double()
            msg.data = angle
            publishers[joint].publish(msg)

    while True:
        for point in program.points:
            if stop_event is not None and stop_event.is_set():
                print("Programm abgebrochen.")
                return
            print(f"-> Punkt '{point.name}'")
            target = resolve_point(point, current)
            send(target)
            current = target
            _interruptible_sleep(point.dwell_s, stop_event)
        if not loop or (stop_event is not None and stop_event.is_set()):
            break
        print("Programm-Ende erreicht, starte erneut (--loop) ...")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "program",
        nargs="?",
        help="Pfad zu einer Programmdatei (JSON). Ohne Angabe: verfügbare Programme auflisten.",
    )
    parser.add_argument("--loop", action="store_true", help="Programm nach Ende erneut abfahren")
    parser.add_argument("--headless", action="store_true", help="gz-sim ohne GUI starten")
    args = parser.parse_args()

    if args.program is None:
        available = list_programs()
        if not available:
            print(f"Keine Programme in {PROGRAMS_DIR} gefunden.")
        else:
            print("Verfügbare Programme:")
            for p in available:
                print(f"  {p}")
        return

    program = Program.from_file(Path(args.program))
    print(f"Lade Programm '{program.name}' mit {len(program.points)} Punkten.")

    for tool in ("gz", "ros2"):
        sim.check_tool(tool)

    sim.isolate_gz_partition()

    sim.WORK_DIR.mkdir(exist_ok=True)
    urdf_path = sim.WORK_DIR / f"{sim.MODEL_NAME}.urdf"
    world_path = sim.WORK_DIR / "world.sdf"

    print("Erzeuge URDF für den kopfüber montierten FR20 ...")
    sim.write_mount_urdf(urdf_path)
    sim.append_joint_position_controllers(urdf_path)

    print("Konvertiere URDF zu SDF ...")
    robot_model_sdf = sim.urdf_to_sdf_model(urdf_path)

    print("Erzeuge SDF-Welt ...")
    world_path.write_text(sim.build_world_sdf(robot_model_sdf))

    gz_cmd = ["gz", "sim", "-r", str(world_path)]
    if args.headless:
        gz_cmd.insert(2, "-s")
    print("Starte Gazebo Jetty (gz-sim) ...")
    import subprocess

    gz_proc = subprocess.Popen(gz_cmd, env=sim.gazebo_env_with_fr20_meshes())

    try:
        sim.wait_for_world(sim.WORLD_NAME)
        print("Warte, bis die Gelenk-Controller aktiv sind ...")
        time.sleep(2.0)

        from gz.transport import Node
        from gz.msgs.double_pb2 import Double

        node = Node()
        publishers = {j: node.advertise(f"/{sim.MODEL_NAME}/{j}/cmd_pos", Double) for j in sim.JOINT_NAMES}

        run_program(program, publishers, loop=args.loop)

        print("\nProgramm fertig. Strg+C hier drücken, um zu beenden.")
        gz_proc.wait()
    except KeyboardInterrupt:
        print("\nBeende Gazebo ...")
    finally:
        if gz_proc.poll() is None:
            gz_proc.terminate()
            try:
                gz_proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                gz_proc.kill()


if __name__ == "__main__":
    main()
