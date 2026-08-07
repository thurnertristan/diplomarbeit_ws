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


# Alternative Startwerte (j1, j2, j3, j4) für den Fallback in resolve_point,
# falls die vom aktuellen Punkt aus fortgesetzte IK nicht konvergiert. Deckt
# die üblichen Ellbogen-Konfigurationen ab (oben/unten, links/rechts), damit
# auch große Sprünge zwischen zwei Punkten (z.B. quer über den Tisch) eine
# Lösung finden, statt nur an einem lokalen Minimum nahe der alten Pose
# hängenzubleiben. j5/j6 werden dabei aus dem bisherigen current_joints_rad
# übernommen, da sie kaum Einfluss auf die grobe Erreichbarkeit haben.
_FALLBACK_J1 = (-2.5, -1.25, 0.0, 1.25, 2.5)
_FALLBACK_ELBOWS = ((-1.5, -2.2, -1.2), (-1.5, 2.2, -1.2), (-0.5, -2.6, -1.5), (-0.5, 2.6, -1.5))

_urdf_chain_cache: "kin.UrdfChain | None" = None


def _get_chain() -> "kin.UrdfChain":
    """Lädt die Gelenkkette einmalig aus dem generierten URDF (für die
    Tisch-Kollisionsprüfung in resolve_point, siehe _is_transition_safe)."""
    global _urdf_chain_cache
    if _urdf_chain_cache is None:
        sim.WORK_DIR.mkdir(exist_ok=True)
        urdf_path = sim.WORK_DIR / f"{sim.MODEL_NAME}.urdf"
        if not urdf_path.exists():
            sim.write_mount_urdf(urdf_path)
        _urdf_chain_cache = kin.UrdfChain(urdf_path)
    return _urdf_chain_cache


_TABLE_CHECK_MARGIN_M = 0.05


def _pose_hits_table(position: np.ndarray) -> bool:
    hx, hy = sim.TABLE_SIZE_XY[0] / 2, sim.TABLE_SIZE_XY[1] / 2
    tx, ty = sim.TABLE_CENTER
    x, y, z = position
    return (
        tx - hx - _TABLE_CHECK_MARGIN_M <= x <= tx + hx + _TABLE_CHECK_MARGIN_M
        and ty - hy - _TABLE_CHECK_MARGIN_M <= y <= ty + hy + _TABLE_CHECK_MARGIN_M
        and z < sim.TABLE_TOP_Z + _TABLE_CHECK_MARGIN_M
    )


def _is_transition_safe(a: dict[str, float], b: dict[str, float], steps: int = 15) -> bool:
    """Prüft per Sweep (lineare Gelenkraum-Interpolation zwischen a und b,
    wie es die reale gz-sim JointPositionController-Bewegung annähert), ob
    dabei irgendein Armglied den Tisch schneidet. Siehe Kommentar bei
    resolve_point - zwei für sich sichere Posen können trotzdem über einen
    unsicheren Zwischenweg verbunden sein, wenn sich dabei die
    Ellbogen-Konfiguration stark ändert."""
    chain = _get_chain()
    for frac in np.linspace(0, 1, steps):
        inter = {j: a[j] * (1 - frac) + b[j] * frac for j in sim.JOINT_NAMES}
        t = np.eye(4)
        for joint in chain.chain:
            t = t @ kin._origin_transform(joint.find("origin"))
            if joint.get("type") != "fixed":
                t = t @ kin._axis_rotation(joint.find("axis"), inter[joint.get("name")])
            if _pose_hits_table(t[:3, 3]):
                return False
    return True


def resolve_point(
    point: Point,
    current_joints_rad: dict[str, float],
    allow_fallback: bool = True,
) -> dict[str, float]:
    """Berechnet die Ziel-Gelenkwinkel (rad) für einen Punkt.

    Für kartesische Punkte wird die numerische IK aus `fr20_kinematics`
    verwendet, mit der aktuellen Pose als Startwert (Seed) - dadurch bleibt
    die gefundene Lösung meist nahe an der vorherigen Konfiguration, statt
    zwischen mehreren möglichen Ellbogen-Stellungen zu springen.

    Ist der Sprung zum Zielpunkt zu groß (z.B. weil ein Programm quer über
    den Tisch springt), bleibt diese kontinuitätsbasierte Suche manchmal in
    einem lokalen Minimum hängen, obwohl der Punkt eigentlich erreichbar
    wäre. Deshalb wird bei einem Fehlschlag zusätzlich mit ein paar
    alternativen Ellbogen-Konfigurationen (_FALLBACK_ELBOWS/_FALLBACK_J1)
    gesucht - und zwar nicht nur die ERSTE gefundene Lösung genommen,
    sondern unter allen gefundenen Kandidaten die bevorzugt, deren
    Übergang von current_joints_rad aus nachweislich nicht durch den Tisch
    schwingt (siehe _is_transition_safe). Das behebt genau das Problem, das
    beim Debuggen von programs/beispiel_figur.json auftrat: mehrere gültige
    IK-Lösungen existierten, aber die naiv erste gefundene kollidierte.
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

    candidates: list[dict[str, float]] = []
    primary = kin.inverse_kinematics(target, current_joints_rad)
    if primary is not None:
        candidates.append(primary)

    if allow_fallback:
        for j1 in _FALLBACK_J1:
            for j2, j3, j4 in _FALLBACK_ELBOWS:
                seed = dict(current_joints_rad, j1=j1, j2=j2, j3=j3, j4=j4)
                sol = kin.inverse_kinematics(target, seed)
                if sol is not None:
                    candidates.append(sol)
                    if len(candidates) >= 8:  # genug Auswahl, nicht unnötig weitersuchen
                        break
            if len(candidates) >= 8:
                break

    if not candidates:
        raise RuntimeError(
            f"Keine IK-Lösung für Punkt '{point.name}' ({point.xyz}) gefunden - "
            f"Punkt evtl. außerhalb der Reichweite."
        )

    for candidate in candidates:
        if _is_transition_safe(current_joints_rad, candidate):
            return candidate

    # Keiner der Kandidaten ist nachweislich sicher - besser als nichts ist
    # trotzdem die erste (kontinuitätsnächste) Lösung, aber der Aufrufer
    # sollte das per Sweep-Analyse/--dry-run zusätzlich prüfen.
    return candidates[0]


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


def _build_travel_waypoints(point: Point, current_joints_rad: dict[str, float]) -> list[dict[str, float]]:
    """Baut die Gelenkziele, die für einen Punkt tatsächlich gesendet werden.

    Bei kartesischen Zielen unterhalb von sim.SAFE_TRAVEL_Z wird zuerst auf
    diese Höhe zurückgezogen (gleiche X/Y wie die aktuelle Pose), dann
    seitlich zum Ziel verfahren, erst dann abgesenkt - statt direkt quer zu
    fahren. Grund: Zwei für sich sichere Punkte können unterschiedliche
    Ellbogen-Konfigurationen brauchen; die reine Gelenkraum-Interpolation
    zwischen ihnen schwingt dann oft tief durch den Tisch (siehe
    programs/beispiel_figur.json, per Sweep-Analyse nachgewiesen - vgl.
    Kommentar bei SAFE_TRAVEL_Z in fr20_ceiling_gazebo.py). Auf der
    Rückzugshöhe bleibt genug Abstand zum Tisch, auch wenn dabei die
    Ellbogen-Konfiguration wechselt.

    Bei Gelenkpose-Zielen (`joints`) oder wenn kaum seitliche Bewegung nötig
    ist, wird direkt zum Ziel gefahren wie bisher.
    """
    if point.xyz is None:
        return [resolve_point(point, current_joints_rad)]

    x, y, z = point.xyz
    if z >= sim.SAFE_TRAVEL_Z - 1e-6:
        return [resolve_point(point, current_joints_rad)]

    current_xyz = kin.forward_kinematics(current_joints_rad)[:3, 3]
    lateral_dist = math.hypot(current_xyz[0] - x, current_xyz[1] - y)
    if lateral_dist < 0.05:
        return [resolve_point(point, current_joints_rad)]

    # resolve_point wählt selbst unter mehreren IK-Kandidaten die per
    # _is_transition_safe nachweislich tischsichere Lösung - der Rückzug auf
    # SAFE_TRAVEL_Z reduziert zusätzlich das Risiko, dass ALLE Kandidaten
    # einer einzigen großen Bewegung unsicher sind, indem er sie in drei
    # kleinere (und damit leichter sicher lösbare) Teilschritte zerlegt.
    #
    # Ein zusätzlicher Versuch, hier ADAPTIV per rekursiver Bisektion
    # (kartesische Zwischenpunkte) nachzubessern, wurde wieder verworfen:
    # bei den hartnäckigen Fällen in programs/beispiel_figur1.json (P7-P11)
    # brachte das keine zuverlässige Verbesserung (teils sogar mehr
    # Kollisionen) und war um Größenordnungen langsamer (>1 Minute statt
    # <1s pro Punkt). Für die verbleibenden bekannten Problemfälle wurden
    # stattdessen von Hand geprüfte sichere Zwischenpunkte in die
    # Programmdatei selbst eingefügt (siehe deren Kommentare/Namen).
    try:
        retract = resolve_point(
            Point(name=f"{point.name} (Rückzug)", xyz=(current_xyz[0], current_xyz[1], sim.SAFE_TRAVEL_Z), rpy_deg=point.rpy_deg),
            current_joints_rad,
        )
        travel = resolve_point(
            Point(name=f"{point.name} (Verfahren)", xyz=(x, y, sim.SAFE_TRAVEL_Z), rpy_deg=point.rpy_deg),
            retract,
        )
        final = resolve_point(point, travel)
    except RuntimeError:
        return [resolve_point(point, current_joints_rad)]
    return [retract, travel, final]


def run_program(
    program: Program,
    publishers: dict,
    loop: bool = False,
    stop_event: "threading.Event | None" = None,
) -> dict[str, float]:
    """Fährt alle Punkte eines Programms nacheinander an.

    `publishers` sind gz-transport-Publisher pro Gelenk, wie sie
    fr20_ceiling_gazebo.send_look_at_table_pose() bzw. robot_gui.py sie
    aufbauen (ein Double-Topic pro Gelenkname aus sim.JOINT_NAMES).

    Ist `stop_event` gesetzt (z.B. threading.Event), wird nach dem aktuellen
    Punkt abgebrochen, sobald das Event gesetzt wird - damit sich zwei
    Programme, die denselben Roboter ansteuern, sauber ablösen können (siehe
    gui/my_gui.py: ein Button-Klick während ein anderes Programm läuft
    bricht es ab und fährt stattdessen sofort die neue Route).

    Gibt die zuletzt gesendete Gelenkpose zurück, damit ein Aufrufer (z.B.
    die GUI für die Jog-Buttons) weiß, von wo aus als nächstes weitergefahren
    werden kann.
    """
    Double = sim.import_gz("msgs", "double_pb2").Double

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
            waypoints = _build_travel_waypoints(point, current)
            for i, target in enumerate(waypoints):
                send(target)
                current = target
                is_last = i == len(waypoints) - 1
                _interruptible_sleep(point.dwell_s if is_last else 0.8, stop_event)
        if not loop or (stop_event is not None and stop_event.is_set()):
            break
        print("Programm-Ende erreicht, starte erneut (--loop) ...")

    return current


def dry_run_program(program: Program) -> bool:
    """Löst alle Punkte eines Programms auf und gibt die Gelenkwinkel aus,
    OHNE Gazebo/ROS 2 zu starten und ohne irgendetwas an einen echten oder
    simulierten Roboter zu senden.

    Nützlich, um neue/geänderte Punkte in einer JSON-Datei schnell zu
    prüfen (IK-Lösbarkeit, Sprünge zwischen Punkten) - genau das, was beim
    Debuggen von programs/beispiel_drehung.json von Hand gemacht wurde,
    hier als fester CLI-Modus. Baut dafür nur das URDF (reines Python,
    kein externes Tool nötig) und nutzt SAFE_STAGING_POSE als Start-Seed,
    wie es die echte run_program()-Ausführung auch täte.

    Gibt True zurück, wenn alle Punkte aufgelöst werden konnten.
    """
    sim.WORK_DIR.mkdir(exist_ok=True)
    urdf_path = sim.WORK_DIR / f"{sim.MODEL_NAME}.urdf"
    sim.write_mount_urdf(urdf_path)

    print(f"[Dry-Run] Löse '{program.name}' auf, ausgehend von SAFE_STAGING_POSE (keine Simulation):\n")
    current = dict(sim.SAFE_STAGING_POSE)
    all_ok = True
    for point in program.points:
        try:
            waypoints = _build_travel_waypoints(point, current)
        except (ValueError, RuntimeError) as exc:
            print(f"  FEHLER bei '{point.name}': {exc}")
            all_ok = False
            continue
        n = len(waypoints)
        for i, solution in enumerate(waypoints):
            if n == 1:
                suffix = ""
            elif i == 0:
                suffix = " (Rückzug)"
            elif i == n - 1:
                suffix = " (Ziel)"
            else:
                suffix = f" (Verfahren {i}/{n - 2})"
            degrees = {j: round(math.degrees(a), 2) for j, a in solution.items()}
            print(f"  '{point.name}'{suffix}: {degrees}")
            current = solution

    print("\nAlle Punkte gültig." if all_ok else "\nMindestens ein Punkt konnte nicht aufgelöst werden.")
    return all_ok


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
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Nur die Punkte auflösen und die Gelenkwinkel ausgeben, ohne Gazebo/ROS 2 zu starten.",
    )
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

    if args.dry_run:
        ok = dry_run_program(program)
        raise SystemExit(0 if ok else 1)

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

        Node = sim.import_gz("transport").Node
        Double = sim.import_gz("msgs", "double_pb2").Double

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
