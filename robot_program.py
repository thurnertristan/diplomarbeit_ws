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
from dataclasses import dataclass, field, replace
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


def build_calibration_program() -> Program:
    """Einzelnes Punktprogramm, das sim.CALIBRATION_REFERENCE_XYZ anfährt (den
    markanten Kalibrierpunkt auf der Baugruppe, siehe Kommentar dort) - wird
    vom GUI-Button "Nullpunkt kalibrieren" genutzt, um den Roboter automatisch
    dorthin zu fahren, statt den kompletten Weg per Jog manuell zurücklegen zu
    müssen. Von dort aus wird nur noch die Restabweichung feinjustiert, bevor
    calibrate_table_offset() aufgerufen wird."""
    return Program(
        name="Kalibrierpunkt",
        points=[Point(name="Kalibrierpunkt", xyz=sim.CALIBRATION_REFERENCE_XYZ, dwell_s=0.0)],
    )


def calibrate_table_offset(current_joints_rad: dict[str, float]) -> tuple[float, float, float]:
    """Kalibriert den Tisch-Offset: Aufrufer muss den Roboter vorher (per
    build_calibration_program automatisch angefahren, danach per Jog
    feinjustiert) exakt auf sim.CALIBRATION_REFERENCE_XYZ gebracht haben
    (siehe Kommentar dort - der markante Punkt auf der Baugruppe). Der Offset
    ist die
    Differenz zwischen der per Vorwärtskinematik berechneten tatsächlichen
    Werkzeugposition und dieser bekannten Soll-Position - er wird sofort
    gespeichert (sim.save_table_offset) und danach von allen kartesischen
    Programmpunkten berücksichtigt (siehe _build_travel_waypoints).

    Die Tischhöhe (Z) ändert sich nicht und muss daher nicht neu kalibriert
    werden - dz bleibt immer 0, kalibriert wird nur die X/Y-Lage.

    Gibt den neuen Offset (dx, dy, dz in m) zurück.
    """
    actual_xyz = kin.forward_kinematics(current_joints_rad)[:3, 3]
    dx, dy, _dz = (a - r for a, r in zip(actual_xyz, sim.CALIBRATION_REFERENCE_XYZ))
    offset = (dx, dy, 0.0)
    sim.save_table_offset(*offset)
    return offset


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


def _build_travel_waypoints(
    point: Point,
    current_joints_rad: dict[str, float],
    offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> list[dict[str, float]]:
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

    `offset_xyz` (siehe sim.load_table_offset()) wird - falls ungleich Null -
    einmal auf den kartesischen Zielpunkt addiert, bevor irgendetwas anderes
    berechnet wird: damit landen auch die Rückzug-/Verfahren-Zwischenschritte
    schon an der korrigierten Position, statt am Ende noch einmal seitlich
    nachzukorrigieren. Gelenkpose-Ziele sind davon unberührt (die sind nicht
    tischrelativ gemeint).
    """
    if point.xyz is None:
        return [resolve_point(point, current_joints_rad)]

    if offset_xyz != (0.0, 0.0, 0.0):
        point = replace(point, xyz=tuple(p + o for p, o in zip(point.xyz, offset_xyz)))

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


def _wait_while(event: "threading.Event | None", stop_event: "threading.Event | None") -> None:
    """Blockiert, solange `event` gesetzt ist (z.B. eine Pause), kehrt aber
    sofort zurück, sobald `stop_event` gesetzt wird."""
    while event is not None and event.is_set() and not (stop_event is not None and stop_event.is_set()):
        time.sleep(0.1)


def run_program(
    program: Program,
    publishers: dict,
    loop: bool = False,
    stop_event: "threading.Event | None" = None,
    pause_event: "threading.Event | None" = None,
    current: dict[str, float] | None = None,
    get_current=None,
    on_target_sent=None,
) -> dict[str, float]:
    """Fährt alle Punkte eines Programms nacheinander an.

    `publishers` sind gz-transport-Publisher pro Gelenk, wie sie
    fr20_ceiling_gazebo.launch_simulation() aufbaut (ein Double-Topic pro
    Gelenkname aus sim.JOINT_NAMES).

    Ist `stop_event` gesetzt (z.B. threading.Event), wird nach dem aktuellen
    Punkt abgebrochen, sobald das Event gesetzt wird - damit sich zwei
    Programme, die denselben Roboter ansteuern, sauber ablösen können (siehe
    gui/my_gui.py: ein Button-Klick während ein anderes Programm läuft
    bricht es ab und fährt stattdessen sofort die neue Route).

    Ist `pause_event` gesetzt, wird vor jedem Zwischenschritt gewartet,
    solange es gesetzt bleibt (siehe gui/my_gui.py: Pause/Weiter-Buttons) -
    anders als stop_event bricht das nicht ab, sondern hält nur an.

    `current` ist die als bekannt angenommene Startpose - Default
    SAFE_STAGING_POSE (siehe unten), da gz-sim den Roboter physikalisch
    immer in der Null-Stellung spawnt und jeder Aufrufer vor dem ersten
    Zielpunkt zunächst dorthin fährt (send_to_safe_staging_pose).

    `get_current` (optional) wird vor jedem Punkt aufgerufen, um `current`
    aufzufrischen - Hook für einen Aufrufer mit eigenem, von außen
    veränderlichem Positions-Zustand (z.B. gui/my_gui.py: wird während
    einer Pause gejoggt, macht das Programm beim Fortsetzen genau dort
    weiter, wo der Jog den Arm hingefahren hat, statt die alte Position
    vor der Pause anzunehmen).

    `on_target_sent` (optional) wird nach jedem gesendeten Zwischenschritt
    mit der neuen Pose aufgerufen - Hook für einen Aufrufer, der den
    Fortschritt zusätzlich selbst mitverfolgen will (z.B. die GUI, um
    self._current_joints synchron zu halten).

    Gibt die zuletzt gesendete Gelenkpose zurück.
    """
    Double = sim.import_gz("msgs", "double_pb2").Double
    if current is None:
        current = dict(sim.SAFE_STAGING_POSE)
    offset_xyz = sim.load_table_offset()
    if offset_xyz != (0.0, 0.0, 0.0):
        print(f"Tisch-Offset aktiv: dx={offset_xyz[0]:+.4f} dy={offset_xyz[1]:+.4f} dz={offset_xyz[2]:+.4f} m")

    def send(joint_angles: dict[str, float]) -> None:
        for joint, angle in joint_angles.items():
            msg = Double()
            msg.data = angle
            publishers[joint].publish(msg)

    def stopped() -> bool:
        return stop_event is not None and stop_event.is_set()

    while True:
        for point in program.points:
            _wait_while(pause_event, stop_event)
            if stopped():
                print("Programm abgebrochen.")
                return current
            if get_current is not None:
                current = get_current()
            print(f"-> Punkt '{point.name}'")
            waypoints = _build_travel_waypoints(point, current, offset_xyz=offset_xyz)
            for i, target in enumerate(waypoints):
                _wait_while(pause_event, stop_event)
                if stopped():
                    print("Programm abgebrochen.")
                    return current
                send(target)
                current = target
                if on_target_sent is not None:
                    on_target_sent(current)
                is_last = i == len(waypoints) - 1
                _interruptible_sleep(point.dwell_s if is_last else 0.8, stop_event)
        if not loop or stopped():
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

    offset_xyz = sim.load_table_offset()
    print(f"[Dry-Run] Löse '{program.name}' auf, ausgehend von SAFE_STAGING_POSE (keine Simulation):")
    if offset_xyz != (0.0, 0.0, 0.0):
        print(f"  Tisch-Offset aktiv: dx={offset_xyz[0]:+.4f} dy={offset_xyz[1]:+.4f} dz={offset_xyz[2]:+.4f} m")
    print()
    current = dict(sim.SAFE_STAGING_POSE)
    all_ok = True
    for point in program.points:
        try:
            waypoints = _build_travel_waypoints(point, current, offset_xyz=offset_xyz)
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

    print("Starte Gazebo Jetty (gz-sim) ...")
    gz_proc, publishers = sim.launch_simulation(headless=args.headless)

    try:
        print("Fahre zunächst in die sichere Zwischenpose (Kollisionsvermeidung) ...")
        sim.send_to_safe_staging_pose(publishers)

        run_program(program, publishers, loop=args.loop)

        print("\nProgramm fertig. Strg+C hier drücken, um zu beenden.")
        gz_proc.wait()
    except KeyboardInterrupt:
        print("\nBeende Gazebo ...")
    finally:
        sim.shutdown_simulation(gz_proc)


if __name__ == "__main__":
    main()
