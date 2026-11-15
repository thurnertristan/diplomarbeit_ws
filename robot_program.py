#!/usr/bin/env python3
"""Vorlage: Punktprogramme für den deckenmontierten FR20 abfahren.

Ein "Programm" ist eine benannte Liste von Punkten (siehe `programs/*.json`
für Beispiele). Jeder Punkt ist entweder

  - eine kartesische Zielposition {"x", "y", "z"} (optional "rpy" für die
    Werkzeugorientierung, Grad, Standard: senkrecht nach unten wie
    `fr20_scene.LOOK_AT_TABLE_POSE`), die per inverser Kinematik
    (`fr20_kinematics`) in Gelenkwinkel umgerechnet wird, oder
  - eine Gelenkpose {"joints": {"j1": ..., ..., "j6": ...}} in Grad, die
    direkt verwendet wird.

Eigene Programme = eigene JSON-Dateien nach diesem Schema, ohne Code-Änderung.

Start (Beispiel):
    source /opt/ros/jazzy/setup.bash
    python3 robot_program.py programs/beispiel_tisch.json
    python3 robot_program.py programs/beispiel_gelenkpose.json --loop

Koordinaten: x/y/z = Mitte der Flanschfläche (tool0) in Weltkoordinaten (m), Ursprung
am Boden unter der Deckenhalterung. Jede Bahn wird vor dem Fahren gegen Tisch und Baugruppe
geprüft (fr20_collision) - nicht kollisionsfrei fahrbare Programme starten gar nicht erst.
Prüfen ohne Simulation:
    python3 robot_program.py programs/beispiel_tisch.json --dry-run

Voraussetzungen: siehe Docstring von fr20_scene.py.
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

import fr20_scene as sim
import fr20_collision as col
import fr20_kinematics as kin

PROGRAMS_DIR = Path(__file__).resolve().parent / "programs"

DEFAULT_DWELL_S = 1.0  # Wartezeit an einem Punkt, falls er kein eigenes "dwell_s" angibt.

# Sicherheitsfaktor auf die Gelenk-Geschwindigkeitsgrenze für die Reisezeit-Schätzung
# (die Grenzgeschwindigkeit wird real erst nach einer Beschleunigungsphase erreicht) plus
# Marge für den PD-Regler zum Einschwingen.
_TRAVEL_SPEED_SAFETY = 0.6
_SETTLE_MARGIN_S = 0.4


def _min_settle_time(current: dict[str, float], target: dict[str, float]) -> float:
    """Schätzt, wie lange das langsamste Gelenk von `current` nach `target` (rad) braucht,
    inkl. Bremsmarge (siehe _TRAVEL_SPEED_SAFETY/_SETTLE_MARGIN_S)."""
    travel_s = 0.0
    for joint, target_rad in target.items():
        current_rad = current.get(joint, target_rad)
        delta_rad = abs(target_rad - current_rad)
        v_max = sim.JOINT_VELOCITY_LIMIT_RAD_S.get(joint)
        if v_max:
            travel_s = max(travel_s, delta_rad / (v_max * _TRAVEL_SPEED_SAFETY))
    return travel_s + _SETTLE_MARGIN_S


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
        """Lädt ein Programm aus einer JSON-Datei. Jeder Eintrag in "points" ist entweder
        ein vollständiger Punkt (dict, wie bisher) oder - falls er stattdessen in der
        Punkt-Bibliothek (siehe POINT_LIBRARY_PATH/load_point_library) gepflegt wird - einfach
        nur sein Name (String). So lassen sich mehrere Biegeformen dieselben, einmal
        eingemessenen Punkte teilen, statt sie in jeder Programmdatei zu duplizieren."""
        data = json.loads(path.read_text())
        library = load_point_library()
        points = []
        for entry in data["points"]:
            if isinstance(entry, str):
                if entry not in library:
                    raise KeyError(
                        f"Punkt '{entry}' (referenziert in '{path.name}') ist nicht in der "
                        f"Punkt-Bibliothek ({POINT_LIBRARY_PATH}) vorhanden."
                    )
                points.append(library[entry])
            else:
                points.append(Point.from_dict(entry))
        return Program(name=data.get("name", path.stem), points=points)


def list_programs() -> list[Path]:
    """Alle verfügbaren Programmdateien im `programs/`-Ordner (die Punkt-Bibliothek selbst
    zählt nicht als Programm und wird ausgeschlossen)."""
    if not PROGRAMS_DIR.exists():
        return []
    return sorted(p for p in PROGRAMS_DIR.glob("*.json") if p != POINT_LIBRARY_PATH)


# Punkt-Bibliothek: einmal eingemessene, benannte Punkte, die von mehreren Programmen
# (Biegeformen) gemeinsam per Name referenziert werden können, siehe Program.from_file().
POINT_LIBRARY_PATH = PROGRAMS_DIR / "punkte.json"


def load_point_library() -> dict[str, Point]:
    """Lädt die Punkt-Bibliothek (Name -> Point). Fehlt die Datei, ist die Bibliothek einfach
    leer - Programme, die nur vollständige Punkte statt Namen nutzen, brauchen sie nicht."""
    if not POINT_LIBRARY_PATH.exists():
        return {}
    data = json.loads(POINT_LIBRARY_PATH.read_text())
    return {name: Point.from_dict({**entry, "name": name}) for name, entry in data.items()}


def save_point_to_library(name: str, point: Point) -> None:
    """Speichert (oder überschreibt) einen benannten Punkt in der Punkt-Bibliothek - z.B. vom
    GUI-Button "Position speichern" genutzt, um die aktuelle Armstellung dauerhaft unter einem
    Namen abzulegen, den spätere Biegeformen dann nur noch referenzieren müssen."""
    library_data = json.loads(POINT_LIBRARY_PATH.read_text()) if POINT_LIBRARY_PATH.exists() else {}
    entry: dict = {"dwell_s": point.dwell_s}
    if point.joints_deg is not None:
        entry["joints"] = point.joints_deg
    else:
        entry["xyz"] = list(point.xyz)
        if point.rpy_deg is not None:
            entry["rpy"] = list(point.rpy_deg)
    library_data[name] = entry
    POINT_LIBRARY_PATH.parent.mkdir(exist_ok=True)
    POINT_LIBRARY_PATH.write_text(json.dumps(library_data, indent=2, ensure_ascii=False))


# ---------------------------------------------------------------------------
# Punkt -> Gelenkwinkel
# ---------------------------------------------------------------------------

# Default-Werkzeugorientierung ohne "rpy": Werkzeug-Z-Achse zeigt senkrecht nach unten.
_DEFAULT_TOOL_R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float)


def _rpy_deg_to_matrix(rpy_deg: tuple[float, float, float]) -> np.ndarray:
    r, p, y = (math.radians(v) for v in rpy_deg)
    return kin._rpy_to_matrix(r, p, y)


# ---------------------------------------------------------------------------
# Kollisionsfreie Bahnplanung
# ---------------------------------------------------------------------------
#
# Jede Bewegung zwischen zwei Gelenkposen fährt in der Simulation (und als PTP/MoveJ am
# echten Roboter) die lineare Gelenkraum-Bahn. plan_motion() liefert nur Wegpunkte, deren
# Teilbahnen fr20_collision.is_path_safe() bestanden haben - findet sich keine sichere Bahn,
# wird mit PlanningError abgebrochen, BEVOR sich der Roboter bewegt. (Früher wurde in diesem
# Fall trotzdem die erste IK-Lösung gefahren - genau dann ging es durch den Tisch.)


class PlanningError(RuntimeError):
    """Für einen Punkt gibt es keine kollisionsfreie Lösung bzw. Bahn."""


# Startwerte (j1, j2, j3, j4) für die IK-Suche nach weiteren Lösungen. Bevorzugt werden
# Konfigurationen mit Ellbogen zur Decke (am deckenmontierten Arm hängt der Ellbogen sonst
# Richtung Tisch und der Unterarm taucht in die Baugruppe ein).
_SEED_J1 = (-2.5, -1.25, 0.0, 1.25, 2.5, 0.87)
_SEED_ELBOWS = ((-0.95, -2.3, -1.46), (-1.5, -2.2, -1.2), (-0.5, -2.6, -1.5), (-1.5, 2.2, -1.2), (-0.5, 2.6, -1.5))
_SEED_MAX_ITERATIONS = 150
_MAX_CANDIDATES = 8


def _target_matrix(point: Point, xyz: tuple[float, float, float]) -> np.ndarray:
    target = np.eye(4)
    target[:3, :3] = _rpy_deg_to_matrix(point.rpy_deg) if point.rpy_deg is not None else _DEFAULT_TOOL_R
    target[:3, 3] = xyz
    return target


def _joint_distance(a: dict[str, float], b: dict[str, float]) -> float:
    return sum(abs(a[j] - b[j]) for j in sim.JOINT_NAMES)


def _ik_candidates(
    target: np.ndarray, seed: dict[str, float], boxes, search: bool = True
) -> list[dict[str, float]]:
    """IK-Lösungen für `target`, die selbst kollisionsfrei stehen, sortiert nach Nähe zu `seed`.
    `search=False`: nur die Lösung ausgehend von `seed` (schnell, z.B. für Jog)."""
    found: list[dict[str, float]] = []

    def add(sol: dict[str, float] | None) -> None:
        if sol is None or any(_joint_distance(sol, f) < 1e-3 for f in found):
            return
        if col.is_pose_safe(sol, boxes):
            found.append(sol)

    add(kin.inverse_kinematics(target, seed))
    if search:
        for j2, j3, j4 in _SEED_ELBOWS:
            for j1 in _SEED_J1:
                s = dict(seed, j1=j1, j2=j2, j3=j3, j4=j4)
                add(kin.inverse_kinematics(target, s, max_iterations=_SEED_MAX_ITERATIONS))
                if len(found) >= _MAX_CANDIDATES:
                    break
            if len(found) >= _MAX_CANDIDATES:
                break
    return sorted(found, key=lambda c: _joint_distance(c, seed))


_LIFT_STEP_M = 0.05


def _lift(joints: dict[str, float], boxes) -> list[dict[str, float]]:
    """Senkrechter Rückzug: gleiche Werkzeugorientierung und x/y, Flansch in Schritten von
    _LIFT_STEP_M Richtung SAFE_TRAVEL_Z angehoben. Kleine Schritte, weil eine lineare
    Gelenkraum-Fahrt keine Gerade ist - in einem Zug würde der Flansch nah an der Baugruppe zu
    Beginn kurz absinken.

    Gibt die Zwischenposen (ohne `joints`) zurück, solange jeder Schritt kollisionsfrei ist.
    Nah an der j1-Achse kann der Arm nicht beliebig hoch: um den Flansch dort anzuheben, muss
    er sich falten und der Oberarm schwenkt Richtung Tisch - dann endet die Liste früher."""
    pose = kin.forward_kinematics(joints)
    z0 = pose[2, 3]
    if z0 >= sim.SAFE_TRAVEL_Z - 1e-3:
        return []
    steps = int(np.ceil((sim.SAFE_TRAVEL_Z - z0) / _LIFT_STEP_M))
    path, prev = [], joints
    for z in np.linspace(z0, sim.SAFE_TRAVEL_Z, steps + 1)[1:]:
        pose[2, 3] = z
        nxt = kin.inverse_kinematics(pose, prev)
        if nxt is None or not col.is_path_safe(prev, nxt, boxes):
            break
        path.append(nxt)
        prev = nxt
    return path


def _lift_levels(path: list[dict[str, float]]) -> list[int]:
    """Bis zu vier Rückzugshöhen (Anzahl Schritte), höchste zuerst; 0 = gar nicht anheben."""
    n = len(path)
    return sorted({n, (2 * n) // 3, n // 3, min(n, 2), 0}, reverse=True)


def _park(j1: float) -> dict[str, float]:
    """Sichere Parkpose (Arm zur Decke gefaltet, siehe sim.SAFE_STAGING_POSE) mit gegebenem j1."""
    return dict(sim.SAFE_STAGING_POSE, j1=j1)


def _plan_to_pose(
    start: dict[str, float], goal: dict[str, float], boxes
) -> list[dict[str, float]] | None:
    """Kollisionsfreie Wegpunkte start -> goal oder None. Versucht nacheinander:
      1. direkt,
      2. senkrecht hoch auf SAFE_TRAVEL_Z, oben verfahren, senkrecht absenken,
      3. über die Parkpose (Arm zur Decke gefaltet): hoch, j1 drehen, runter."""
    if col.is_path_safe(start, goal, boxes):
        return [goal]

    def chain_ok(route: list[dict[str, float]]) -> bool:
        prev = start
        for w in route:
            if not col.is_path_safe(prev, w, boxes):
                return False
            prev = w
        return True

    lift_start, lift_goal = _lift(start, boxes), _lift(goal, boxes)
    park = [_park(start["j1"]), _park(goal["j1"])]
    tried: set[tuple[int, int, bool]] = set()
    # 2. senkrecht hoch, auf dieser Höhe verfahren, senkrecht runter - höchste Höhen zuerst.
    # 3. dasselbe, aber oben über die Parkpose (Arm zur Decke gefaltet, j1 drehen).
    for via_park in (False, True):
        for ks in _lift_levels(lift_start):
            for kg in _lift_levels(lift_goal):
                if (ks, kg, via_park) in tried or (ks == kg == 0 and not via_park):
                    continue
                tried.add((ks, kg, via_park))
                up = lift_start[:ks]
                down = list(reversed(lift_goal[:kg])) + [goal]
                route = _dedupe(up + (park if via_park else []) + down)
                if chain_ok(route):
                    return route
    return None


def _dedupe(route: list[dict[str, float]]) -> list[dict[str, float]]:
    """Entfernt direkt aufeinanderfolgende (fast) gleiche Posen."""
    out: list[dict[str, float]] = []
    for w in route:
        if not out or _joint_distance(out[-1], w) > 1e-4:
            out.append(w)
    return out


def resolve_point(
    point: Point,
    current_joints_rad: dict[str, float],
    allow_fallback: bool = True,
    offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> dict[str, float]:
    """Ziel-Gelenkwinkel (rad) für einen Punkt - nur Lösungen innerhalb der Gelenkgrenzen, die
    selbst kollisionsfrei stehen, die nächstgelegene zu `current_joints_rad`. Prüft NICHT die
    Bahn dorthin (dafür plan_motion). Wirft PlanningError, wenn es keine solche Lösung gibt."""
    boxes = col.obstacles(offset_xyz)
    if point.joints_deg is not None:
        goal = {j: math.radians(deg) for j, deg in point.joints_deg.items()}
        c = col.pose_clearance(goal, boxes)
        if not c.ok:
            raise PlanningError(f"Gelenkpose '{point.name}' kollidiert ({c}).")
        return goal
    if point.xyz is None:
        raise ValueError(f"Punkt '{point.name}' hat weder 'joints' noch 'xyz'.")
    xyz = tuple(p + o for p, o in zip(point.xyz, offset_xyz))
    candidates = _ik_candidates(_target_matrix(point, xyz), current_joints_rad, boxes, search=allow_fallback)
    if not candidates:
        raise PlanningError(
            f"Keine kollisionsfreie IK-Lösung für Punkt '{point.name}' {tuple(round(v, 3) for v in xyz)} - "
            f"außerhalb der Reichweite oder zu nah an Tisch/Baugruppe "
            f"(Flansch mind. {col.CLEARANCE_MARGIN_M * 1000:.0f} mm Abstand, Baugruppe-Oberkante z = {sim.ASSEMBLY_TOP_Z:.3f})."
        )
    return candidates[0]


def plan_motion(
    point: Point,
    current_joints_rad: dict[str, float],
    offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> list[dict[str, float]]:
    """Kollisionsfreie Wegpunkte (Gelenkposen, rad) von der aktuellen Pose zum Punkt.

    `offset_xyz` (kalibrierter Tisch-Offset) wird auf kartesische Ziele addiert und verschiebt
    auch Tisch/Baugruppe in der Kollisionsprüfung. Gelenkpose-Ziele bleiben unverändert.
    Wirft PlanningError, wenn es keine sichere Bahn gibt."""
    boxes = col.obstacles(offset_xyz)
    start_c = col.pose_clearance(current_joints_rad, boxes)
    if not start_c.ok:
        raise PlanningError(f"Aktuelle Pose kollidiert bereits ({start_c}) - erst per Jog freifahren.")

    if point.joints_deg is not None:
        goals = [resolve_point(point, current_joints_rad, offset_xyz=offset_xyz)]
    else:
        if point.xyz is None:
            raise ValueError(f"Punkt '{point.name}' hat weder 'joints' noch 'xyz'.")
        xyz = tuple(p + o for p, o in zip(point.xyz, offset_xyz))
        goals = _ik_candidates(_target_matrix(point, xyz), current_joints_rad, boxes)
        if not goals:
            resolve_point(point, current_joints_rad, offset_xyz=offset_xyz)  # wirft mit Erklärung

    for goal in goals:
        route = _plan_to_pose(current_joints_rad, goal, boxes)
        if route is not None:
            return route
    raise PlanningError(
        f"Keine kollisionsfreie Bahn zu Punkt '{point.name}' gefunden - Punkt anpassen "
        f"oder Zwischenpunkt einfügen."
    )


HOME_POINT = Point(name="Nullpunkt", xyz=sim.CALIBRATION_REFERENCE_XYZ, dwell_s=0.0)
"""Wird von run_program() (home_at_end=True) nach dem letzten Punkt automatisch angefahren -
derselbe Kalibrierpunkt wie build_calibration_program(), da er als einziger fester
Referenzpunkt auf der Baugruppe bereits bekannt/erreichbar ist."""


def build_calibration_program() -> Program:
    """Einzelnes Punktprogramm zum markanten Kalibrierpunkt (sim.CALIBRATION_REFERENCE_XYZ) -
    vom GUI-Button "Nullpunkt kalibrieren" genutzt, um den Roboter automatisch dorthin zu
    fahren, statt den Weg per Jog manuell zurücklegen zu müssen."""
    return Program(
        name="Kalibrierpunkt",
        points=[Point(name="Kalibrierpunkt", xyz=sim.CALIBRATION_REFERENCE_XYZ, dwell_s=0.0)],
    )


def calibrate_table_offset(current_joints_rad: dict[str, float]) -> tuple[float, float, float]:
    """Kalibriert den Tisch-Offset: Aufrufer muss den Roboter vorher exakt auf
    sim.CALIBRATION_REFERENCE_XYZ gebracht haben. Der Offset ist die Differenz zwischen der
    per Vorwärtskinematik berechneten tatsächlichen Position und dieser Soll-Position, wird
    sofort gespeichert (sim.save_table_offset) und danach auf alle kartesischen
    Programmpunkte angewendet (siehe plan_motion).

    Die Tischhöhe (Z) ändert sich nicht - dz bleibt immer 0.

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

def _interruptible_sleep(
    seconds: float,
    stop_event: "threading.Event | None",
    abort_event: "threading.Event | None" = None,
) -> None:
    """Wie time.sleep(), kehrt aber sofort zurück, sobald stop_event/abort_event gesetzt wird -
    damit ein Abbruch nicht erst die volle dwell_s eines Punkts abwarten muss."""
    if stop_event is None and abort_event is None:
        time.sleep(seconds)
        return
    deadline = time.time() + seconds
    while time.time() < deadline and not _interrupted(stop_event, abort_event):
        time.sleep(min(0.05, deadline - time.time()))


def _interrupted(stop_event: "threading.Event | None", abort_event: "threading.Event | None" = None) -> bool:
    """True bei hartem Stopp (`stop_event`: Programm wird abgelöst/GUI schließt - sofortiger
    Abbruch ohne Nullpunkt-Rückfahrt) oder sanftem Abbruch (`abort_event`: Button 'Abbruch' -
    überspringt restliche Punkte, fährt aber noch den Nullpunkt an)."""
    return (stop_event is not None and stop_event.is_set()) or (
        abort_event is not None and abort_event.is_set()
    )


def _wait_while(
    event: "threading.Event | None",
    stop_event: "threading.Event | None",
    abort_event: "threading.Event | None" = None,
) -> None:
    """Blockiert, solange `event` gesetzt ist (z.B. Pause), kehrt aber sofort zurück, sobald
    stop_event oder abort_event gesetzt wird."""
    while event is not None and event.is_set() and not _interrupted(stop_event, abort_event):
        time.sleep(0.1)


def _wait_for_step(
    step_event: "threading.Event | None",
    stop_event: "threading.Event | None",
    abort_event: "threading.Event | None" = None,
    on_wait=None,
) -> None:
    """Einzelschrittbetrieb (Entwicklermodus, siehe gui/my_gui.py): blockiert, bis
    `step_event` gesetzt wird ("Nächster Punkt"), und löscht es danach wieder. `step_event=None`
    (Normalbetrieb) kehrt sofort zurück. Kehrt auch bei stop_event/abort_event sofort zurück -
    ein wartendes Einzelschrittprogramm darf sich trotzdem abbrechen lassen.

    `on_wait` (optional): True/False-Callback beim Beginn/Ende des Wartens - Hook für die GUI,
    um den "Nächster Punkt"-Button nur GENAU DANN aktiv zu schalten, wenn er etwas bewirkt."""
    if step_event is None:
        return
    if step_event.is_set():
        step_event.clear()  # schon gesetzt (Restzustand) - kein echtes Warten nötig
        return
    print("Einzelschrittbetrieb: warte auf 'Nächster Punkt' ...")
    if on_wait is not None:
        on_wait(True)
    try:
        while not step_event.is_set():
            if _interrupted(stop_event, abort_event):
                return
            time.sleep(0.05)
        step_event.clear()
    finally:
        if on_wait is not None:
            on_wait(False)


def run_program(
    program: Program,
    publishers: dict,
    loop: bool = False,
    stop_event: "threading.Event | None" = None,
    pause_event: "threading.Event | None" = None,
    current: dict[str, float] | None = None,
    get_current=None,
    on_target_sent=None,
    home_at_end: bool = True,
    step_event: "threading.Event | None" = None,
    abort_event: "threading.Event | None" = None,
    on_step_wait=None,
) -> dict[str, float]:
    """Fährt alle Punkte eines Programms nacheinander an.

    `publishers`: Sollwert-Eingänge pro Gelenk (sim.launch_simulation()).
    `stop_event`: harter, sofortiger Abbruch ohne Nullpunkt-Rückfahrt - so können sich zwei
    Programme am selben Roboter sauber ablösen.
    `pause_event`: hält vor jedem Zwischenschritt an, kein Abbruch.
    `current`: angenommene Startpose, Default SAFE_STAGING_POSE.
    `get_current` (optional): vor jedem Punkt aufgerufen, um `current` von außen aufzufrischen
    (z.B. Jog während einer Pause).
    `on_target_sent` (optional): nach jedem gesendeten Zwischenschritt mit der neuen Pose
    aufgerufen.
    `home_at_end` (Standard True): fährt am Ende zusätzlich HOME_POINT an - entfällt nur bei
    einem harten stop_event-Stopp.
    `step_event` (Einzelschrittbetrieb): wartet vor jedem Punkt auf "Nächster Punkt", anders
    als pause_event also "genau einen Punkt weiterfahren und wieder anhalten".
    `on_step_wait` (optional): True/False-Callback bei Beginn/Ende eines Einzelschritt-Wartens.
    `abort_event` (sanfter Abbruch): überspringt restliche Punkte, fährt aber noch den
    Nullpunkt an. Ein zusätzlicher harter stop_event-Abbruch währenddessen überstimmt sofort.

    Vor der ersten Bewegung wird das GANZE Programm auf kollisionsfreie Bahnen geprüft
    (check_program) - ist ein Punkt nicht sicher erreichbar, wirft run_program PlanningError,
    ohne den Roboter zu bewegen. Jede einzelne Fahrt wird zusätzlich unmittelbar vorher ab der
    tatsächlichen Pose neu geplant (plan_motion), z.B. nach einem Jog während einer Pause.

    Gibt die zuletzt gesendete Gelenkpose zurück.
    """
    if current is None:
        current = dict(sim.SAFE_STAGING_POSE)
    offset_xyz = sim.load_table_offset()
    if offset_xyz != (0.0, 0.0, 0.0):
        print(f"Tisch-Offset aktiv: dx={offset_xyz[0]:+.4f} dy={offset_xyz[1]:+.4f} dz={offset_xyz[2]:+.4f} m")

    start = get_current() if get_current is not None else current
    plans: dict = {}
    print(f"Prüfe '{program.name}' auf kollisionsfreie Bahnen ...")
    problems = check_program(program, start, offset_xyz, home_at_end=home_at_end, plans=plans)
    if problems:
        raise PlanningError(
            f"Programm '{program.name}' wird nicht gestartet - nicht kollisionsfrei fahrbar:\n  "
            + "\n  ".join(problems)
        )

    loop_handle = getattr(next(iter(publishers.values())), "loop", None)

    def send(joint_angles: dict[str, float]) -> None:
        sim.send_joint_pose(joint_angles, publishers)

    def wait_motion(target: dict[str, float], start_pose: dict[str, float], abort) -> dict[str, float] | None:
        """Wartet, bis die Fahrt zu `target` beendet ist. Bei hartem Stopp wird der Arm sofort
        an der aktuellen Stelle angehalten (liegt auf der geprüften Bahn) und None
        zurückgegeben. Ein sanfter Abbruch lässt die laufende (geprüfte) Teilfahrt zu Ende
        fahren."""
        if loop_handle is None:
            _interruptible_sleep(_min_settle_time(start_pose, target), stop_event)
            return None if stopped() else target
        while not loop_handle.is_reached():
            if stopped():
                halted = {j: v for j, v in loop_handle.current().items() if j in sim.JOINT_NAMES}
                loop_handle.set_targets(halted)
                if on_target_sent is not None:
                    on_target_sent(halted)
                return None
            time.sleep(0.03)
        return target

    def stopped() -> bool:
        return stop_event is not None and stop_event.is_set()

    def aborted() -> bool:
        return abort_event is not None and abort_event.is_set()

    def drive_to(
        point: Point,
        current: dict[str, float],
        abort_event: "threading.Event | None" = abort_event,
    ) -> dict[str, float] | None:
        """Fährt die Wegpunkte eines Programmpunkts ab. Gibt bei hartem Stopp None zurück
        (Aufrufer bricht komplett ab), bei sanftem Abbruch die zuletzt erreichte Pose (Aufrufer
        fährt stattdessen noch zum Nullpunkt). Für den Nullpunkt-Anflug selbst wird
        abort_event=None übergeben, damit ein bereits gesetztes abort_event diese letzte Fahrt
        nicht ebenfalls sofort abbricht."""
        if get_current is not None:
            current = get_current()
        print(f"-> Punkt '{point.name}'")
        planned = plans.get(id(point))
        if planned is not None and _joint_distance(planned[0], current) < 1e-6:
            waypoints = planned[1]  # vorab geprüfte Bahn, Start stimmt überein
        else:
            waypoints = plan_motion(point, current, offset_xyz=offset_xyz)  # z.B. nach Jog
        if len(waypoints) > 1:
            print(f"   (kollisionsfreie Umfahrung über {len(waypoints) - 1} Zwischenpose(n))")
        for i, target in enumerate(waypoints):
            _wait_while(pause_event, stop_event, abort_event)
            if stopped():
                print("Programm abgebrochen.")
                return None
            if abort_event is not None and abort_event.is_set():
                return current
            send(target)
            if on_target_sent is not None:
                on_target_sent(target)
            if wait_motion(target, current, abort_event) is None:
                print("Programm abgebrochen.")
                return None
            current = target
            if i == len(waypoints) - 1:
                _interruptible_sleep(point.dwell_s, stop_event, abort_event)
        return current

    while True:  # nur relevant bei loop=True
        for point in program.points:
            _wait_while(pause_event, stop_event, abort_event)
            if stopped():
                print("Programm abgebrochen.")
                return current
            if aborted():
                break  # restliche Punkte überspringen, siehe Nullpunkt-Anflug unten
            _wait_for_step(step_event, stop_event, abort_event, on_wait=on_step_wait)
            if stopped():
                print("Programm abgebrochen.")
                return current
            if aborted():
                break
            result = drive_to(point, current)
            if result is None:
                return current
            current = result
            if aborted():
                break
        if not loop or stopped() or aborted():
            break
        print("Programm-Ende erreicht, starte erneut (--loop) ...")

    if home_at_end and not stopped() and program.points:
        if aborted():
            print("Abbruch: fahre noch zum Nullpunkt, danach Programmende.")
            abort_event.clear()  # Rückfahrt selbst nicht mehr durch abort_event unterbrechen
        _wait_for_step(step_event, stop_event, on_wait=on_step_wait)
        if not stopped():
            result = drive_to(HOME_POINT, current, abort_event=None)
            if result is not None:
                current = result
                print("Programm beendet (Nullpunkt erreicht).")

    return current


def check_program(
    program: Program,
    start: dict[str, float],
    offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    home_at_end: bool = True,
    verbose: bool = False,
    plans: "dict | None" = None,
) -> list[str]:
    """Plant das ganze Programm (inkl. Nullpunkt-Rückfahrt) ab `start` durch, ohne etwas zu
    bewegen. Gibt eine Liste von Fehlermeldungen zurück - leer = alles kollisionsfrei.

    `plans` (optional): wird mit {Punktname: (Startpose, Wegpunkte)} gefüllt, damit
    run_program die geprüften Bahnen direkt fahren kann, statt sie neu zu planen."""
    problems = []
    current = dict(start)
    points = list(program.points) + ([HOME_POINT] if home_at_end and program.points else [])
    for point in points:
        try:
            waypoints = plan_motion(point, current, offset_xyz=offset_xyz)
        except (ValueError, PlanningError) as exc:
            problems.append(f"'{point.name}': {exc}")
            continue
        if plans is not None:
            plans.setdefault(id(point), (dict(current), waypoints))
        if verbose:
            prev = current
            for i, solution in enumerate(waypoints):
                suffix = "" if i == len(waypoints) - 1 else f" (Zwischenpose {i + 1})"
                clearance = col.path_clearance(prev, solution, col.obstacles(offset_xyz))
                degrees = {j: round(math.degrees(a), 1) for j, a in solution.items()}
                print(f"  '{point.name}'{suffix}: {degrees}   min. Abstand {clearance}")
                prev = solution
        current = waypoints[-1]
    return problems


def dry_run_program(program: Program) -> bool:
    """Plant alle Punkte eines Programms inkl. Kollisionsprüfung durch und gibt Gelenkwinkel
    und kleinsten Abstand zu Tisch/Baugruppe aus, OHNE RViz zu starten. Start ist
    SAFE_STAGING_POSE. Gibt True zurück, wenn alles kollisionsfrei fahrbar ist."""
    offset_xyz = sim.load_table_offset()
    print(f"[Dry-Run] Plane '{program.name}' ab SAFE_STAGING_POSE (keine Simulation):")
    if offset_xyz != (0.0, 0.0, 0.0):
        print(f"  Tisch-Offset aktiv: dx={offset_xyz[0]:+.4f} dy={offset_xyz[1]:+.4f} dz={offset_xyz[2]:+.4f} m")
    print()
    problems = check_program(program, dict(sim.SAFE_STAGING_POSE), offset_xyz, verbose=True)
    for problem in problems:
        print(f"  FEHLER {problem}")
    print("\nAlle Punkte kollisionsfrei fahrbar." if not problems else "\nProgramm ist NICHT kollisionsfrei fahrbar.")
    return not problems


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
    parser.add_argument("--headless", action="store_true", help="ohne RViz-Fenster starten")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Nur Bahnen planen und auf Kollisionen prüfen, ohne RViz zu starten.",
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

    print("Starte RViz2 ...")
    sim_proc, publishers = sim.launch_simulation(headless=args.headless)

    try:
        print("Fahre zunächst in die sichere Zwischenpose (Kollisionsvermeidung) ...")
        sim.send_to_safe_staging_pose(publishers)

        run_program(program, publishers, loop=args.loop)

        print("\nProgramm fertig. Strg+C hier drücken, um zu beenden.")
        sim_proc.wait()
    except PlanningError as exc:
        print(f"\n{exc}")
    except KeyboardInterrupt:
        print("\nBeende Simulation ...")
    finally:
        sim.shutdown_simulation(sim_proc)


if __name__ == "__main__":
    main()
