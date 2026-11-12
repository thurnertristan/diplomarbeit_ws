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

Eigene Programme = eigene JSON-Dateien nach diesem Schema, ohne Code-Änderung.

Start (Beispiel):
    source /opt/ros/jazzy/setup.bash
    python3 robot_program.py programs/beispiel_tisch.json
    python3 robot_program.py programs/beispiel_gelenkpose.json --loop

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


# Alternative Startwerte (j1, j2, j3, j4) für den Fallback in resolve_point, falls die IK vom
# aktuellen Punkt aus nicht konvergiert - decken übliche Ellbogen-Konfigurationen ab, damit
# auch große Sprünge zwischen zwei Punkten eine Lösung finden statt an einem lokalen Minimum
# hängenzubleiben.
_FALLBACK_J1 = (-2.5, -1.25, 0.0, 1.25, 2.5)
_FALLBACK_ELBOWS = ((-1.5, -2.2, -1.2), (-1.5, 2.2, -1.2), (-0.5, -2.6, -1.5), (-0.5, 2.6, -1.5))

_TABLE_CHECK_MARGIN_M = 0.05


def _pose_hits_table(position: np.ndarray) -> bool:
    """True, wenn der Punkt über der Tischfläche und unter der Tischplatte liegt."""
    hx, hy = sim.TABLE_SIZE_XY[0] / 2, sim.TABLE_SIZE_XY[1] / 2
    tx, ty = sim.TABLE_CENTER
    x, y, z = position
    return (
        tx - hx - _TABLE_CHECK_MARGIN_M <= x <= tx + hx + _TABLE_CHECK_MARGIN_M
        and ty - hy - _TABLE_CHECK_MARGIN_M <= y <= ty + hy + _TABLE_CHECK_MARGIN_M
        and z < sim.TABLE_TOP_Z + _TABLE_CHECK_MARGIN_M
    )


def _is_transition_safe(a: dict[str, float], b: dict[str, float], steps: int = 15) -> bool:
    """Prüft per Sweep (lineare Gelenkraum-Interpolation a->b, wie sie die reale gz-sim
    JointPositionController-Bewegung annähert), ob dabei ein Armglied den Tisch schneidet -
    zwei für sich sichere Posen können über einen unsicheren Zwischenweg verbunden sein, wenn
    sich die Ellbogen-Konfiguration dabei stark ändert (siehe resolve_point)."""
    chain = kin.load_chain()
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

    Für kartesische Punkte wird numerische IK mit der aktuellen Pose als Seed verwendet
    (bleibt so meist nahe an der vorherigen Konfiguration). Bei einem zu großen Sprung kann
    diese kontinuitätsbasierte Suche in einem lokalen Minimum hängenbleiben, obwohl der Punkt
    erreichbar wäre - dann wird zusätzlich mit alternativen Ellbogen-Konfigurationen
    (_FALLBACK_ELBOWS/_FALLBACK_J1) gesucht. Unter allen Kandidaten wird die bevorzugt, deren
    Übergang von current_joints_rad aus nachweislich nicht durch den Tisch schwingt (siehe
    _is_transition_safe), statt einfach die erste gefundene Lösung zu nehmen.
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

    # Primärlösung gefunden UND nachweislich sicher: sofort zurückgeben, statt zusätzlich das
    # teure Fallback-Raster durchzurechnen (primary ist durch die Seedwahl praktisch immer die
    # kontinuitätsnächste Lösung).
    if primary is not None and _is_transition_safe(current_joints_rad, primary):
        return primary

    # max_iterations niedriger als der Standard (500): die meisten der bis zu 20
    # Raster-Kombinationen konvergieren ohnehin nicht, eine tatsächlich konvergierende Lösung
    # braucht selten mehr als ~50 Iterationen.
    _FALLBACK_MAX_ITERATIONS = 150
    if allow_fallback:
        for j1 in _FALLBACK_J1:
            for j2, j3, j4 in _FALLBACK_ELBOWS:
                seed = dict(current_joints_rad, j1=j1, j2=j2, j3=j3, j4=j4)
                sol = kin.inverse_kinematics(target, seed, max_iterations=_FALLBACK_MAX_ITERATIONS)
                if sol is not None:
                    candidates.append(sol)
                    if len(candidates) >= 8:  # genug Auswahl
                        break
            if len(candidates) >= 8:
                break

    if not candidates:
        raise RuntimeError(
            f"Keine IK-Lösung für Punkt '{point.name}' ({point.xyz}) gefunden - "
            f"Punkt evtl. außerhalb der Reichweite."
        )

    # Unter allen kollisionssicheren Kandidaten die mit der geringsten Gelenkbewegung zur
    # aktuellen Pose wählen - sonst kann bei mehreren gültigen Ellbogen-Konfigurationen eine
    # weit entfernte gewählt werden, obwohl eine näherliegende auch sicher wäre.
    safe_candidates = [c for c in candidates if _is_transition_safe(current_joints_rad, c)]
    if safe_candidates:
        def joint_delta(c: dict[str, float]) -> float:
            return sum(abs(c[j] - current_joints_rad[j]) for j in sim.JOINT_NAMES)

        return min(safe_candidates, key=joint_delta)

    # Keiner der Kandidaten ist nachweislich sicher - besser als nichts ist die erste
    # (kontinuitätsnächste) Lösung, der Aufrufer sollte das per --dry-run zusätzlich prüfen.
    return candidates[0]


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
    Programmpunkte angewendet (siehe _build_travel_waypoints).

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


def _build_travel_waypoints(
    point: Point,
    current_joints_rad: dict[str, float],
    offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> list[dict[str, float]]:
    """Baut die Gelenkziele, die für einen Punkt tatsächlich gesendet werden.

    Bei kartesischen Zielen unterhalb von sim.SAFE_TRAVEL_Z wird zuerst auf diese Höhe
    zurückgezogen, dann seitlich verfahren, erst dann abgesenkt - statt direkt quer zu fahren.
    Grund: zwei für sich sichere Punkte können unterschiedliche Ellbogen-Konfigurationen
    brauchen, reine Gelenkraum-Interpolation zwischen ihnen schwingt dann oft durch den Tisch.

    Bei Gelenkpose-Zielen oder wenn kaum seitliche Bewegung nötig ist, wird direkt gefahren.

    `offset_xyz` wird - falls ungleich Null - einmal auf den Zielpunkt addiert, bevor
    irgendetwas anderes berechnet wird, damit auch Zwischenschritte an der korrigierten
    Position landen. Gelenkpose-Ziele sind davon unberührt.
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

    # resolve_point wählt selbst die per _is_transition_safe nachweislich tischsichere Lösung -
    # der Rückzug auf SAFE_TRAVEL_Z zerlegt eine große Bewegung zusätzlich in drei kleinere,
    # leichter sicher lösbare Teilschritte (Rückzug hoch, seitlich verfahren, absenken).
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

    `publishers`: gz-transport-Publisher pro Gelenk.
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
        waypoints = _build_travel_waypoints(point, current, offset_xyz=offset_xyz)
        for i, target in enumerate(waypoints):
            _wait_while(pause_event, stop_event, abort_event)
            if stopped():
                print("Programm abgebrochen.")
                return None
            if abort_event is not None and abort_event.is_set():
                return current
            settle_s = _min_settle_time(current, target)
            send(target)
            current = target
            if on_target_sent is not None:
                on_target_sent(current)
            is_last = i == len(waypoints) - 1
            base_wait = point.dwell_s if is_last else 0.8
            _interruptible_sleep(max(base_wait, settle_s), stop_event, abort_event)
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


def dry_run_program(program: Program) -> bool:
    """Löst alle Punkte eines Programms auf und gibt die Gelenkwinkel aus, OHNE Gazebo/ROS 2
    zu starten - nützlich, um neue/geänderte Punkte schnell zu prüfen (IK-Lösbarkeit, Sprünge
    zwischen Punkten). Baut nur das URDF, nutzt SAFE_STAGING_POSE als Start-Seed.

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
