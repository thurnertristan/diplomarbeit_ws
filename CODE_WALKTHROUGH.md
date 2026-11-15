# Code-Walkthrough: Automatisiertes Rohrbiegesystem

Abschnittsweise Erklärung des gesamten Python-Codes im Workspace, mit Codeausschnitt +
einfacher Erklärung inklusive Zusammenhänge. `...` in Codeausschnitten = Code ausgelassen
(nicht echter Python-Code).

---

# Datei 1: `fr20_scene.py`

## Abschnitt 1: Zweck und Konfiguration

```python
MODEL_NAME = "fr20"
WORK_DIR = REPO_ROOT / "generated"          # erzeugtes URDF, nicht im Repo
CEILING_Z = 2.0                              # Montagehöhe der Roboterbasis
TABLE_TOP_Z = 0.75                           # Tischoberkante
ASSEMBLY_TOP_Z = TABLE_TOP_Z + 0.045         # Oberkante der Baugruppe
APPROACH_Z = ASSEMBLY_TOP_Z + 0.05           # sichere Arbeitshöhe des Flanschs
CALIBRATION_REFERENCE_XYZ = (..., ..., APPROACH_Z)
FLANGE_OFFSET_M = 0.12                       # wrist3-Frame -> Flanschfläche
JOINT_LIMITS_RAD = {"j1": (-3.0543, 3.0543), "j2": (-4.6251, 1.4835), ...}
```

Die zentrale Datei der Zelle: Raum, Tisch, Baugruppe, Biegekopf-Testachse, feste Gelenkposen
(`LOOK_AT_TABLE_POSE`, `SAFE_STAGING_POSE`), Gelenkgrenzen und -geschwindigkeiten. Alle
anderen Dateien importieren sie als `sim`.

Wichtig: `FLANGE_OFFSET_M`. Im Original-URDF endet die Kette im wrist3-Frame, die
Flanschfläche liegt aber 12 cm weiter vorne (höchster Punkt im `wrist3_link.STL`). Früher saß
`tool0` im wrist3-Frame. Ein Programmpunkt mit „z = 0.807“ (knapp über der Baugruppe) hat den
Flansch dadurch real auf z = 0.687 gedrückt, also in den Tisch. Jetzt liegt `tool0` auf der
Flanschfläche, und alle x/y/z-Punkte meinen die Flanschmitte.

## Abschnitt 2: Hilfsfunktionen

```python
class JointCommand: data            # Sollwinkel (rad) für publishers[gelenk].publish()
def load_table_offset() / save_table_offset(dx, dy, dz)
def write_mount_urdf(path)          # world -> base_link (kopfüber), wrist3 -> tool0 (+0.12 m)
def send_joint_pose(pose, publishers)   # alle Gelenke atomar setzen
def wait_until_reached(publishers)
def send_to_safe_staging_pose(publishers) / send_look_at_table_pose(publishers)
def launch_simulation(headless=False) -> (proc, publishers)   # -> rviz_backend.launch()
def shutdown_simulation(proc)
```

`write_mount_urdf()` hängt den FR20 per 180°-Drehung um X an die Decke und setzt `tool0`
auf den Flansch. `launch_simulation()` startet RViz2 (siehe Datei 1b) und liefert pro Gelenk
einen „Publisher“ mit `publish(msg)`. Das ist dieselbe Schnittstelle, die GUI, Jog,
`robot_program.py` und `bending_head_motor.py` verwenden.

---

# Datei 1b: `rviz_backend.py`

```python
def write_rviz_urdf(path)       # Roboter + Tisch + Baugruppe + Biegekopf-Testachse
class _JointStateLoop:          # rclpy-Thread, veröffentlicht /joint_states mit 30 Hz
    def set_targets(targets)    # neues Segment: alle Armgelenke SYNCHRON
    def wait_until_reached(timeout_s)
    def current()
class RvizProcess               # robot_state_publisher + rviz2, Popen-ähnlich
def launch(headless=False) -> (RvizProcess, publishers)
```

RViz2 zeigt nur an, es gibt keine Physik. `robot_state_publisher` rechnet aus URDF und
`/joint_states` die TF-Frames. `_JointStateLoop` fährt bei jedem neuen Ziel alle sechs
Armgelenke gleichzeitig los und lässt sie gleichzeitig ankommen; die Dauer bestimmt das
langsamste Gelenk. Damit ist die gefahrene Bahn genau die lineare Gelenkraum-Bahn, die
`fr20_collision` vorher prüft, und entspricht einem PTP/MoveJ am echten Roboter. Vorher fuhr
jedes Gelenk unabhängig mit Maximalgeschwindigkeit, also auf einer ungeprüften Bahn.

---

# Datei 1c: `fr20_collision.py`

```python
CLEARANCE_MARGIN_M = 0.03       # Mindestabstand Arm <-> Hindernis
def obstacles(offset_xyz) -> [Box("Tisch"), Box("Baugruppe"), Box("Biegekopf"), Box("Boden")]
def link_points()               # Punktwolke je Armglied aus den STL-Meshes (2-cm-Raster)
def pose_clearance(joints) -> Clearance(distance_m, link, obstacle)
def path_clearance(a, b) -> Clearance   # entlang der linearen Gelenkraum-Bahn a -> b
def is_pose_safe(joints) / is_path_safe(a, b)
```

Prüft die echte Geometrie aller Armglieder (nicht nur die Gelenkpunkte wie früher) gegen
achsparallele Quader. `path_clearance()` tastet die Bahn adaptiv ab: Ist der Abstand groß,
wird in großen Schritten geprüft (kein Armpunkt kann sich bis zur nächsten Probe weiter
bewegen als der Restabstand), nah am Tisch fein (max. 0.5° pro Gelenk). Ein Werkzeug am
Flansch ist noch nicht modelliert.

---

# Datei 2: `fr20_kinematics.py`

## Abschnitt 1: Docstring, Imports, `_rpy_to_matrix()`

```python
import xml.etree.ElementTree as ET
import numpy as np
import fr20_scene as sim

def _rpy_to_matrix(roll, pitch, yaw) -> np.ndarray:
    rz = ...; ry = ...; rx = ...   # Elementardrehungen um Z, Y, X
    return rz @ ry @ rx
```

Diese Datei kann zwei Dinge: **Vorwärtskinematik** (gegeben Gelenkwinkel → wo ist das
Werkzeug?) und **inverse Kinematik** (gegeben Zielposition → welche Gelenkwinkel?). Statt die
Geometrie von Hand als DH-Parameter abzutippen, wird sie direkt aus dem von Datei 1 erzeugten
URDF ausgelesen — so stimmt sie garantiert mit der Simulation überein.

`_rpy_to_matrix()`: RPY = Roll/Pitch/Yaw, die drei Drehwinkel um X/Y/Z-Achse. Baut daraus eine
3×3-Rotationsmatrix (Standard-Darstellung einer Drehung). `@` ist in Python/numpy die
Matrizenmultiplikation (anders als `*`, das elementweise multipliziert).

## Abschnitt 2: `_origin_transform()` und `_axis_rotation()`

```python
def _origin_transform(origin_elem) -> np.ndarray:
    # liest xyz/rpy aus einem URDF-<origin>-Tag, baut 4x4-Transformationsmatrix

def _axis_rotation(axis_elem, angle) -> np.ndarray:
    # Rodrigues-Formel: Rotation um eine beliebige Achse um `angle`
```

Beide bauen eine 4×4-Transformationsmatrix (Rotation + Verschiebung kombiniert).
`_origin_transform`: feste Verschiebung/Drehung eines Gelenks. `_axis_rotation`: Drehung eines
beweglichen Gelenks um seine eigene Achse. Bausteine für die Gelenkkette.

## Abschnitt 3: `UrdfChain`

```python
class UrdfChain:
    def __init__(self, urdf_path, root_link="world", tip_link="tool0"):
        # Kette root->tip per Tiefensuche finden (ein Link kann mehrere Abzweige haben)
        self.chain = chain
        self.movable_joint_names = [...]

    def forward_kinematics(self, joint_angles) -> np.ndarray:
        t = np.eye(4)
        for joint in self.chain:
            t = t @ _origin_transform(...) @ (_axis_rotation(...) falls beweglich)
        return t
```

Liest das URDF ein und sucht per Tiefensuche den Pfad von `"world"` bis `"tool0"`.
`forward_kinematics()`: multipliziert alle Transformationen der Kette nacheinander durch →
komplette Pose des Werkzeugs.

## Abschnitt 4: `load_chain()` und `forward_kinematics()`

```python
_chain_cache = None

def load_chain() -> UrdfChain:
    global _chain_cache
    if _chain_cache is None:
        # Datei bei Bedarf erzeugen, dann einlesen (nur beim ersten Mal)
        _chain_cache = UrdfChain(urdf_path)
    return _chain_cache

def forward_kinematics(joint_angles) -> np.ndarray:
    return load_chain().forward_kinematics(joint_angles)
```

Liest die Baupläne (URDF) nur **einmal** ein und merkt sich das Ergebnis (`_chain_cache`),
statt bei jedem Aufruf neu zu parsen — spart Zeit bei den vielen IK-Aufrufen.
`forward_kinematics()`: "gegeben diese Gelenkwinkel — wo landet die Werkzeugspitze?"

## Abschnitt 5: `_pose_error()`

```python
def _pose_error(current, target) -> np.ndarray:
    pos_err = target[:3, 3] - current[:3, 3]   # Differenz in Metern
    # Drehungs-Differenz als Achse * Winkel (mit Guard gegen Division durch ~0)
    return np.concatenate([pos_err, rot_err])  # 6 Zahlen
```

Vergleicht zwei Posen und sagt "so weit bist du noch daneben" — 3 Zahlen für Position
(Meter), 3 für Ausrichtung (Winkel). Nötig für die inverse Kinematik, die sich schrittweise
an das Ziel herantastet.

## Abschnitt 6: `inverse_kinematics()`

```python
def inverse_kinematics(target_pose, seed, ...) -> dict | None:
    angles = Startwinkel aus seed
    for _ in range(max_iterations):
        current_pose = forward_kinematics(angles)
        error = _pose_error(current_pose, target_pose)
        if error klein genug: return gefundene Winkel

        jacobian = # kleine Testbewegung pro Gelenk, um Wirkung zu messen
        delta_angles = # daraus berechnen, wie alle Gelenke sich bewegen sollten
        angles += delta_angles  # begrenzt, kein zu großer Sprung
    return None
```

Inverse Kinematik: "Werkzeug soll HIER hin — welche Winkel brauche ich?" Keine einfache
Formel dafür, also tastet sich der Code in kleinen Schritten heran ("wärmer/kälter"): Fehler
messen → für jedes Gelenk einzeln testen, was eine winzige Bewegung bewirkt (Jacobian) →
daraus die beste Gesamtbewegung berechnen → wiederholen, bis nah genug dran oder aufgegeben.
Startet immer bei der aktuellen Stellung (`seed`), damit die Lösung nah an der bisherigen
Armstellung bleibt. Mathematisch komplexester Teil im Projekt — bewusst unverändert gelassen.

---

# Datei 3: `robot_program.py`

## Abschnitt 1: Docstring, Imports, `Point`/`Program`

```python
@dataclass
class Point:
    name: str
    joints_deg: dict | None = None   # feste Gelenkstellung (Grad)
    xyz: tuple | None = None          # oder: Zielposition im Raum (Meter)
    rpy_deg: tuple | None = None      # Werkzeug-Ausrichtung
    dwell_s: float = DEFAULT_DWELL_S  # Wartezeit an diesem Punkt

@dataclass
class Program:
    name: str
    points: list[Point]
```

Fährt "Punktprogramme" ab — eine Liste von Zielpunkten, die der Arm nacheinander anfährt
(aus JSON-Dateien im `programs/`-Ordner). Jeder `Point` ist entweder eine Position im Raum
oder direkt eine fertige Gelenkstellung. `@dataclass` erspart das Schreiben von `__init__`.

## Abschnitt 2: JSON-Dateien einlesen

```python
@staticmethod
def from_dict(d: dict) -> "Point": ...   # JSON-Eintrag -> Point

@staticmethod
def from_file(path: Path) -> "Program":
    data = json.loads(path.read_text())
    points = [Point.from_dict(p) for p in data["points"]]
    return Program(name=..., points=points)

def list_programs() -> list[Path]:
    return sorted(PROGRAMS_DIR.glob("*.json"))
```

Wandeln eine JSON-Datei in echte `Point`/`Program`-Objekte um. `list_programs()` listet alle
`.json`-Dateien im `programs/`-Ordner auf.

## Abschnitt 3: Kollisionsfreie Bahnplanung (`plan_motion()`)

```python
class PlanningError(RuntimeError)

def _ik_candidates(target, seed, boxes, search=True):
    # IK ab seed + Raster aus j1-Werten und Ellbogen-Stellungen (Ellbogen zur Decke zuerst)
    # -> nur Lösungen, die selbst kollisionsfrei stehen, sortiert nach Nähe zu seed

def _plan_to_pose(start, goal, boxes):
    1. direkt, wenn is_path_safe(start, goal)
    2. senkrecht hoch (_lift: 5-cm-Schritte, so hoch wie sicher möglich), verfahren, absenken
    3. wie 2., aber oben über die Parkpose (SAFE_STAGING_POSE mit passendem j1)
    sonst None

def plan_motion(point, current, offset_xyz) -> [Wegpunkte]   # oder PlanningError
def resolve_point(point, current, allow_fallback=True, offset_xyz=...) -> Gelenkwinkel
```

Übersetzt „wohin soll der Arm“ in eine Folge von Gelenkposen, bei der **jede** Teilbahn per
`fr20_collision.is_path_safe()` geprüft ist. Gibt es keine sichere Lösung, wird mit
`PlanningError` abgebrochen, bevor sich der Roboter bewegt. Früher wurde in diesem Fall
trotzdem die erste IK-Lösung gefahren, und genau dann ging es durch den Tisch.
`resolve_point()` liefert nur eine einzelne kollisionsfreie Zielpose (für den Jog, der den
kurzen Weg dorthin anschließend selbst prüft).

## Abschnitt 6: `HOME_POINT`, `build_calibration_program()`, `calibrate_table_offset()`

```python
HOME_POINT = Point(name="Nullpunkt", xyz=sim.CALIBRATION_REFERENCE_XYZ, dwell_s=0.0)

def build_calibration_program() -> Program:
    return Program(name="Kalibrierpunkt", points=[Point(..., xyz=sim.CALIBRATION_REFERENCE_XYZ)])

def calibrate_table_offset(current_joints_rad) -> tuple:
    actual_xyz = kin.forward_kinematics(current_joints_rad)[:3, 3]
    dx, dy, _ = actual_xyz - sim.CALIBRATION_REFERENCE_XYZ
    sim.save_table_offset(dx, dy, 0.0)
    return (dx, dy, 0.0)
```

`HOME_POINT`: der "Nullpunkt", den der Arm am Ende eines Programms anfährt.
`build_calibration_program()`: Mini-Programm mit nur diesem Punkt (Kalibrier-Button).
`calibrate_table_offset()`: vergleicht Ist- mit Soll-Position, speichert die Differenz als
Korrekturwert.

## Abschnitt 7: `_interruptible_sleep()`

```python
def _interruptible_sleep(seconds, stop_event, abort_event=None) -> None:
    ...  # wie time.sleep(), kehrt aber sofort zurück bei Abbruch

```

`_interruptible_sleep`: abbrechbares Warten.

## Abschnitt 8: `_interrupted()`, `_wait_while()`, `_wait_for_step()`

```python
def _interrupted(stop_event, abort_event=None) -> bool:
    return stop_event gesetzt ODER abort_event gesetzt

def _wait_while(event, stop_event, abort_event=None) -> None:
    ...  # wartet z.B. während "Pause" aktiv ist, bricht sofort bei stop/abort

def _wait_for_step(step_event, stop_event, abort_event=None, on_wait=None) -> None:
    ...  # Einzelschrittbetrieb: wartet auf "Nächster Punkt"-Klick
```

Drei Warte-Helfer: `_interrupted` = einfache Ja/Nein-Frage. `_wait_while` = Pause-Funktion
(macht automatisch weiter, sobald Pause aufgehoben wird). `_wait_for_step` =
Einzelschrittbetrieb (nur Entwicklermodus, wartet auf aktiven Klick).

## Abschnitt 9: `run_program()` — Teil 1 (Aufbau + `drive_to()`)

```python
def run_program(program, publishers, loop=False, stop_event=None, pause_event=None,
                 current=None, get_current=None, on_target_sent=None,
                 home_at_end=True, step_event=None, abort_event=None, on_step_wait=None):
    if current is None:
        current = dict(sim.SAFE_STAGING_POSE)
    offset_xyz = sim.load_table_offset()

    def drive_to(point, current, abort_event=...):
        waypoints = plan_motion(point, current, offset_xyz=offset_xyz)  # kollisionsgeprüft
        for target in waypoints:
            _wait_while(pause_event, ...)   # ggf. Pause abwarten
            send(target)                     # Winkel an RViz schicken
            _interruptible_sleep(...)         # warten bis Bewegung fertig
        return current
```

Die zentrale Funktion, die ein ganzes Punktprogramm abfährt (viele Parameter für alle
Steuermöglichkeiten: Pause, Abbruch, Einzelschritt, Fortschritts-Callback). `drive_to()`
(innere Funktion): fährt genau einen Programmpunkt ab — holt die sicheren Zwischenschritte
(Abschnitt 7), sendet sie nacheinander, wartet dazwischen.

## Abschnitt 10: `run_program()` — Teil 2 (Hauptschleife + Nullpunkt-Anflug)

```python
    def stopped(): return stop_event is not None and stop_event.is_set()
    def aborted(): return abort_event is not None and abort_event.is_set()

    while True:  # nur relevant bei loop=True
        for point in program.points:
            _wait_while(pause_event, stop_event, abort_event)
            if stopped(): return current
            if aborted(): break
            _wait_for_step(step_event, stop_event, abort_event, on_wait=on_step_wait)
            if stopped(): return current
            if aborted(): break
            result = drive_to(point, current)
            if result is None: return current
            current = result
            if aborted(): break
        if not loop or stopped() or aborted():
            break

    if home_at_end and not stopped() and program.points:
        if aborted():
            abort_event.clear()  # Rückfahrt selbst nicht mehr unterbrechen
        result = drive_to(HOME_POINT, current, abort_event=None)
        if result is not None:
            current = result

    return current
```

Die eigentliche Hauptschleife: geht alle Punkte des Programms der Reihe nach durch, ruft für
jeden `drive_to()` (Teil 1) auf, prüft dabei ständig auf Pause/Abbruch/Einzelschritt. Bei
`loop=True` fängt das Programm nach dem letzten Punkt wieder von vorne an. Ganz am Ende (wenn
`home_at_end=True`) wird zusätzlich noch der Nullpunkt angefahren, bevor die Funktion
zurückkehrt — auch nach einem sanften Abbruch (`abort_event`), aber NICHT nach einem harten
Stopp (`stop_event`), da der Umweg dort nur stören würde. Das `abort_event=None` beim
Nullpunkt-Anflug verhindert, dass ein bereits gesetztes Abbruch-Signal diese letzte,
gewünschte Fahrt selbst wieder abbricht.

## Abschnitt 11: `check_program()` und `dry_run_program()`

```python
def check_program(program, start, offset_xyz, home_at_end=True, verbose=False) -> [Fehler]:
    for point in program.points + [HOME_POINT]:
        waypoints = plan_motion(point, current, offset_xyz)   # PlanningError -> Fehlerliste
        current = waypoints[-1]

def dry_run_program(program) -> bool:
    problems = check_program(program, SAFE_STAGING_POSE, verbose=True)
```

`check_program()` plant das ganze Programm inklusive Nullpunkt-Rückfahrt durch, ohne etwas zu
bewegen. `run_program()` ruft es vor der ersten Bewegung auf: Ist auch nur ein Punkt nicht
kollisionsfrei erreichbar, startet das Programm gar nicht (in der GUI erscheint dann ein
Warnfenster „Kollisionsgefahr“). `--dry-run` gibt pro Wegpunkt die Gelenkwinkel und den
kleinsten Abstand zu Tisch/Baugruppe aus, ohne RViz zu starten.

## Abschnitt 12: CLI / `main()`

```python
def main() -> None:
    parser.add_argument("program", nargs="?", help="Pfad zu einer Programmdatei (JSON)")
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    if args.program is None:
        # keine Datei angegeben -> nur verfügbare Programme auflisten
        return

    program = Program.from_file(Path(args.program))

    if args.dry_run:
        ok = dry_run_program(program)
        raise SystemExit(0 if ok else 1)

    sim_proc, publishers = sim.launch_simulation(headless=args.headless)
    try:
        sim.send_to_safe_staging_pose(publishers)
        run_program(program, publishers, loop=args.loop)
        sim_proc.wait()
    except KeyboardInterrupt:
        ...
    finally:
        sim.shutdown_simulation(sim_proc)
```

Der Einstiegspunkt beim direkten Start (`python3 robot_program.py programs/....json`).
Derselbe Aufbau wie schon in `fr20_scene.main()` (Datei 1): Simulation
starten, sicher zur Zwischenpose, Programm abfahren, sauber beenden per `finally`. `raise
SystemExit(0 if ok else 1)` bei `--dry-run`: gibt einen Exit-Code zurück (0 = alles okay,
1 = Fehler) — nützlich, wenn man das automatisiert aufruft und das Ergebnis auswerten will.

**Damit ist `robot_program.py` komplett durch.**

---

# Datei 4: `bending_head_motor.py`

## Abschnitt 1: Docstring + Sicherheitskonstanten

```python
DRIVETRAIN_CAPACITY_NM = 225.0    # Antriebsstrang-Kapazität
MEASURED_BENDING_MOMENT_NM = 230.0  # gemessenes Biegemoment

TORQUE_LIMIT_NM = min(DRIVETRAIN_CAPACITY_NM, MEASURED_BENDING_MOMENT_NM)
```

Diese Datei steuert den **einen echten Motor** im System an — den Servo, der den Biegekopf
antreibt (Panasonic MSMF082L1U1 + Verstärker MCDLN35SG). Nach demselben Backend-Muster wie
überall im Projekt: eine Klasse (`BendingHeadAxis`) kennt nur "fahre um X weiter", ein
austauschbares "Backend" übernimmt die tatsächliche Kommunikation mit der Hardware.

Wichtig zum Verstehen der Konstanten: `225.0` und `230.0` liegen SEHR nah beieinander — das
gemessene Biegemoment ist beinahe größer als das, was der Antriebsstrang eigentlich sicher
kann. Die Sicherheitsmarge ist praktisch null. `TORQUE_LIMIT_NM` nimmt deshalb bewusst den
**kleineren** der beiden Werte (`min(...)`), damit der Antriebsstrang nicht überlastet wird —
auch wenn das bedeutet, dass der Motor eventuell nicht das volle nötige Drehmoment erreicht.
Das ist als Warnung im Code markiert: eine echte Lösung (stärkeres Getriebe/Motor) muss vor
dem echten Betrieb noch gefunden werden.

## Abschnitt 2: `BendingBackend` (Schnittstelle) und `SimulatedBendingBackend`

```python
class BendingBackend(Protocol):
    def connect(self, torque_limit_nm: float) -> None: ...
    def move_by(self, delta: float) -> None: ...

class SimulatedBendingBackend:
    def connect(self, torque_limit_nm): print(f"[Simulation] ... {torque_limit_nm} Nm")
    def move_by(self, delta): print(f"[Simulation] ... {delta} weiter")
```

`Protocol` ist Pythons Art, eine **Schnittstelle** zu definieren — sagt nur "jedes Backend
muss `connect()` und `move_by()` haben", ohne selbst etwas zu tun. `SimulatedBendingBackend`
ist die simpelste Umsetzung davon: loggt nur, was gesendet würde, ohne echte Hardware
anzusprechen — zum gefahrlosen Testen der Ablauflogik.

## Abschnitt 3: `_encode_int32()`

```python
def _encode_int32(value: int) -> tuple[int, int]:
    raw = value & 0xFFFFFFFF
    return raw & 0xFFFF, (raw >> 16) & 0xFFFF
```

Kleiner Hilfsbaustein für Modbus (das Kommunikationsprotokoll zum Verstärker, siehe unser
Gespräch dazu in dieser Session): Modbus-Register sind immer 16 Bit groß, aber manche Werte
(z.B. Positionen) brauchen 32 Bit. Diese Funktion zerlegt einen 32-Bit-Wert in zwei
16-Bit-Häften — das niederwertige Register zuerst (so verlangt es das Handbuch des
MCDLN35SG). `value & 0xFFFFFFFF` sorgt dafür, dass auch negative Zahlen korrekt als
"Zweierkomplement" (die übliche Computer-Darstellung negativer Zahlen) behandelt werden.

## Abschnitt 4: `ModbusRTUBendingBackend` — Register-Adressen

```python
@dataclass
class ModbusRTUBendingBackend:
    port: str = "/dev/ttyUSB0"
    baudrate: int = 115200
    ...
    REGISTER_MODBUS_COMM_RIGHTS = 0x4300
    REGISTER_BLOCK_OPERATION_SELECT = 0x3000 + 6 * 0x100 + 28 * 2
    REGISTER_TORQUE_LIMIT_1 = 0x3000 + 0 * 0x100 + 13 * 2
    REGISTER_TORQUE_LIMIT_2 = 0x3000 + 5 * 0x100 + 22 * 2
    REGISTER_BLOCK_VELOCITY_0 = 0x4600
    REGISTER_BLOCK_0_COMMAND = 0x4800
    REGISTER_BLOCK_NUMBER = 0x4414
    COIL_SERVO_ON = 0x0060
    COIL_STROBE = 0x0120
    COIL_BUSY = 0x0140

    command_units_per_step: float | None = None   # TODO, noch nicht bekannt
    rated_torque_nm: float | None = None            # TODO, noch nicht bekannt
```

Das ist das Backend für die **echte Hardware** — die Adressen stammen aus dem offiziellen
Panasonic-Handbuch (das wir in dieser Session zusammen durchsucht haben). Wichtig zu
verstehen: dieser Verstärker hat kein einzelnes "fahre auf Position X"-Register, sondern
arbeitet mit vorprogrammierten "Blöcken" (Befehl + Ziel + Geschwindigkeit), die man auswählt
und auslöst (Details dazu gleich in `move_by()`).

Zwei Werte sind bewusst noch `None`: `command_units_per_step` (Umrechnung von Bewegungseinheit
in die vom Verstärker erwarteten "command units", hängt von einer Getriebeeinstellung ab, die
noch nicht ausgelesen wurde) und `rated_torque_nm` (Nenndrehmoment des Motors aus dem
Datenblatt, für die Drehmoment-Limit-Umrechnung nötig). Beide werden absichtlich NICHT
geraten — siehe nächster Abschnitt.

## Abschnitt 5: `ModbusRTUBendingBackend.connect()`

```python
def connect(self, torque_limit_nm: float) -> None:
    if self.rated_torque_nm is None:
        raise NotImplementedError("rated_torque_nm ist nicht gesetzt - ...")

    self._write_registers(self.REGISTER_MODBUS_COMM_RIGHTS, [0x0055])       # Rechte holen
    self._write_registers(self.REGISTER_BLOCK_OPERATION_SELECT, ...)         # Block-Betrieb an

    torque_limit_pct_x10 = round(torque_limit_nm / self.rated_torque_nm * 1000)
    self._write_registers(self.REGISTER_TORQUE_LIMIT_1, ...)
    self._write_registers(self.REGISTER_TORQUE_LIMIT_2, ...)

    self._write_coil(self.COIL_SERVO_ON, True)   # Servo erst NACH Limit einschalten
```

Baut die Verbindung auf und setzt dabei **zuerst** das Drehmoment-Limit, bevor der Servo
überhaupt eingeschaltet wird (`COIL_SERVO_ON` ganz am Ende) — wichtig wegen der minimalen
Sicherheitsmarge aus Abschnitt 1. Fehlt `rated_torque_nm`, bricht die Funktion sofort mit
einer klaren Fehlermeldung ab (`NotImplementedError`), statt mit einem geratenen Wert
weiterzumachen und den Motor eventuell falsch zu begrenzen.

## Abschnitt 6: `ModbusRTUBendingBackend.move_by()`

```python
def move_by(self, delta: float) -> None:
    if self.command_units_per_step is None:
        raise NotImplementedError(...)
    travel_command_units = round(delta * self.command_units_per_step)

    # Geschwindigkeit/Beschleunigung für "Block 0" setzen
    self._write_registers(self.REGISTER_BLOCK_VELOCITY_0, [...])
    ...

    # Block 0 = "Relative positioning"-Befehl mit dem Zielwert befüllen
    command_word = ...
    self._write_registers(self.REGISTER_BLOCK_0_COMMAND, [...])

    # Block auswählen und per Strobe-Signal auslösen
    self._write_registers(self.REGISTER_BLOCK_NUMBER, [0])
    self._write_coil(self.COIL_STROBE, False)
    self._write_coil(self.COIL_STROBE, True)   # OFF->ON-Flanke startet die Bewegung

    # warten, bis der Verstärker meldet "fertig" (BUSY-Bit wird 0)
    while ...:
        if self._read_busy(): continue
        return
    raise TimeoutError(...)
```

Die eigentliche Bewegung: befüllt einen vorprogrammierten "Bewegungssatz" (Block 0) mit dem
Befehl "fahre relativ um X weiter", wählt ihn aus, und löst ihn durch einen kurzen
Aus-Ein-Wechsel des Strobe-Signals aus (eine "Flanke" — Motoren/Verstärker reagieren oft auf
den Wechsel eines Signals, nicht auf seinen Zustand). Danach wird gewartet, bis der Verstärker
"fertig" meldet (`BUSY`-Bit wird 0), mit Timeout als Absicherung, falls das nie passiert.

## Abschnitt 7: `RvizBendingBackend`

```python
class RvizBendingBackend:
    def __init__(self, publishers, start_position_deg=0.0):
        self._current_rad = math.radians(start_position_deg)
        sim.send_joint_pose({"bending_head": self._current_rad}, publishers)

    def connect(self, torque_limit_nm): print(f"[RViz] ... (gilt hier nur real)")

    def move_by(self, delta):
        self._current_rad += math.radians(delta)
        # neue absolute Position an RViz senden
```

Das Backend, das wir in dieser Session gemeinsam gebaut und getestet haben — steuert die
Test-Achse aus Datei 1b (`write_rviz_urdf`) in RViz an, statt echte Hardware. Wichtiger Unterschied
zu den anderen Backends: der Simulation erwartet immer eine **absolute** Zielposition, kein
"fahre um X weiter" — deshalb führt dieses Backend die absolute Position selbst nach
(`_current_rad`, wird bei jedem `move_by()` weiter aufaddiert).

## Abschnitt 8: `BendingHeadAxis`

```python
class BendingHeadAxis:
    def __init__(self, backend, torque_limit_nm=TORQUE_LIMIT_NM, start_position=0.0):
        self._backend = backend
        backend.connect(torque_limit_nm)
        self.current_position = start_position
        self._lock = threading.Lock()

    def move_by(self, delta: float) -> float:
        with self._lock:
            self._backend.move_by(delta)
            self.current_position += delta
            return self.current_position
```

Das ist die Klasse, mit der der restliche Code (GUI, Tests) tatsächlich arbeitet — egal
welches Backend dahintersteckt. Hält die aktuelle Position rein rechnerisch nach (keine
Rückmeldung vom echten Motor, "open loop"). `self._lock` (ein `threading.Lock`) sorgt dafür,
dass zwei Threads (z.B. der Testlauf-Worker UND ein gleichzeitiger Jog-Klick) sich beim
Aktualisieren von `current_position` nicht gegenseitig ins Gehege kommen.

## Abschnitt 9: `run_bending_test_sequence()`

```python
def run_bending_test_sequence(axis, steps, stop_event=None, pause_event=None,
                               on_position_changed=None, settle_s=None,
                               dwell_s=0.5, step_size=20.0) -> float:
    for delta in steps:
        n = ceil(abs(delta) / step_size)
        target = axis.current_position + delta   # Ziel EINMAL fest berechnen

        for i in range(n):
            if pausiert oder gestoppt: return / weiter warten

            remaining_steps = n - i
            increment = (target - axis.current_position) / remaining_steps  # <- der Fix!
            axis.move_by(increment)

            wait_s = settle_s(increment) if settle_s else 0.0
            warten bis wait_s um ist
    return axis.current_position
```

Fährt eine Liste relativer Testschritte in vielen kleinen Zwischenschritten ab (nicht als
einen großen Sprung), damit Pause/Abbruch regelmäßig geprüft werden können. Der interessante
Teil, den wir in dieser Session gemeinsam gefixt haben: das Ziel wird **einmal fest**
berechnet (`target`), und bei JEDEM Zwischenschritt wird die Restdistanz `target -
axis.current_position` neu ausgerechnet — statt einfach immer denselben festen Schritt zu
addieren. Grund: wird während einer Pause per Jog-Button manuell nachgefahren, würde ein
fester Schritt das Endergebnis um genau den Jog-Betrag verschieben. Mit der Neuberechnung
gleicht sich das automatisch aus, egal wie oft/viel dazwischen gejoggt wurde.

## Abschnitt 10: `main()` (CLI)

```python
def main() -> None:
    parser.add_argument("delta", type=float)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    mode.add_argument("--simulate", action="store_true")
    mode.add_argument("--rviz", action="store_true")
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    if args.rviz:
        sim_proc, publishers = sim.launch_simulation()
        axis = BendingHeadAxis(RvizBendingBackend(publishers))
        axis.move_by(args.delta)
        sim_proc.wait()
        return

    backend = SimulatedBendingBackend() if args.simulate else ModbusRTUBendingBackend(port=args.port)
    axis = BendingHeadAxis(backend)
    axis.move_by(args.delta)
```

Drei Betriebsarten über Kommandozeilen-Flags: `--simulate` (nur loggen), `--rviz` (Test in
der Simulation, genau das haben wir in dieser Session mehrfach so aufgerufen), oder ohne Flag
= echte Hardware über `--port`. `mode.add_mutually_exclusive_group()` (nicht extra gezeigt,
aber im echten Code) sorgt dafür, dass man nicht versehentlich `--simulate` und `--rviz`
gleichzeitig angeben kann.

**Damit ist `bending_head_motor.py` komplett durch.**

---

# Datei 5: `biegekraft_rechner.py`

## Abschnitt 1: Docstring + `MATERIALS_MPA`

```python
MATERIALS_MPA = {
    "S235": 235, "S355": 355, "1.4301": 215, "1.4571": 220,
    "AlMg3": 130, "AlMgSi1_T6": 260, "Cu-DHP": 60,
}
```

Ein eigenständiges, von der Simulation komplett unabhängiges Rechen-Tool: schätzt ab, wie viel
Biegemoment nötig ist, um ein bestimmtes Rohr zu biegen, und daraus, wie viel Drehmoment der
Motor an seiner Welle bräuchte — als Entscheidungsgrundlage für die Motor-/Getriebeauswahl,
BEVOR feststeht, welche Hardware überhaupt gekauft wird. Der Docstring warnt ausdrücklich: das
ist eine vereinfachte **Handrechnung**, keine FEM-Simulation (FEM = Finite-Elemente-Methode,
eine sehr viel genauere, aber aufwendigere Berechnungsmethode) — sie ignoriert Reibung,
Kaltverfestigung (Material wird beim Biegen härter) und Rückfederung. `MATERIALS_MPA`: eine
Nachschlagetabelle für die Fließgrenze (ab welcher Belastung sich ein Material dauerhaft
verformt) gängiger Werkstoffe in MPa (= N/mm²).

## Abschnitt 2: `BendingEstimate` und `bending_estimate()`

```python
@dataclass
class BendingEstimate:
    outer_diameter_mm: float
    ...
    plastic_moment_nm: float

    def motor_shaft_torque_nm(self, gear_ratio, efficiency=0.8, safety_factor=1.5) -> float:
        output_torque = self.plastic_moment_nm * safety_factor
        return output_torque / (gear_ratio * efficiency)

def bending_estimate(outer_diameter_mm, wall_thickness_mm, yield_strength_mpa) -> BendingEstimate:
    d_a = outer_diameter_mm
    d_i = outer_diameter_mm - 2 * wall_thickness_mm

    w_elastic = math.pi / 32 * (d_a**4 - d_i**4) / d_a   # elastisches Widerstandsmoment
    z_plastic = (d_a**3 - d_i**3) / 6                     # plastisches Widerstandsmoment

    m_elastic_nm = yield_strength_mpa * w_elastic / 1000.0
    m_plastic_nm = yield_strength_mpa * z_plastic / 1000.0
    return BendingEstimate(...)
```

**Einfach erklärt:** Ein Rohr biegen heißt, seinen Querschnitt (einen Ring) so weit zu
verbiegen, bis er dauerhaft verformt bleibt. `bending_estimate()` rechnet dafür zwei Werte
nach Standard-Biegeformeln aus:
- **elastisches Biegemoment**: ab wann das Material anfängt, sich zu verformen, aber noch in
  seine Ausgangsform zurückfedern würde (wie ein Gummiband).
- **vollplastisches Biegemoment**: ab wann der GANZE Querschnitt dauerhaft verformt ist — das
  ist der praktisch relevante Wert fürs tatsächliche Rohrbiegen.

`d_a`/`d_i` sind Außen-/Innendurchmesser (Innendurchmesser = außen minus zweimal Wandstärke).
Die Formeln selbst (`w_elastic`, `z_plastic`) sind Standard-Ingenieurformeln für einen runden
Hohlquerschnitt, nicht selbst hergeleitet.

`motor_shaft_torque_nm()`: rechnet vom Biegemoment am Rohr aufs nötige Motor-Wellendrehmoment
um, unter Berücksichtigung von Getriebeübersetzung (`gear_ratio` — je größer, desto weniger
Motordrehmoment nötig, aber langsamer), Wirkungsgrad (`efficiency` — ein Getriebe "verliert"
immer etwas Kraft durch Reibung) und einem Sicherheitsfaktor (`safety_factor`, deckt Dinge ab,
die diese vereinfachte Rechnung nicht erfasst).

## Abschnitt 3: `main()` (CLI)

```python
def main() -> None:
    parser.add_argument("--od", type=float, required=True)     # Außendurchmesser
    parser.add_argument("--wall", type=float, required=True)   # Wandstärke
    material_group.add_argument("--material", choices=sorted(MATERIALS_MPA))
    material_group.add_argument("--yield-mpa", type=float)     # oder direkt die Fließgrenze
    parser.add_argument("--gear-ratio", type=float, default=1.0)
    parser.add_argument("--efficiency", type=float, default=0.8)
    parser.add_argument("--safety-factor", type=float, default=1.5)
    args = parser.parse_args()

    estimate = bending_estimate(args.od, args.wall, yield_mpa)
    motor_torque = estimate.motor_shaft_torque_nm(args.gear_ratio, args.efficiency, args.safety_factor)
    print(...)  # alle Zwischenwerte + Ergebnis ausgeben
```

Reines Kommandozeilen-Tool — kein GUI-Zugriff, keine Simulation, nur Zahlen rein, Zahlen raus.
`material_group` (eine sich gegenseitig ausschließende Argumentgruppe): man gibt entweder
`--material S235` (nutzt die Tabelle aus Abschnitt 1) ODER `--yield-mpa 250` (eigener Wert)
an, nicht beides gleichzeitig. Aufrufbeispiel aus dem Docstring:
`python3 biegekraft_rechner.py --od 20 --wall 1.5 --material S235 --gear-ratio 50`.

**Damit ist `biegekraft_rechner.py` komplett durch.**

---

# Datei 6: `gui/` — die Bedienoberfläche

Die GUI ist in mehrere Dateien aufgeteilt (haben wir in dieser Session gemeinsam so
umgebaut): `my_gui.py` ist nur noch der "Zusammenbauer", die eigentliche Fachlogik steckt in
sogenannten **Mixins** — kleinen, thematisch fokussierten Klassen, von denen `MyApp` erbt.
Warum das funktioniert: die Bibliothek `pygubu` (liest das grafische Oberflächen-Layout aus
`my_gui.ui` ein) sucht per `connect_callbacks(self)` alle Methoden, die im Designer als
"Knopf X ruft Methode Y" benannt sind — egal, in welcher Datei diese Methode ursprünglich
geschrieben wurde, solange sie am Ende auf `self` (also der einen `MyApp`-Instanz) landet.
Mehrfachvererbung (`class MyApp(StyleMixin, JogMixin, ...)`) macht genau das möglich, ohne den
Code zu duplizieren.

## Abschnitt 1: `gui/constants.py`

```python
PROJECT_PATH = pathlib.Path(__file__).parent
PROGRAM_GELENKPOSE = REPO_ROOT / "programs" / "beispiel_gelenkpose.json"
...
JOG_STEP_M = 0.05
JOG_STEP_OPTIONS = {"1 mm": 0.001, "1 cm": 0.01, ...}
FONT_SIZE_OPTIONS = {"Klein": 9, "Mittel": 11, "Groß": 14}
DEV_MODE_PASSWORD = "bensen"
JOG_PAD_BUTTONS = (("x", 1, "▲"), ("x", -1, "▼"), ...)
```

Reine Konstanten-Sammlung — Dateipfade und feste Werte, die von mehreren Mixin-Dateien
gebraucht werden (z.B. die Jog-Schrittweiten-Auswahl in den Optionen). In eine eigene Datei
ausgelagert, damit sich `my_gui.py` und die Mixins nicht gegenseitig importieren müssten (das
würde einen "Zirkelimport"-Fehler geben — Datei A braucht Datei B, die wiederum A braucht).

## Abschnitt 2: `gui/style_mixin.py` — `StyleMixin`

```python
class StyleMixin:
    def set_image(self, widget_id, filename, size=(512, 512)): ...
    def _palette(self) -> dict[str, str]: ...       # Farben, je nach Dark/Hell-Modus
    def _apply_style(self) -> None: ...               # setzt alle ttk-Styles
    def _refresh_toplevel_backgrounds(self) -> None: ...
    def _make_responsive(self, window, container) -> None: ...
    def _fit_window_to_screen(self, window, margin=80) -> None: ...
    def _style_widget(self, widget_id, style_name=None, text=None) -> None: ...
```

Alles rund ums **Aussehen**. `_palette()` gibt ein Dictionary mit Farbwerten zurück, je
nachdem ob Dark Mode an ist. `_apply_style()` ist der zentrale Ort, an dem das komplette
Erscheinungsbild (Schriftgrößen, Knopf-Farben je nach Bedeutung: Start=grün,
Abbruch=rot, ...) gesetzt wird — bewusst in Python statt in der `.ui`-Datei selbst, weil der
Designer die `.ui`-Datei beim Speichern komplett neu schreiben würde und reine
XML-Anpassungen sonst verloren gingen. `_fit_window_to_screen()`: verkleinert ein Fenster,
falls es größer als der Bildschirm wäre.

## Abschnitt 3: `gui/jog_mixin.py` — `JogMixin`

```python
class JogMixin:
    def _jog(self, dx=0.0, dy=0.0, dz=0.0, drot_deg=0.0) -> bool: ...
    def _hold_jog(self, axis, sign) -> None: ...       # Knopf halten = wiederholt fahren
    def _release_jog(self) -> None: ...
    def _bind_jog_hold(self, widget_id, axis, sign, symbol) -> None: ...
    def _bind_jog_pad(self, suffix) -> None: ...        # bindet alle 8 Pfeile auf einmal
    def _bind_jog_toggle(self, toggle_button_id, controls_frame_id) -> None: ...
    def _update_position_readout(self) -> None: ...    # Live-Anzeige, alle 300ms
```

Steuert den FR20-Arm per Knopfdruck in kleinen Schritten (X/Y/Z/Werkzeugdrehung) — nutzt dafür
`resolve_point()` aus `robot_program.py` (Datei 3, Abschnitt 5), aber ohne den
Rückzug/Verfahren/Absenken-Umweg, weil Jog-Schritte klein genug sind, um sicher zu bleiben.
`_hold_jog()`: sobald ein Pfeil-Knopf gedrückt gehalten wird, fährt der Arm sofort einen
Schritt und plant sich selbst alle 150ms erneut ein (`self.mainwindow.after(...)`), solange
der Knopf gehalten bleibt — `_release_jog()` bricht das beim Loslassen ab. `_bind_jog_pad()`
ist der Helfer, den wir in dieser Session eingeführt haben, um die 8 Pfeil-Knöpfe (X+/X-/Y+/
Y-/Z+/Z-/Rot+/Rot-) nicht dreimal (für die drei Jog-Fenster) einzeln aufzuschreiben, sondern
über eine Schleife.

## Abschnitt 4: `gui/program_mixin.py` — `ProgramMixin`

```python
class ProgramMixin:
    def _ensure_simulation_running(self) -> dict: ...   # startet RViz einmalig
    def _stop_running_program(self, graceful=False) -> bool: ...
    def _run_program_async(self, program_source, on_done=None, home_at_end=True) -> None: ...
    def on_gelenkpose_pressed(self) / on_tisch_pressed(self): ...
    def on_pause_pressed() / on_weiter_pressed() / on_abbruch_pressed(): ...
    def _open_calibration_window(self) -> None: ...
    def on_kalibrieren_pressed(self) -> None: ...
    def _confirm_and_calibrate(self) -> None: ...
    def _run_figure_program(self, program_path) -> None: ...
    def on_figur1_pressed() / on_tischecken_pressed(): ...
    def on_naechster_punkt_pressed() / _set_naechster_punkt_enabled(): ...
```

Das größte Mixin — verbindet die GUI mit `robot_program.run_program()` (Datei 3). Wichtigster
Baustein: `_run_program_async()` startet ein Punktprogramm in einem **eigenen
Hintergrund-Thread**, damit die Fensteroberfläche währenddessen nicht einfriert (Tkinter, das
GUI-Framework, blockiert sonst komplett, solange irgendein Code läuft). `on_done`
(Callback-Funktion) wird aufgerufen, sobald das Programm fertig ist, aber über
`self.mainwindow.after(0, on_done)` wieder auf den Haupt-Thread umgeleitet — Tkinter darf
nämlich nur vom Haupt-Thread aus bedient werden, niemals direkt aus einem Hintergrund-Thread.
`_open_calibration_window()`/`_confirm_and_calibrate()`: der zweistufige Kalibrierablauf
(automatisch zum Referenzpunkt fahren, dann per Dialog bestätigen).

## Abschnitt 5: `gui/bending_mixin.py` — `BendingMixin`

```python
class BendingMixin:
    def on_biegekopf_pressed(self) -> None: ...          # startet den Testlauf
    def on_biegekopf_pause_pressed() / _weiter_pressed() / _abbruch_pressed(): ...
    def _do_bending_jog_step(self, sign) -> bool: ...
    def _hold_bending_jog(self, sign) -> None: ...
    def _release_bending_jog(self) -> None: ...
```

Analog zu `ProgramMixin`, aber für den Biegekopf-Testlauf (Datei 4, `bending_head_motor.py`)
— läuft in einem eigenen Thread, unabhängig vom Arm-Programm, weil es ein anderer Motor ist.
`on_biegekopf_pressed()` nutzt bewusst `RvizBendingBackend` (nicht die echte
Modbus-Hardware) — ein Testen-Button in der GUI soll nicht versehentlich den echten Servo
ansteuern können, solange die Register-Adressen noch nicht final verdrahtet sind (siehe Datei
4, Abschnitt 5-6).

## Abschnitt 6: `gui/options_mixin.py` — `OptionsMixin`

```python
class OptionsMixin:
    def on_testen_pressed() / on_testen_zurueck_pressed(): ...
    def on_optionen_pressed(self) -> None: ...        # baut alle Comboboxen auf
    def _on_jogstep_selected(label) / _on_schriftgroesse_selected(label) / ...
    def on_darkmode_toggle_pressed(self) -> None: ...
    def on_entwicklermodus_toggle_pressed(self) -> None: ...  # braucht Passwort
    def on_manuell_pressed() / on_manuell_schliessen_pressed(): ...
```

Die Nebenfenster ohne eigenen Hintergrund-Thread: Auswahlmenü "Testen", das große
Optionen-Fenster (Jog-Schrittweite, Schriftgröße, Zoom, Dark Mode, Entwicklermodus) und die
freie manuelle Jog-Steuerung. `on_entwicklermodus_toggle_pressed()`: fragt beim Aktivieren ein
Passwort ab (`DEV_MODE_PASSWORD` aus `constants.py`) — der Code selbst sagt klar, dass das
KEIN echter Sicherheitsmechanismus ist, nur ein Schutz vor versehentlichem Aktivieren (der
Einzelschrittbetrieb ist nichts für den normalen Betrieb).

## Abschnitt 7: `gui/log_mixin.py` — `_StdoutTee` und `LogMixin`

```python
class _StdoutTee:
    def write(self, text: str) -> int:
        self._original.write(text)   # normale Terminal-Ausgabe bleibt erhalten
        self._callback(text)         # zusätzlich an die Log-Anzeige weiterleiten
        return len(text)

class LogMixin:
    def _on_log_line(self, text) -> None: ...     # threadsicher in eine deque schreiben
    def _append_log_widget(self, text) -> None: ...
    def on_log_pressed(self) -> None: ...           # öffnet das Log-Fenster
```

`_StdoutTee` ist ein kleiner Trick: `sys.stdout` (wohin `print()` normalerweise schreibt) wird
durch dieses Objekt ersetzt, das JEDE Ausgabe zusätzlich an die Log-Anzeige im GUI-Fenster
weiterleitet — ohne die normale Terminal-Ausgabe zu verlieren ("tee" wie beim Unix-Befehl
`tee`, der eine Ausgabe in zwei Richtungen gleichzeitig schickt). Da `print()`-Aufrufe auch
aus Hintergrund-Threads kommen können (während ein Programm läuft), schreibt `_on_log_line()`
nur threadsicher in eine Warteschlange (`deque`), das eigentliche Update des Text-Widgets läuft
wieder über `self.mainwindow.after(0, ...)` auf dem Haupt-Thread.

## Abschnitt 8: `gui/my_gui.py` — der Zusammenbau

```python
sys.path.insert(0, str(REPO_ROOT))   # damit fr20_scene etc. gefunden werden

from style_mixin import StyleMixin
from jog_mixin import JogMixin
from program_mixin import ProgramMixin
from bending_mixin import BendingMixin
from options_mixin import OptionsMixin
from log_mixin import LogMixin, _StdoutTee

class MyApp(StyleMixin, JogMixin, ProgramMixin, BendingMixin, OptionsMixin, LogMixin):
    def __init__(self, master=None):
        # alle Zustandsvariablen (Locks, Events, Threads) werden hier zentral angelegt
        self.builder = pygubu.Builder()
        self.builder.add_from_file(PROJECT_UI)
        self.mainwindow = self.builder.get_object('mainwindow', master)
        self._apply_style()
        self.builder.connect_callbacks(self)   # <- verbindet ALLE on_..._pressed-Methoden
        ...

    def on_beenden_pressed(self): self.on_close()
    def on_close(self): ...    # zentrale Aufräum-Routine
    def run(self): ...          # zeigt zuerst Kalibrierung, startet RViz im Hintergrund

if __name__ == '__main__':
    app = MyApp()
    app.run()
```

Der "Dirigent": importiert alle Mixins, baut `MyApp` daraus zusammen, und übernimmt nur noch
das, was wirklich zentral/gemeinsam ist — die Initialisierung aller Zustandsvariablen
(`__init__`), das Aufräumen beim Beenden (`on_close`, stoppt alle laufenden Programme, fährt
RViz runter) und den Programmstart (`run()`). `builder.connect_callbacks(self)` ist der
Moment, in dem pygubu tatsächlich nachschaut, welche Methoden auf `self` existieren (egal aus
welchem Mixin) und sie an die passenden Knöpfe im `.ui`-Layout bindet.

**Damit ist der komplette Code-Walkthrough durch — alle 6 Programmdateien + alle 8 GUI-Module.**

## Zusammenfassung: wie alles zusammenhängt

```
fr20_scene.py            <- Fundament: Zellengeometrie, Posen, URDF, Start/Stopp, sendet Winkel
        ↑                    (importiert als "sim" praktisch überall)
        |                    rviz_backend.py: RViz2-Anzeige + synchrone Gelenkfahrt
fr20_kinematics.py       <- Mathematik: Gelenkwinkel <-> Position im Raum
        ↑
fr20_collision.py        <- Abstand der Armglieder (STL-Meshes) zu Tisch/Baugruppe
        ↑
robot_program.py         <- fährt Punktprogramme ab, plant kollisionsfreie Bahnen
        ↑
        |
gui/ (constants, style_mixin, jog_mixin, program_mixin,    <- Bedienoberfläche, nutzt
      bending_mixin, options_mixin, log_mixin, my_gui.py)      robot_program + bending_head_motor

bending_head_motor.py    <- steuert den echten Biegekopf-Servo an (Simulate/Modbus/RViz)
        ↑ (nur informativ, kein Code-Import in die andere Richtung)
        |
biegekraft_rechner.py    <- eigenständiges Rechen-Tool, hilft bei der Motor-/Getriebeauswahl
                             (komplett unabhängig von der Simulation)
```

