# Code-Walkthrough: Automatisiertes Rohrbiegesystem

Abschnittsweise Erklärung des gesamten Python-Codes im Workspace, mit Codeausschnitt +
einfacher Erklärung inklusive Zusammenhänge. `...` in Codeausschnitten = Code ausgelassen
(nicht echter Python-Code).

---

# Datei 1: `fr20_ceiling_gazebo.py`

## Abschnitt 1: Docstring + Imports + `import_gz()`

```python
import argparse, importlib, json, os, shutil, subprocess, sys, time
import xml.etree.ElementTree as ET
from pathlib import Path

import bending_head_motor as bhm

_GZ_VERSION_SUFFIXES = ("", "10", "11", ..., "18")

def import_gz(base: str, submodule: str = ""):
    for suffix in _GZ_VERSION_SUFFIXES:
        name = f"gz.{base}{suffix}" + (f".{submodule}" if submodule else "")
        try:
            return importlib.import_module(name)
        except ModuleNotFoundError:
            continue
    raise ModuleNotFoundError(...)
```

Diese Datei ist das Fundament des ganzen Projekts — sie baut zur Laufzeit eine 3D-Welt in
Gazebo (dem Physik-Simulator), setzt den FR20-Roboterarm kopfüber an die "Decke" und stellt
Funktionen bereit, mit denen andere Dateien (`robot_program.py`, `gui/`) den Arm bewegen.
Alle anderen Dateien importieren sie meist als `sim`.

`import_gz()`: Gazebos Python-Module heißen je nach installierter Version unterschiedlich
(`gz.transport` oder `gz.transport13`). Diese Funktion probiert einfach alle plausiblen Namen
durch und nimmt den ersten, der existiert — statt eines festen Imports, der nur auf manchen
Rechnern klappt.

## Abschnitt 2: Konfiguration — Raum, Tisch, Baugruppe

```python
MODEL_NAME = "fr20"
ROOM_SIZE_XY = 4.0
CEILING_Z = 2.0
MOUNT_XY = (0.0, 0.0)

TABLE_CENTER = MOUNT_XY
TABLE_TOP_Z = 0.75

ASSEMBLY_MESH_PATH = REPO_ROOT / "meshes" / "baugruppe1" / "Baugruppe1.stl"
ASSEMBLY_MESH_SCALE = 0.001  # mm -> m
ASSEMBLY_XY_OFFSET_M = (-0.0252107, -0.0048693)
ASSEMBLY_MIN_Z_M = -0.021
ASSEMBLY_SIZE_XY_M = (0.7, 1.705)

TABLE_MARGIN_M = 0.05
TABLE_END_OVERHANG_M = 0.15
TABLE_SIZE_XY = (...)
```

Ein "Baukasten" mit Maßangaben (in Metern, außer wo mm markiert). Der Raum ist 4×4m, 2m hoch;
der Roboter hängt mittig an der Decke. Die Baugruppe (`ASSEMBLY_*`) ist dein reales
3D-Bauteil als STL-Datei — STL aus SolidWorks ist in mm, Gazebo rechnet in m, daher
`ASSEMBLY_MESH_SCALE`. Die Bounding-Box-Werte wurden einmalig aus der Datei ausgemessen. Der
Tisch wird nicht aus einer Datei geladen, sondern einfach als Quader berechnet, etwas größer
als die Baugruppe.

## Abschnitt 3: Kalibrierpunkt, Gelenke, Roboter-Posen

```python
CALIBRATION_REFERENCE_XYZ = (...)  # markanter Punkt auf der Baugruppe, +0.3/-0.1 versetzt
TABLE_OFFSET_PATH = REPO_ROOT / "table_offset.json"
JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]
LOOK_AT_TABLE_POSE = {...}   # fertige Gelenkstellung: Blick senkrecht auf Tischmitte
SAFE_STAGING_POSE = {...}    # sichere Zwischenstellung
SAFE_TRAVEL_Z = 1.4
```

`CALIBRATION_REFERENCE_XYZ`: Referenzpunkt für die Nullpunkt-Kalibrierung, mit Versatz, weil
die exakte Tischmitte auf der Drehachse von Gelenk j1 liegt (kinematische Singularität — dort
gibt's keine eindeutige IK-Lösung). `JOINT_NAMES`: der FR20 hat 6 Drehgelenke (6
Freiheitsgrade, damit das Werkzeug jede Position+Ausrichtung erreichen kann).
`SAFE_STAGING_POSE`/`SAFE_TRAVEL_Z`: der wichtigste Sicherheitsmechanismus — ein direkter
Sprung von der Start- zur Zielstellung würde oft durch den Tisch schwingen (Gelenkraum ≠
gerader Weg im echten Raum), deshalb wird immer erst eine sichere Zwischenstellung angefahren.

## Abschnitt 4: PID-Gains und Geschwindigkeitsgrenzen der Gelenke

```python
JOINT_GAINS = {"j1": (6000, 0, 950), "j2": (12000, 0, 2200), ...}
JOINT_VELOCITY_LIMIT_RAD_S = {"j1": 3.15, ...}
```

Jedes Gelenk wird von einem PID-Regler gesteuert: P reagiert auf die aktuelle Abweichung, I
(hier immer 0) auf aufsummierten Dauerfehler, D bremst basierend auf der
Änderungsgeschwindigkeit (verhindert Überschwingen). j2 (Schultergelenk) hat die höchsten
Werte, weil es das meiste Gewicht trägt. Die Geschwindigkeitsgrenzen stammen direkt aus der
Hersteller-URDF und werden später in `robot_program.py` genutzt, um genug Wartezeit
einzuplanen.

## Abschnitt 5: Biegekopf-Testachsen-Konstanten + `WORK_DIR`

```python
BENDING_HEAD_MODEL_NAME = "bending_head_axis"
BENDING_HEAD_JOINT_NAME = "bend"
BENDING_HEAD_EFFORT_NM = bhm.TORQUE_LIMIT_NM   # Querverweis zu bending_head_motor.py
BENDING_HEAD_VELOCITY_RAD_S = 1.0
BENDING_HEAD_INERTIA_YY_KGM2 = 0.05
BENDING_HEAD_GAINS = (1.8, 0, 0.6)
BENDING_HEAD_SHAFT_LENGTH_M = 0.4
BENDING_HEAD_XY = (...)  # neben dem Tisch positioniert
BENDING_HEAD_Z = TABLE_TOP_Z + 0.05

WORK_DIR = Path(__file__).resolve().parent / "gazebo_fr20_ceiling_generated"
```

Konstanten für die rein virtuelle Test-Achse in Gazebo (Platzhalter für den echten
Biegekopf-Motor, um Kinematik/Timing zu testen, bevor die Modbus-Hardware fertig ist).
`BENDING_HEAD_EFFORT_NM` übernimmt das reale Drehmoment-Limit aus `bending_head_motor.py`,
damit der Wert nur an einer Stelle gepflegt wird. `WORK_DIR` ist der Ordner, in den die
generierten URDF-/SDF-Dateien geschrieben werden — nichts davon ist von Hand geschrieben,
alles wird bei jedem Start neu zusammengebaut.

## Abschnitt 6: Hilfsfunktionen (Teil 1) — Prozess-/Umgebungs-Helfer

```python
def isolate_gz_partition() -> None:
    os.environ["GZ_PARTITION"] = f"fr20_ceiling_{os.getpid()}"

def gazebo_env_with_fr20_meshes() -> dict:
    ...  # trägt den Repo-Ordner in GZ_SIM_RESOURCE_PATH ein

def check_tool(name: str) -> None:
    if shutil.which(name) is None: sys.exit("Fehler: ...")

def load_table_offset() / save_table_offset(dx, dy, dz):
    ...  # liest/schreibt table_offset.json

def run(cmd, **kwargs):
    print(f"$ {' '.join(cmd)}")
    return subprocess.run(cmd, check=True, **kwargs)
```

`isolate_gz_partition()`: verhindert, dass sich mehrere Gazebo-Läufe gegenseitig stören
(nutzt die Prozess-ID als eindeutigen Namen). `gazebo_env_with_fr20_meshes()`: sorgt dafür,
dass Gazebo die 3D-Meshes des Arms findet (kein installiertes ROS-Paket, liegt im Repo).
`check_tool()`: prüft, ob z.B. `gz` installiert ist, mit klarer Fehlermeldung.
`load_table_offset()`/`save_table_offset()`: das Kalibrierungs-Speicherpaar. `run()`: kleiner
Wrapper, der den Befehl vor der Ausführung ausgibt.

## Abschnitt 7: `write_mount_urdf()` und `append_joint_position_controllers()`

```python
def write_mount_urdf(path: Path) -> None:
    tree = ET.parse(FR20_SOURCE_URDF)
    robot = tree.getroot()
    # fügt "world"-Link + fixes Gelenk world->base_link ein (180°-Kippung = kopfüber)
    # fügt "tool0"-Link + fixes Gelenk wrist3_link->tool0 ein
    tree.write(path, ...)

def append_joint_position_controllers(urdf_path: Path) -> None:
    # hängt pro Gelenk ein JointPositionController-Plugin an (macht es über
    # gz-transport ansteuerbar), Gains aus JOINT_GAINS
```

URDF = XML-Textdatei, die den Roboter als Kette von Links (starre Teile) und Joints (Gelenke)
beschreibt. Das offizielle Hersteller-URDF geht davon aus, dass der Arm auf dem Boden steht —
`write_mount_urdf()` fügt per Code einen "world"-Verankerungspunkt (kopfüber gekippt) und
einen "tool0"-Namen für die Werkzeugspitze ein, ohne die Originaldatei zu verändern.
`append_joint_position_controllers()` macht die 6 Gelenke tatsächlich fernsteuerbar, indem
für jedes ein Steuerungs-Plugin mit Topic-Namen (`/fr20/j1/cmd_pos` usw.) angehängt wird.

## Abschnitt 8: `urdf_to_sdf_model()`

```python
def urdf_to_sdf_model(urdf_path: Path) -> str:
    result = run(["gz", "sdf", "-p", str(urdf_path)], capture_output=True, text=True, env=...)
    sdf_text = result.stdout
    start = sdf_text.index("<model")
    end = sdf_text.rindex("</model>") + len("</model>")
    return sdf_text[start:end]
```

URDF beschreibt nur einen Roboter, SDF (Gazebos eigenes Format) eine ganze Welt (Boden,
Wände, mehrere Modelle). Diese Funktion ruft das Kommandozeilentool `gz sdf -p` auf, das
URDF→SDF umwandelt, und schneidet aus der Ausgabe nur das `<model>...</model>`-Stück heraus.
Der Roboter soll von Anfang an Teil der Welt-Datei sein (nicht nachträglich reingespawnt),
damit er zuverlässig im Simulationsfenster angezeigt wird.

## Abschnitt 9: `build_bending_head_model_sdf()`

```python
def build_bending_head_model_sdf() -> str:
    return f"""
    <model name="{BENDING_HEAD_MODEL_NAME}">
      <link name="stand"> ... </link>          <!-- fixer Sockel -->
      <link name="shaft">                       <!-- drehbare Welle -->
        <inertial>...</inertial>
        <collision>...<cylinder>...</cylinder></collision>
        <visual>...</visual>
        <collision name="collision_marker">...<box>...</box></collision>  <!-- "Fahne" -->
        <visual name="visual_marker">...</visual>
      </link>
      <joint name="stand_fixed" type="fixed">...</joint>
      <joint name="{BENDING_HEAD_JOINT_NAME}" type="revolute">
        <axis><xyz>0 1 0</xyz><limit>...</limit></axis>
      </joint>
      <plugin ...JointPositionController.../>
    </model>"""
```

Die Biegekopf-Testachse: ein fixer Ständer + eine um die Y-Achse drehbare Welle. `<collision>`
ist die Geometrie, mit der die Physik rechnet (oft vereinfacht), `<visual>` was man sieht.
Ein reiner Zylinder wäre um seine eigene Achse rotationssymmetrisch — man würde eine Drehung
gar nicht sehen — deshalb sitzt eine kleine, außermittig platzierte "Fahne" (Marker) fix an
der Welle, rein kosmetisch. Das `<plugin>` am Ende macht die Achse über
`/bending_head_axis/bend/cmd_pos` ansteuerbar, genau wie die Arm-Gelenke.

## Abschnitt 10: `build_world_sdf()`

```python
def build_world_sdf(robot_model_sdf: str) -> str:
    def wall(name, pose, size) -> str: ...   # eine Wand bauen (DRY-Prinzip)
    walls = "".join([wall("wall_north", ...), wall("wall_south", ...), wall("wall_east", ...)])
    # wall_west fehlt bewusst - Standardkamera blickt von dort hinein

    return f"""<sdf ...><world ...>
      <physics .../> <plugin .../>  <!-- Physics, UserCommands, SceneBroadcaster, Sensors -->
      <light .../>
      <model name="ground_plane">...</model>
      <model name="ceiling">...</model>
{walls}
      <model name="table">...</model>
      <model name="assembly">...</model>
{build_bending_head_model_sdf()}
{robot_model_sdf}
    </world></sdf>"""
```

Die Dachfunktion, die alles zu einer kompletten Welt zusammenfügt. `wall()` ist eine lokale
Hilfsfunktion, um die vier (hier: drei) Wände nicht viermal zu wiederholen. Wände/Decke sind
transparent (`WALL_ALPHA`), damit der Roboter aus jedem Winkel sichtbar bleibt, die
Kollisionsgeometrie bleibt trotzdem real. Am Ende werden `{build_bending_head_model_sdf()}`
und `{robot_model_sdf}` (kommt als Parameter rein, aus Abschnitt 8) einfach an der richtigen
Stelle eingesetzt — jede Funktion kümmert sich um genau einen Teil der Welt.

## Abschnitt 11: `wait_for_world()` und `send_joint_pose()`

```python
def wait_for_world(world_name, timeout=60.0) -> None:
    while noch Zeit übrig:
        if "/world/<name>/control" in gz-Diensteliste: return
        time.sleep(0.5)
    raise TimeoutError(...)

def send_joint_pose(pose: dict, publishers=None) -> None:
    if publishers is None:
        # eigenen Node + Publisher pro Gelenk aufbauen (Standalone-Aufruf)
    for _ in range(5):  # mehrfach senden, gegen verlorene erste Discovery-Nachrichten
        for joint, angle in pose.items():
            publishers[joint].publish(Double-Nachricht mit angle)
        time.sleep(0.3)
```

`wait_for_world()`: Polling-Schleife, die wartet, bis Gazebo die Welt vollständig geladen hat
(prüft alle 0.5s, ob ein interner Dienst schon existiert). Wirft `TimeoutError` statt
`sys.exit()`, weil letzteres in Hintergrund-Threads (GUI) nicht sauber abgefangen würde.
`send_joint_pose()`: die zentrale "Bewege den Arm"-Funktion — schickt Zielwinkel pro Gelenk
über gz-transport, mehrfach hintereinander gesendet, weil die allerersten Nachrichten nach
Verbindungsaufbau verloren gehen können.

## Abschnitt 12: `send_to_safe_staging_pose()` und `send_look_at_table_pose()`

```python
def send_to_safe_staging_pose(publishers=None) -> None:
    send_joint_pose(SAFE_STAGING_POSE, publishers)
    time.sleep(1.0)  # Zeit zum tatsächlichen Erreichen

def send_look_at_table_pose(publishers=None) -> None:
    send_joint_pose(LOOK_AT_TABLE_POSE, publishers)
```

Benannte Wrapper um `send_joint_pose()` für die beiden festen Posen aus Abschnitt 3. Werden
praktisch immer als Paar nacheinander aufgerufen: erst sicher zur Zwischenstellung, dann zur
Zielstellung — derselbe zweistufige Ablauf, der in `robot_program.py` für beliebige
Zielpunkte nochmal (allgemeiner) auftaucht.

## Abschnitt 13: `launch_simulation()`

```python
def launch_simulation(headless=False) -> tuple[subprocess.Popen, dict]:
    check_tool("gz"); check_tool("ros2")
    isolate_gz_partition()

    write_mount_urdf(urdf_path)
    append_joint_position_controllers(urdf_path)
    robot_model_sdf = urdf_to_sdf_model(urdf_path)
    world_path.write_text(build_world_sdf(robot_model_sdf))

    gz_proc = subprocess.Popen(["gz", "sim", "-r", str(world_path), ...], env=...)
    wait_for_world(WORLD_NAME)
    time.sleep(2.0)

    publishers = {j: node.advertise(...) for j in JOINT_NAMES}
    publishers["bending_head"] = node.advertise(...)
    return gz_proc, publishers
```

Die wichtigste Funktion der Datei — ruft alles bisher Besprochene zusammen. Praktisch jede
andere Datei im Projekt startet eine Simulation über genau diesen einen Funktionsaufruf.
Ablauf: Vorbedingungen prüfen → Dateien vorbereiten (URDF/SDF, Abschnitte 7+8+10) → Gazebo als
Subprozess starten (`Popen`, nicht-blockierend, läuft im Hintergrund weiter) → warten bis
bereit → Publisher pro Gelenk (+ Biegekopf-Testachse) aufbauen. Rückgabe `(gz_proc,
publishers)` ist der zentrale "Handshake": damit kann der Aufrufer den Arm steuern und die
Simulation später sauber beenden.

## Abschnitt 14: `shutdown_simulation()` und `main()`

```python
def shutdown_simulation(gz_proc) -> None:
    if gz_proc.poll() is None:
        gz_proc.terminate()          # freundlich
        try: gz_proc.wait(timeout=10)
        except subprocess.TimeoutExpired: gz_proc.kill()   # erzwungen

def main() -> None:
    args = parser.parse_args()  # nur --headless
    gz_proc, publishers = launch_simulation(headless=args.headless)
    try:
        send_to_safe_staging_pose(publishers)
        send_look_at_table_pose(publishers)
        gz_proc.wait()
    except KeyboardInterrupt:
        print("Beende Gazebo ...")
    finally:
        shutdown_simulation(gz_proc)

if __name__ == "__main__":
    main()
```

`shutdown_simulation()`: zweistufiges Beenden — erst freundlich (`terminate`), nach 10s
notfalls erzwungen (`kill`), damit nie ein Geister-Prozess hängen bleibt. `main()`: der
CLI-Einstiegspunkt beim direkten Start. `try/except/finally`: `finally` läuft garantiert immer
(auch bei Strg+C), dort steht das Herunterfahren, damit Gazebo in jedem Fall sauber beendet
wird. `if __name__ == "__main__":` sorgt dafür, dass das nur beim direkten Start passiert,
nicht wenn eine andere Datei nur importiert.

---

# Datei 2: `fr20_kinematics.py`

## Abschnitt 1: Docstring, Imports, `_rpy_to_matrix()`

```python
import xml.etree.ElementTree as ET
import numpy as np
import fr20_ceiling_gazebo as sim

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

## Abschnitt 3: Hilfswerte für die Kollisionsprüfung

```python
_DEFAULT_TOOL_R = ...  # Standard-Ausrichtung: Werkzeug zeigt nach unten
def _rpy_deg_to_matrix(rpy_deg): ...   # Grad -> Radiant -> Rotationsmatrix

_FALLBACK_J1 = (-2.5, -1.25, 0.0, 1.25, 2.5)
_FALLBACK_ELBOWS = (...)  # alternative Ellbogen-Stellungen
_TABLE_CHECK_MARGIN_M = 0.05

def _pose_hits_table(position) -> bool: ...  # liegt der Punkt über/im Tisch?
```

`_pose_hits_table()`: prüft, ob ein Punkt innerhalb der Tischfläche UND unterhalb der
Tischplatte liegt (= Kollision). `_FALLBACK_J1`/`_FALLBACK_ELBOWS`: vorbereitete
Alternativ-Startstellungen für die IK, falls sie vom aktuellen Ausgangspunkt keine Lösung
findet.

## Abschnitt 4: `_is_transition_safe()`

```python
def _is_transition_safe(a, b, steps=15) -> bool:
    for frac in np.linspace(0, 1, steps):
        inter = Zwischenstellung zwischen a und b
        t = Vorwärtskinematik für diese Zwischenstellung
        if _pose_hits_table(t[:3, 3]): return False
    return True
```

Prüft nicht nur Start/Ende, sondern den ganzen Weg dazwischen: teilt die Bewegung in 15
Zwischenschritte und prüft jeden auf Tisch-Kollision. Nötig, weil zwei sichere Stellungen
über einen unsicheren Zwischenweg verbunden sein können.

## Abschnitt 5: `resolve_point()`

```python
def resolve_point(point, current_joints_rad, allow_fallback=True) -> dict:
    if point.joints_deg is not None:
        return Grad -> Radiant  # direkt, fertig

    primary = kin.inverse_kinematics(target, current_joints_rad)
    if primary gefunden UND sicher: return primary

    for j1 in _FALLBACK_J1:
        for elbow in _FALLBACK_ELBOWS:
            sol = kin.inverse_kinematics(target, alternativer_start)
            candidates.append(sol)

    return bester (sicherster, geringste Bewegung) Kandidat
```

Zentrale Funktion: übersetzt "wo soll der Arm hin" in "welche Gelenkwinkel". Erst der
schnelle Versuch (IK von aktueller Stellung). Klappt das nicht/unsicher, wird mit mehreren
Alternativ-Starts nochmal gesucht, am Ende die sicherste, am wenigsten sprunghafte Lösung
gewählt.

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

## Abschnitt 7: `_interruptible_sleep()` und `_build_travel_waypoints()`

```python
def _interruptible_sleep(seconds, stop_event, abort_event=None) -> None:
    ...  # wie time.sleep(), kehrt aber sofort zurück bei Abbruch

def _build_travel_waypoints(point, current_joints_rad, offset_xyz=(0,0,0)) -> list:
    if point.xyz is None:
        return [resolve_point(point, current_joints_rad)]  # Gelenkpose: direkt

    retract = resolve_point(..., hoch über Startposition)
    travel = resolve_point(..., hoch über Zielposition)
    final = resolve_point(point, ...)
    return [retract, travel, final]
```

`_interruptible_sleep`: abbrechbares Warten. `_build_travel_waypoints`: praktischer
Kollisionsschutz — erst hoch zurückziehen, seitlich verfahren, dann erst absenken, statt
direkt quer zu fahren.

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
        waypoints = _build_travel_waypoints(point, current, offset_xyz=offset_xyz)
        for target in waypoints:
            _wait_while(pause_event, ...)   # ggf. Pause abwarten
            send(target)                     # Winkel an Gazebo schicken
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

## Abschnitt 11: `dry_run_program()`

```python
def dry_run_program(program: Program) -> bool:
    sim.write_mount_urdf(urdf_path)   # nur URDF, keine Simulation nötig
    current = dict(sim.SAFE_STAGING_POSE)
    all_ok = True
    for point in program.points:
        try:
            waypoints = _build_travel_waypoints(point, current, offset_xyz=offset_xyz)
        except (ValueError, RuntimeError) as exc:
            print(f"FEHLER bei '{point.name}': {exc}")
            all_ok = False
            continue
        for solution in waypoints:
            print(f"'{point.name}': {Gelenkwinkel in Grad}")
            current = solution
    return all_ok
```

Löst alle Punkte eines Programms auf und zeigt die berechneten Gelenkwinkel an — **ohne**
Gazebo/ROS 2 zu starten und ohne irgendwas an einen echten oder simulierten Roboter zu
schicken. Praktisch, um eine neue/geänderte Programmdatei schnell auf Fehler zu prüfen
(unerreichbare Punkte, große Sprünge), bevor man überhaupt Gazebo startet — genau das haben
wir in dieser Session mit `--dry-run` benutzt, um die Kürzungen zu verifizieren.

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

    gz_proc, publishers = sim.launch_simulation(headless=args.headless)
    try:
        sim.send_to_safe_staging_pose(publishers)
        run_program(program, publishers, loop=args.loop)
        gz_proc.wait()
    except KeyboardInterrupt:
        ...
    finally:
        sim.shutdown_simulation(gz_proc)
```

Der Einstiegspunkt beim direkten Start (`python3 robot_program.py programs/....json`).
Derselbe Aufbau wie schon in `fr20_ceiling_gazebo.main()` (Datei 1, Abschnitt 14): Simulation
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

## Abschnitt 7: `GazeboBendingBackend`

```python
class GazeboBendingBackend:
    def __init__(self, publishers, start_position_deg=0.0):
        self._current_rad = math.radians(start_position_deg)
        sim.send_joint_pose({"bending_head": self._current_rad}, publishers)

    def connect(self, torque_limit_nm): print(f"[Gazebo] ... (gilt hier nur real)")

    def move_by(self, delta):
        self._current_rad += math.radians(delta)
        # neue absolute Position an Gazebo senden
```

Das Backend, das wir in dieser Session gemeinsam gebaut und getestet haben — steuert die
Test-Achse aus Datei 1 (Abschnitt 9) in Gazebo an, statt echte Hardware. Wichtiger Unterschied
zu den anderen Backends: der Gazebo-Regler erwartet immer eine **absolute** Zielposition, kein
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
    mode.add_argument("--gazebo", action="store_true")
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    if args.gazebo:
        gz_proc, publishers = sim.launch_simulation(headless=args.headless)
        axis = BendingHeadAxis(GazeboBendingBackend(publishers))
        axis.move_by(args.delta)
        gz_proc.wait()
        return

    backend = SimulatedBendingBackend() if args.simulate else ModbusRTUBendingBackend(port=args.port)
    axis = BendingHeadAxis(backend)
    axis.move_by(args.delta)
```

Drei Betriebsarten über Kommandozeilen-Flags: `--simulate` (nur loggen), `--gazebo` (Test in
der Simulation, genau das haben wir in dieser Session mehrfach so aufgerufen), oder ohne Flag
= echte Hardware über `--port`. `mode.add_mutually_exclusive_group()` (nicht extra gezeigt,
aber im echten Code) sorgt dafür, dass man nicht versehentlich `--simulate` und `--gazebo`
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
    def _ensure_simulation_running(self) -> dict: ...   # startet Gazebo einmalig
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
`on_biegekopf_pressed()` nutzt bewusst `GazeboBendingBackend` (nicht die echte
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
sys.path.insert(0, str(REPO_ROOT))   # damit fr20_ceiling_gazebo etc. gefunden werden

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
    def run(self): ...          # zeigt zuerst Kalibrierung, startet Gazebo im Hintergrund

if __name__ == '__main__':
    app = MyApp()
    app.run()
```

Der "Dirigent": importiert alle Mixins, baut `MyApp` daraus zusammen, und übernimmt nur noch
das, was wirklich zentral/gemeinsam ist — die Initialisierung aller Zustandsvariablen
(`__init__`), das Aufräumen beim Beenden (`on_close`, stoppt alle laufenden Programme, fährt
Gazebo runter) und den Programmstart (`run()`). `builder.connect_callbacks(self)` ist der
Moment, in dem pygubu tatsächlich nachschaut, welche Methoden auf `self` existieren (egal aus
welchem Mixin) und sie an die passenden Knöpfe im `.ui`-Layout bindet.

**Damit ist der komplette Code-Walkthrough durch — alle 6 Programmdateien + alle 8 GUI-Module.**

## Zusammenfassung: wie alles zusammenhängt

```
fr20_ceiling_gazebo.py   <- Fundament: baut die Gazebo-Welt, startet/stoppt sie, sendet Winkel
        ↑                    (importiert als "sim" praktisch überall)
        |
fr20_kinematics.py       <- Mathematik: Gelenkwinkel <-> Position im Raum
        ↑
        |
robot_program.py         <- fährt Punktprogramme ab, nutzt sim + kin, Kollisionsvermeidung
        ↑
        |
gui/ (constants, style_mixin, jog_mixin, program_mixin,    <- Bedienoberfläche, nutzt
      bending_mixin, options_mixin, log_mixin, my_gui.py)      robot_program + bending_head_motor

bending_head_motor.py    <- steuert den echten Biegekopf-Servo an (Simulate/Modbus/Gazebo)
        ↑ (nur informativ, kein Code-Import in die andere Richtung)
        |
biegekraft_rechner.py    <- eigenständiges Rechen-Tool, hilft bei der Motor-/Getriebeauswahl
                             (komplett unabhängig von der Simulation)
```

