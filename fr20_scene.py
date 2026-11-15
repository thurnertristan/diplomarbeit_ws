#!/usr/bin/env python3
"""Szene und Konfiguration für den deckenmontierten Fairino FR20 (Simulation in RViz2).

Hier stehen alle gemeinsamen Daten der Zelle: Raum, Deckenhalterung, Tisch,
Baugruppe, Biegekopf-Testachse, feste Gelenkposen und Gelenkgrenzen. Dazu
das Roboter-URDF: Grundlage ist das offizielle URDF aus
`fairino_description/urdf/fairino20_v6.urdf` (vendored von
https://github.com/FAIR-INNOVATION/frcobot_ros2). Zur Laufzeit kommen ein
fixer Link "world" (Gelenk "world -> base_link", per 180°-Rotation um X
kopfüber an der Decke) und ein "tool0"-Frame am Ende der Kette dazu.

Die Anzeige selbst übernimmt RViz2 (siehe rviz_backend.py). Es gibt keine
Physik und keine automatische Kollisionserkennung - Kollisionen mit Tisch
und Baugruppe prüft fr20_collision.py, bevor eine Bewegung gesendet wird.

Voraussetzungen (ROS 2 Jazzy):
    sudo apt install ros-jazzy-rviz2 ros-jazzy-robot-state-publisher

Start (nur Szene anzeigen, Roboter fährt auf den Tisch):
    source /opt/ros/jazzy/setup.bash
    python3 fr20_scene.py
"""

from __future__ import annotations

import json
import shutil
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

# ---------------------------------------------------------------------------
# Konfiguration
# ---------------------------------------------------------------------------

MODEL_NAME = "fr20"

REPO_ROOT = Path(__file__).resolve().parent
FR20_SOURCE_URDF = REPO_ROOT / "fairino_description" / "urdf" / "fairino20_v6.urdf"

# Zur Laufzeit erzeugte Dateien (URDF, robot_state_publisher-Parameter) - nicht im Repo.
WORK_DIR = REPO_ROOT / "generated"

ROOM_SIZE_XY = 4.0          # Grundfläche des Raums (Quadrat) in m
CEILING_Z = 2.0             # Deckenhöhe = Montagehöhe der Roboterbasis
MOUNT_XY = (0.0, 0.0)       # Position der Deckenhalterung über dem Boden

TABLE_CENTER = MOUNT_XY     # x, y Position, an der die Baugruppe steht (mittig unter dem Roboter)
TABLE_TOP_Z = 0.75          # Höhe, auf der die Unterseite der Baugruppe aufliegt

# STL-Baugruppe des Nutzers (Downloads/Baugruppe1.stl, vendored). Mesh liegt in mm vor
# (SolidWorks-Export) und ist nicht um seinen Ursprung zentriert - Skalierung sowie
# Bounding-Box-Werte unten wurden einmalig aus der Datei ausgelesen:
#   min (mm): (-375.21, -857.37, -21.0)   max (mm): (324.79, 847.63, 24.0)
ASSEMBLY_MESH_PATH = REPO_ROOT / "meshes" / "baugruppe1" / "Baugruppe1.stl"
ASSEMBLY_MESH_SCALE = 0.001  # mm -> m
ASSEMBLY_XY_OFFSET_M = (-0.0252107, -0.0048693)  # Bounding-Box-Mitte (x,y) relativ zum Mesh-Ursprung
ASSEMBLY_MIN_Z_M = -0.021                        # tiefster Punkt der Baugruppe relativ zum Mesh-Ursprung
ASSEMBLY_HEIGHT_M = 0.045                        # Höhe der Baugruppe (max z - min z)
ASSEMBLY_SIZE_XY_M = (0.7, 1.705)                # Grundfläche der Baugruppe (Bounding Box x/y)
ASSEMBLY_TOP_Z = TABLE_TOP_Z + ASSEMBLY_HEIGHT_M  # Oberkante der Baugruppe (Welt-z)

# Die Baugruppe steht auf einem Tisch, etwas größer als ihre Grundfläche, der von TABLE_TOP_Z
# bis zum Boden reicht. TABLE_MARGIN_M gilt rundum, an den kurzen Enden (y-Richtung) kommt
# zusätzlich TABLE_END_OVERHANG_M dazu.
TABLE_MARGIN_M = 0.05
TABLE_END_OVERHANG_M = 0.15
TABLE_SIZE_XY = (
    ASSEMBLY_SIZE_XY_M[0] + 2 * TABLE_MARGIN_M,
    ASSEMBLY_SIZE_XY_M[1] + 2 * TABLE_MARGIN_M + 2 * TABLE_END_OVERHANG_M,
)

# Referenzpunkt für die Nullpunkt-Kalibrierung: Bounding-Box-Mitte der Baugruppe, eindeutig
# wiederzufinden. Liegt fast auf der Mount-Achse des Roboters (IK-Totzone, siehe
# LOOK_AT_TABLE_POSE), daher derselbe Versatz 0.3/-0.1 m. z = Anfahrhöhe über der Baugruppe
# (siehe APPROACH_Z) - der Flansch darf nicht auf der Baugruppe aufsitzen. Rutscht die
# Baugruppe, wird die Differenz zur tatsächlich angetasteten Position als TABLE_OFFSET
# gespeichert und auf alle kartesischen Programmpunkte addiert.
APPROACH_CLEARANCE_M = 0.05  # Abstand Flansch (tool0) über der Baugruppen-Oberkante
APPROACH_Z = ASSEMBLY_TOP_Z + APPROACH_CLEARANCE_M
CALIBRATION_REFERENCE_XYZ = (
    TABLE_CENTER[0] - ASSEMBLY_XY_OFFSET_M[0] + 0.3,
    TABLE_CENTER[1] - ASSEMBLY_XY_OFFSET_M[1] - 0.1,
    APPROACH_Z,
)

# Persistierter Tisch-Offset (dx, dy, dz in m) aus der letzten Kalibrierung - eigene Datei
# statt Wert im Code, damit er über GUI-Starts hinweg erhalten bleibt.
TABLE_OFFSET_PATH = REPO_ROOT / "table_offset.json"

# Abstand Flanschfläche <-> wrist3_link-Frame entlang der Werkzeugachse (m), aus dem
# wrist3_link.STL ausgelesen (höchster Mesh-Punkt z = 0.120). tool0 sitzt genau auf der
# Flanschfläche - alle kartesischen Programmpunkte meinen also die Flanschmitte. Früher lag
# tool0 im wrist3-Frame, 12 cm HINTER dem Flansch: ein Punkt "z = 0.807" (knapp über der
# Baugruppe) drückte den Flansch dadurch real auf z = 0.687, also in den Tisch.
FLANGE_OFFSET_M = 0.12

# Gelenkreihenfolge, wie sie fairino_description für den FR20 verwendet.
JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]

# Gelenkgrenzen (rad) aus fairino_description/urdf/fairino20_v6.urdf - die IK
# (fr20_kinematics) darf nur Lösungen innerhalb dieser Grenzen liefern.
JOINT_LIMITS_RAD = {
    "j1": (-3.0543, 3.0543),
    "j2": (-4.6251, 1.4835),
    "j3": (-2.8274, 2.8274),
    "j4": (-4.6251, 1.4835),
    "j5": (-3.0543, 3.0543),
    "j6": (-3.0543, 3.0543),
}

# Pose, in der der Endeffektor (tool0) senkrecht von der Decke auf die Tischmitte herabschaut
# (per IK aus fr20_kinematics berechnet). Zielpunkt bewusst NICHT exakt TABLE_CENTER: das
# liegt genau auf der j1-Rotationsachse - eine kinematische Singularität ohne IK-Lösung, plus
# eine reale Reichweiten-Totzone von ca. 0.3 m Radius um diese Achse. Der Zielpunkt liegt
# daher 0.3/-0.1 m neben der Mitte - visuell weiterhin "an der Tischmitte", aber IK-lösbar.
LOOK_AT_TABLE_POSE = {
    "j1": 0.87427,
    "j2": -0.95155,
    "j3": -2.30106,
    "j4": -1.45978,
    "j5": -1.57081,
    "j6": -2.44506,
}

# Die Simulation startet in der URDF-Null-Stellung. Ein direkter Sprung von dort zu
# LOOK_AT_TABLE_POSE lässt Unterarm/Handgelenk mitten durch den Tisch schwingen.
# SAFE_STAGING_POSE ist eine Zwischenpose, für die sowohl "Null -> hier" als auch
# "hier -> LOOK_AT_TABLE_POSE" kollisionsfrei sind - jeder Ablauf fährt sie zuerst an.
SAFE_STAGING_POSE = {
    "j1": 0.87400,
    "j2": 0.00000,
    "j3": -2.20909,
    "j4": -1.50000,
    "j5": -1.57080,
    "j6": -2.44500,
}

# Höhe, auf die robot_program.py beim Wechsel zwischen zwei kartesischen Punkten
# zurückzieht, bevor seitlich verfahren wird - zwei sichere Punkte können unterschiedliche
# Ellbogen-Konfigurationen brauchen, reine Gelenkraum-Interpolation zwischen ihnen schwingt
# dann oft durch den Tisch. Deutlich über TABLE_TOP_Z für Abstand auch bei Ellbogen-Wechsel.
SAFE_TRAVEL_Z = 1.4

# Geschwindigkeitsgrenzen der Gelenke (rad/s) aus fairino_description/urdf/fairino20_v6.urdf -
# robot_program.py plant damit vor jedem Punkt genug Wartezeit ein (siehe _min_settle_time).
JOINT_VELOCITY_LIMIT_RAD_S = {
    "j1": 3.15,
    "j2": 3.15,
    "j3": 3.15,
    "j4": 3.2,
    "j5": 3.2,
    "j6": 3.2,
}

# Biegekopf-Testachse (siehe bending_head_motor.py/RvizBendingBackend): bewusst ein
# einfacher Platzhalter (Ständer + drehbare Welle) statt eines Nachbaus des realen
# Biegekopfs - es geht nur um Kinematik/Zeitablauf im Zusammenspiel mit dem FR20-Arm, solange
# die Modbus-Register des realen MCDLN35SG-Servos noch nicht final sind.
BENDING_HEAD_JOINT = "bending_head"
BENDING_HEAD_VELOCITY_RAD_S = 1.0  # Platzhalter, bis ein konkretes Getriebe/Motorprofil feststeht
BENDING_HEAD_SHAFT_LENGTH_M = 0.4

# Position der Achse: neben dem Tisch, am Ende der Baugruppe entlang ihrer langen Achse
# (Y-Richtung), mit Abstand zur Tischkante größer als die Wellenlänge.
BENDING_HEAD_XY = (
    TABLE_CENTER[0],
    TABLE_CENTER[1] - TABLE_SIZE_XY[1] / 2 - BENDING_HEAD_SHAFT_LENGTH_M - 0.1,
)
BENDING_HEAD_Z = TABLE_TOP_Z + 0.05


# ---------------------------------------------------------------------------
# Hilfsfunktionen
# ---------------------------------------------------------------------------

class JointCommand:
    """Sollwert-Nachricht für ein Gelenk: `.data` = Zielwinkel in rad. Wird an
    `publishers[gelenk].publish()` übergeben (siehe rviz_backend.launch)."""

    def __init__(self, data: float = 0.0) -> None:
        self.data = data


def check_tool(name: str) -> None:
    """Prüft, ob ein Kommandozeilentool (z.B. "rviz2") installiert ist."""
    if shutil.which(name) is None:
        sys.exit(
            f"Fehler: '{name}' wurde nicht gefunden. Bitte ROS 2 sourcen "
            f"(z.B. 'source /opt/ros/jazzy/setup.bash') und die benötigten "
            f"Pakete installieren (siehe Docstring am Kopf dieser Datei)."
        )


def load_table_offset() -> tuple[float, float, float]:
    """Lädt den zuletzt kalibrierten Tisch-Offset (dx, dy, dz in m). Ohne vorherige
    Kalibrierung (Datei fehlt): (0, 0, 0)."""
    if not TABLE_OFFSET_PATH.exists():
        return (0.0, 0.0, 0.0)
    data = json.loads(TABLE_OFFSET_PATH.read_text())
    return (data["dx"], data["dy"], data["dz"])


def save_table_offset(dx: float, dy: float, dz: float) -> None:
    """Speichert einen neu kalibrierten Tisch-Offset, siehe load_table_offset()."""
    TABLE_OFFSET_PATH.write_text(json.dumps({"dx": dx, "dy": dy, "dz": dz}, indent=2))


def write_mount_urdf(path: Path) -> None:
    """Hängt den FR20 kopfüber an die Decke. Das Original-URDF kennt weder einen
    'world'-Verankerungslink noch ein 'tool0'-Werkzeugframe - beides wird hier ins geparste
    XML eingefügt:
      - 'world' -> 'base_link': fixes Gelenk, per rpy="pi 0 0" um 180° um die X-Achse gekippt
        (Arm hängt nach unten statt zu stehen), xyz setzt die Deckenposition.
      - 'wrist3_link' -> 'tool0': fixes Gelenk, FLANGE_OFFSET_M entlang z - tool0 liegt auf
        der Flanschfläche, damit fr20_kinematics.py die echte Flanschpose berechnet.
    """
    tree = ET.parse(FR20_SOURCE_URDF)
    robot = tree.getroot()
    robot.set("name", MODEL_NAME)

    mount_x, mount_y = MOUNT_XY
    ET.SubElement(robot, "link", {"name": "world"})
    world_joint = ET.SubElement(robot, "joint", {"name": "world_to_base", "type": "fixed"})
    ET.SubElement(world_joint, "origin", {
        "xyz": f"{mount_x} {mount_y} {CEILING_Z}",
        "rpy": "3.14159265358979 0 0",
    })
    ET.SubElement(world_joint, "parent", {"link": "world"})
    ET.SubElement(world_joint, "child", {"link": "base_link"})

    ET.SubElement(robot, "link", {"name": "tool0"})
    tool0_joint = ET.SubElement(robot, "joint", {"name": "wrist3_link-tool0_fixed_joint", "type": "fixed"})
    ET.SubElement(tool0_joint, "origin", {"xyz": f"0 0 {FLANGE_OFFSET_M}", "rpy": "0 0 0"})
    ET.SubElement(tool0_joint, "parent", {"link": "wrist3_link"})
    ET.SubElement(tool0_joint, "child", {"link": "tool0"})

    tree.write(path, xml_declaration=True, encoding="unicode")


def send_joint_pose(pose: dict, publishers: dict) -> None:
    """Setzt für alle Gelenke in `pose` (rad) den Sollwert. Die Simulation fährt alle Gelenke
    synchron (gleichzeitiger Start und gleichzeitige Ankunft) auf das Ziel - siehe
    rviz_backend._JointStateLoop."""
    loop = getattr(next(iter(publishers.values())), "loop", None)
    if loop is not None:
        loop.set_targets({j: float(a) for j, a in pose.items()})  # alle Gelenke atomar
        return
    for joint, angle in pose.items():
        publishers[joint].publish(JointCommand(angle))


def send_to_safe_staging_pose(publishers: dict) -> None:
    """Fährt den Arm aus der Null-Stellung sicher an SAFE_STAGING_POSE heran - MUSS die erste
    Bewegung nach dem Start sein (Null -> LOOK_AT_TABLE_POSE direkt kollidiert mit dem Tisch).
    Wartet, bis die Pose erreicht ist."""
    send_joint_pose(SAFE_STAGING_POSE, publishers)
    wait_until_reached(publishers)


def send_look_at_table_pose(publishers: dict) -> None:
    """Fährt den Arm in die Blick-auf-Tisch-Pose. Setzt voraus, dass der Arm bereits bei
    SAFE_STAGING_POSE steht - ein direkter Sprung aus der Null-Stellung würde kollidieren."""
    send_joint_pose(LOOK_AT_TABLE_POSE, publishers)
    wait_until_reached(publishers)


def wait_until_reached(publishers: dict, timeout_s: float = 30.0) -> None:
    """Blockiert, bis die Simulation alle Sollwerte erreicht hat (oder `timeout_s` vorbei ist)."""
    loop = getattr(next(iter(publishers.values())), "loop", None)
    if loop is None:
        time.sleep(1.0)
        return
    loop.wait_until_reached(timeout_s)


def launch_simulation(headless: bool = False):
    """Startet robot_state_publisher + RViz2 und gibt (proc, publishers) zurück -
    `publishers` enthält pro FR20-Gelenk (j1..j6) und für BENDING_HEAD_JOINT ein Objekt mit
    publish(JointCommand). Der Arm steht danach in der Null-Stellung; Aufrufer schicken selbst
    send_to_safe_staging_pose() hinterher. Zum Beenden shutdown_simulation() verwenden."""
    import rviz_backend

    return rviz_backend.launch(headless=headless)


def shutdown_simulation(proc) -> None:
    """Beendet eine mit launch_simulation() gestartete Simulation - erst freundlich, nach 10s
    notfalls erzwungen."""
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except Exception:
            proc.kill()


# ---------------------------------------------------------------------------
# Hauptprogramm
# ---------------------------------------------------------------------------

def main() -> None:
    print("Starte RViz2 ...")
    proc, publishers = launch_simulation()
    try:
        print("Fahre zunächst in die sichere Zwischenpose (Kollisionsvermeidung) ...")
        send_to_safe_staging_pose(publishers)
        print("Fahre in die Blick-auf-Tisch-Pose ...")
        send_look_at_table_pose(publishers)
        print("\nFertig. RViz-Fenster schließen oder Strg+C hier drücken, um zu beenden.")
        proc.wait()
    except KeyboardInterrupt:
        print("\nBeende RViz2 ...")
    finally:
        shutdown_simulation(proc)


if __name__ == "__main__":
    main()
