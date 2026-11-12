#!/usr/bin/env python3
"""Fairino FR20 an der Decke montiert, blickt auf einen Tisch (Gazebo Jetty / gz-sim 10).

Baut zur Laufzeit eine kleine SDF-Welt (Raum mit Boden, Wänden, Decke und
Tisch) sowie ein URDF eines Fairino FR20, dessen Basis kopfüber an der
Decke fixiert ist. Grundlage ist das offizielle URDF aus
`fairino_description/urdf/fairino20_v6.urdf` (vendored von
https://github.com/FAIR-INNOVATION/frcobot_ros2) - diesem wird zur
Laufzeit ein fixer Link "world" vorangestellt (Gelenk "world -> base_link"
ist fix, per 180°-Rotation um X kopfüber montiert, der Arm selbst bleibt
über seine 6 Drehgelenke j1..j6 frei beweglich) sowie ein "tool0"-Frame am
Ende der Kette angehängt (fix an wrist3_link). Der Roboter wird direkt als
Teil der Welt-SDF geladen (nicht nachträglich reingespawnt) und dann per
gz-transport in eine Pose gefahren, in der das Werkzeug (tool0) von oben
auf die Tischmitte herabschaut.

Voraussetzungen (ROS 2, getestet mit Jazzy + Gazebo Jetty/gz-sim10 UND
Gazebo Harmonic/gz-sim8):
    sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
    sudo apt install python3-gz-transport<N> python3-gz-msgs<N>   # <N> je nach
        installierter Gazebo-Version, z.B. transport15/msgs12 bei Jetty oder
        transport13/msgs10 bei Harmonic - siehe `dpkg -l | grep gz-transport`.
    Das Python-Modul selbst kann je nach Version unversioniert (gz.transport)
    oder versioniert (z.B. gz.transport13) heißen - siehe import_gz() unten,
    das beide Fälle automatisch abdeckt.

Vor dem Start ROS 2 sourcen:
    source /opt/ros/jazzy/setup.bash
    python3 fr20_ceiling_gazebo.py

Optionen:
    --headless     startet gz-sim ohne GUI (Server-only, z.B. für CI)
"""

from __future__ import annotations

import argparse
import importlib
import json
import os
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import bending_head_motor as bhm

# Verschiedene Gazebo-Versionen benennen ihre Python-Bindings unterschiedlich (unversioniert
# "gz.transport" bei Jetty, versioniert "gz.transport13" bei Harmonic, damit mehrere
# Major-Versionen nebeneinander installierbar sind) - import_gz() probiert beide Varianten durch.
_GZ_VERSION_SUFFIXES = ("", "10", "11", "12", "13", "14", "15", "16", "17", "18")


def import_gz(base: str, submodule: str = ""):
    """Importiert ein gz-Python-(Unter-)Modul unabhängig davon, ob die installierte Gazebo-
    Version es unversioniert (z.B. "gz.transport") oder versioniert ("gz.transport13")
    bereitstellt. `base` z.B. "transport"/"msgs", `submodule` optional z.B. "double_pb2"."""
    if SIM_BACKEND == "rviz" and (base, submodule) == ("msgs", "double_pb2"):
        # RViz-Betrieb braucht Gazebo nicht - eigener Ersatz für gz.msgs.Double.
        import rviz_backend
        return rviz_backend
    tried = []
    for suffix in _GZ_VERSION_SUFFIXES:
        name = f"gz.{base}{suffix}" + (f".{submodule}" if submodule else "")
        tried.append(name)
        try:
            return importlib.import_module(name)
        except ModuleNotFoundError:
            continue
    raise ModuleNotFoundError(
        f"Kein gz-Python-Modul für '{base}' gefunden. Versucht: {', '.join(tried)}. "
        f"Ist das passende Paket installiert (z.B. python3-gz-{base}<N>, "
        f"<N> = installierte Gazebo-Version, siehe 'dpkg -l | grep gz-{base}')?"
    )

# ---------------------------------------------------------------------------
# Konfiguration
# ---------------------------------------------------------------------------

# Simulations-Backend: "gazebo" (Physik-Simulation, Standard) oder "rviz" (nur 3D-Anzeige
# in RViz2, deutlich leichter - siehe rviz_backend.py). Umschalten per Umgebungsvariable:
#   FR20_SIM=rviz ./start_gui.sh
SIM_BACKEND = os.environ.get("FR20_SIM", "gazebo").strip().lower()

MODEL_NAME = "fr20"
WORLD_NAME = "ceiling_fr20_world"

REPO_ROOT = Path(__file__).resolve().parent
FR20_SOURCE_URDF = REPO_ROOT / "fairino_description" / "urdf" / "fairino20_v6.urdf"

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
ASSEMBLY_SIZE_XY_M = (0.7, 1.705)                # Grundfläche der Baugruppe (Bounding Box x/y)

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
# LOOK_AT_TABLE_POSE), daher derselbe Versatz 0.3/-0.1 m. Rutscht die Baugruppe, wird die
# Differenz zur tatsächlich angetasteten Position als TABLE_OFFSET gespeichert und auf alle
# kartesischen Programmpunkte addiert.
CALIBRATION_REFERENCE_XYZ = (
    TABLE_CENTER[0] - ASSEMBLY_XY_OFFSET_M[0] + 0.3,
    TABLE_CENTER[1] - ASSEMBLY_XY_OFFSET_M[1] - 0.1,
    TABLE_TOP_Z - ASSEMBLY_MIN_Z_M,
)

# Persistierter Tisch-Offset (dx, dy, dz in m) aus der letzten Kalibrierung - eigene Datei
# statt Wert im Code, damit er über GUI-Starts hinweg erhalten bleibt.
TABLE_OFFSET_PATH = REPO_ROOT / "table_offset.json"

# Gelenkreihenfolge, wie sie fairino_description für den FR20 verwendet.
JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]

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

# gz-sim spawnt den Roboter physikalisch immer in der URDF-Null-Stellung, unabhängig von der
# im Python-Code angenommenen "aktuellen" Pose. Ein direkter Sprung von dort zu
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

# PID-Gains für die gz-sim JointPositionController-Plugins. Schulternahe Gelenke tragen mehr
# Last (Eigengewicht des hängenden Arms, 20kg Traglast) und brauchen höhere Gains. d_gain ist
# bewusst hoch, da run_program() den Zielwinkel als Sprung vorgibt statt sanft anzunähern -
# mehr Dämpfung bremst das Überschwingen, auf Kosten einer etwas trägeren Annäherung.
JOINT_GAINS = {
    "j1": (6000, 0, 950),
    "j2": (12000, 0, 2200),
    "j3": (6000, 0, 950),
    "j4": (1000, 0, 120),
    "j5": (1000, 0, 120),
    "j6": (400, 0, 50),
}

# Geschwindigkeitsgrenzen der Gelenke (rad/s) aus fairino_description/urdf/fairino20_v6.urdf -
# robot_program.py plant damit vor jedem Punkt genug Wartezeit ein (siehe _min_settle_time),
# damit ein Gelenk bei großen Sprüngen abbremsen kann, statt über den Zielwinkel zu schießen.
JOINT_VELOCITY_LIMIT_RAD_S = {
    "j1": 3.15,
    "j2": 3.15,
    "j3": 3.15,
    "j4": 3.2,
    "j5": 3.2,
    "j6": 3.2,
}

# Biegekopf-Testachse (siehe bending_head_motor.py/GazeboBendingBackend): bewusst ein
# einfacher Platzhalter (Ständer + drehbare Welle) statt eines Nachbaus des realen
# Biegekopfs - es geht nur um Kinematik/Zeitablauf im Zusammenspiel mit dem FR20-Arm, solange
# die Modbus-Register des realen MCDLN35SG-Servos noch nicht final sind.
BENDING_HEAD_MODEL_NAME = "bending_head_axis"
BENDING_HEAD_JOINT_NAME = "bend"

# effort = reales Drehmoment-Limit aus bending_head_motor.py - rein informativ als
# Gelenkgrenze in der Simulation, KEIN Ersatz für das echte Limit am Verstärker. velocity ein
# Platzhalter, bis ein konkretes Getriebe/Motorprofil feststeht.
BENDING_HEAD_EFFORT_NM = bhm.TORQUE_LIMIT_NM
BENDING_HEAD_VELOCITY_RAD_S = 1.0

# Angenommene Trägheit der Welle um ihre eigene (Y-)Achse - deutlich höher als eine dünne
# Platzhalter-Welle physisch hätte: bei zu kleiner Trägheit schaukelt sich der Regler auf
# statt sauber anzunähern. 0.05 kg*m^2 entspricht ungefähr Spannfutter+Welle+Getriebe.
BENDING_HEAD_INERTIA_YY_KGM2 = 0.05
BENDING_HEAD_GAINS = (1.8, 0, 0.6)  # p, i, d - kritisch gedämpft für BENDING_HEAD_INERTIA_YY_KGM2

BENDING_HEAD_SHAFT_LENGTH_M = 0.4

# Position der Achse: neben dem Tisch, am Ende der Baugruppe entlang ihrer langen Achse
# (Y-Richtung). Y-Abstand zur Tischkante bewusst größer als BENDING_HEAD_SHAFT_LENGTH_M,
# damit die Welle nicht ins Kollisionsvolumen des Tisches ragt (sonst erzeugt die
# Physik-Engine Kontaktkräfte, die das Gelenk unkontrolliert durchdrehen lassen).
BENDING_HEAD_XY = (
    TABLE_CENTER[0],
    TABLE_CENTER[1] - TABLE_SIZE_XY[1] / 2 - BENDING_HEAD_SHAFT_LENGTH_M - 0.1,
)
BENDING_HEAD_Z = TABLE_TOP_Z + 0.05

WORK_DIR = Path(__file__).resolve().parent / "gazebo_fr20_ceiling_generated"


# ---------------------------------------------------------------------------
# Hilfsfunktionen
# ---------------------------------------------------------------------------

def isolate_gz_partition() -> None:
    """Isoliert diesen Lauf in einer eigenen gz-transport-Partition - gz-transport findet
    Prozesse sonst per Multicast über die ganze Maschine, ein Spawn-Aufruf könnte bei einem
    alten 'gz sim'-Prozess mit gleichem Weltnamen landen. Muss VOR gz-sim-Start gesetzt sein."""
    os.environ["GZ_PARTITION"] = f"fr20_ceiling_{os.getpid()}"


def gazebo_env_with_fr20_meshes() -> dict:
    """Ergänzt GZ_SIM_RESOURCE_PATH um den Ordner, der 'fairino_description' enthält - gz-sim
    sucht 'model://'-URIs (aus 'package://' beim URDF-Einlesen umgewandelt) nur dort;
    fairino_description ist kein installiertes ROS-Paket, sondern liegt im Workspace."""
    resource_root = str(REPO_ROOT)

    env = os.environ.copy()
    for var in ("GZ_SIM_RESOURCE_PATH", "IGN_GAZEBO_RESOURCE_PATH"):
        existing = env.get(var, "")
        parts = [p for p in existing.split(os.pathsep) if p]
        if resource_root not in parts:
            parts.append(resource_root)
        env[var] = os.pathsep.join(parts)
    return env


def check_tool(name: str) -> None:
    """Prüft, ob ein Kommandozeilentool (z.B. "gz") installiert ist."""
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


def run(cmd: list[str], **kwargs) -> subprocess.CompletedProcess:
    """Führt einen Befehl aus und gibt ihn vorher zur Nachvollziehbarkeit aus."""
    print(f"$ {' '.join(cmd)}")
    return subprocess.run(cmd, check=True, **kwargs)


def write_mount_urdf(path: Path) -> None:
    """Hängt den FR20 kopfüber an die Decke. Das Original-URDF kennt weder einen
    'world'-Verankerungslink noch ein 'tool0'-Werkzeugframe - beides wird hier ins geparste
    XML eingefügt:
      - 'world' -> 'base_link': fixes Gelenk, per rpy="pi 0 0" um 180° um die X-Achse gekippt
        (Arm hängt nach unten statt zu stehen), xyz setzt die Deckenposition.
      - 'wrist3_link' -> 'tool0': fixes Gelenk mit Identitäts-Transformation, damit
        fr20_kinematics.py eine 'tool0'-Endpose berechnen kann.
    """
    tree = ET.parse(FR20_SOURCE_URDF)
    robot = tree.getroot()
    robot.set("name", MODEL_NAME)

    mount_x, mount_y = MOUNT_XY
    world_link = ET.SubElement(robot, "link", {"name": "world"})
    world_joint = ET.SubElement(robot, "joint", {"name": "world_to_base", "type": "fixed"})
    ET.SubElement(world_joint, "origin", {
        "xyz": f"{mount_x} {mount_y} {CEILING_Z}",
        "rpy": "3.14159265358979 0 0",
    })
    ET.SubElement(world_joint, "parent", {"link": "world"})
    ET.SubElement(world_joint, "child", {"link": "base_link"})

    tool0_link = ET.SubElement(robot, "link", {"name": "tool0"})
    tool0_joint = ET.SubElement(robot, "joint", {"name": "wrist3_link-tool0_fixed_joint", "type": "fixed"})
    ET.SubElement(tool0_joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(tool0_joint, "parent", {"link": "wrist3_link"})
    ET.SubElement(tool0_joint, "child", {"link": "tool0"})

    tree.write(path, xml_declaration=True, encoding="unicode")


def append_joint_position_controllers(urdf_path: Path) -> None:
    """Hängt pro Gelenk ein gz-sim JointPositionController-Plugin an - so lassen sich die 6
    Armgelenke einzeln über gz-transport-Topics ansteuern, ganz ohne ros2_control."""
    plugins = ['  <gazebo>']
    for joint in JOINT_NAMES:
        p, i, d = JOINT_GAINS[joint]
        plugins.append(f"""    <plugin filename="gz-sim-joint-position-controller-system"
             name="gz::sim::systems::JointPositionController">
      <joint_name>{joint}</joint_name>
      <topic>/{MODEL_NAME}/{joint}/cmd_pos</topic>
      <p_gain>{p}</p_gain>
      <i_gain>{i}</i_gain>
      <d_gain>{d}</d_gain>
    </plugin>""")
    plugins.append('  </gazebo>\n</robot>')

    text = urdf_path.read_text()
    text = text.replace("</robot>", "\n".join(plugins))
    urdf_path.write_text(text)


def urdf_to_sdf_model(urdf_path: Path) -> str:
    """Wandelt das URDF vorab in ein SDF <model>-Element um - der Roboter wird NICHT erst
    nach dem gz-sim-Start per 'ros_gz_sim create' nachträglich gespawnt (das Modell existierte
    dabei physikalisch, wurde aber nie in der GUI-Szene angezeigt); von Anfang an Teil der
    world.sdf, rendert er zuverlässig."""
    env = gazebo_env_with_fr20_meshes()
    result = run(["gz", "sdf", "-p", str(urdf_path)], capture_output=True, text=True, env=env)
    sdf_text = result.stdout
    start = sdf_text.index("<model")
    end = sdf_text.rindex("</model>") + len("</model>")
    return sdf_text[start:end]


def build_bending_head_model_sdf() -> str:
    """SDF-Modell für die Biegekopf-Testachse (siehe BENDING_HEAD_*-Konstanten und
    bending_head_motor.GazeboBendingBackend): Ständer (fix mit der Welt verbunden) und eine
    drehbare Welle, mit JointPositionController-Plugin wie bei den FR20-Armgelenken -
    ansteuerbar über /{BENDING_HEAD_MODEL_NAME}/{BENDING_HEAD_JOINT_NAME}/cmd_pos.

    Bewusst ein Platzhalter (kein Nachbau des realen Biegekopfs); sobald Mechanik und Motor
    feststehen, Geometrie und BENDING_HEAD_EFFORT_NM/VELOCITY/GAINS entsprechend ersetzen.

    Die Welle rotiert um die Y-Achse - Link-Posen bleiben unrotiert, nur die Visual-/
    Kollisionsgeometrie der Welle wird für die Darstellung um 90° gekippt, damit
    Gelenk-Achsvektor und Trägheitsmoment im selben, unrotierten Bezugsrahmen bleiben.
    """
    stand_height = BENDING_HEAD_Z
    shaft_length = BENDING_HEAD_SHAFT_LENGTH_M
    shaft_radius = 0.02
    shaft_mass = 5.0  # siehe BENDING_HEAD_INERTIA_YY_KGM2 für die Begründung
    p, i, d = BENDING_HEAD_GAINS

    # Ein reiner Zylinder ist um seine Längsachse rotationssymmetrisch - eine Drehung wäre
    # von außen unsichtbar. Deshalb eine radial herausragende "Fahne" fix an der Welle
    # (eigenes visual/collision im shaft-Link), asymmetrisch platziert, damit sie bei keiner
    # Drehstellung mit dem Ständer kollidiert.
    marker_length = shaft_length * 0.5
    marker_width = 0.04
    marker_protrusion = 0.08
    marker_x_offset = shaft_radius + marker_protrusion / 2
    marker_y_offset = shaft_length / 2 - marker_length / 2 - 0.02

    return f"""
    <model name="{BENDING_HEAD_MODEL_NAME}">
      <pose>{BENDING_HEAD_XY[0]} {BENDING_HEAD_XY[1]} 0 0 0 0</pose>
      <link name="stand">
        <pose>0 0 {stand_height / 2} 0 0 0</pose>
        <collision name="collision"><geometry><box><size>0.1 0.1 {stand_height}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>0.1 0.1 {stand_height}</size></box></geometry>
          <material><ambient>0.3 0.3 0.32 1</ambient><diffuse>0.3 0.3 0.32 1</diffuse></material>
        </visual>
      </link>
      <link name="shaft">
        <pose>0 {shaft_length / 2} {stand_height} 0 0 0</pose>
        <inertial>
          <mass>{shaft_mass}</mass>
          <inertia>
            <ixx>0.1</ixx>
            <iyy>{BENDING_HEAD_INERTIA_YY_KGM2}</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>
        <collision name="collision">
          <pose>0 0 0 1.5708 0 0</pose>
          <geometry><cylinder><radius>{shaft_radius}</radius><length>{shaft_length}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <pose>0 0 0 1.5708 0 0</pose>
          <geometry><cylinder><radius>{shaft_radius}</radius><length>{shaft_length}</length></cylinder></geometry>
          <material><ambient>0.8 0.6 0.1 1</ambient><diffuse>0.8 0.6 0.1 1</diffuse></material>
        </visual>
        <collision name="collision_marker">
          <pose>{marker_x_offset} {marker_y_offset} 0 0 0 0</pose>
          <geometry><box><size>{marker_protrusion} {marker_length} {marker_width}</size></box></geometry>
        </collision>
        <visual name="visual_marker">
          <pose>{marker_x_offset} {marker_y_offset} 0 0 0 0</pose>
          <geometry><box><size>{marker_protrusion} {marker_length} {marker_width}</size></box></geometry>
          <material><ambient>0.85 0.1 0.1 1</ambient><diffuse>0.85 0.1 0.1 1</diffuse></material>
        </visual>
      </link>
      <joint name="stand_fixed" type="fixed">
        <parent>world</parent>
        <child>stand</child>
      </joint>
      <joint name="{BENDING_HEAD_JOINT_NAME}" type="revolute">
        <parent>stand</parent>
        <child>shaft</child>
        <axis>
          <xyz>0 1 0</xyz>
          <limit><effort>{BENDING_HEAD_EFFORT_NM}</effort><velocity>{BENDING_HEAD_VELOCITY_RAD_S}</velocity></limit>
        </axis>
      </joint>
      <plugin filename="gz-sim-joint-position-controller-system"
              name="gz::sim::systems::JointPositionController">
        <joint_name>{BENDING_HEAD_JOINT_NAME}</joint_name>
        <topic>/{BENDING_HEAD_MODEL_NAME}/{BENDING_HEAD_JOINT_NAME}/cmd_pos</topic>
        <p_gain>{p}</p_gain>
        <i_gain>{i}</i_gain>
        <d_gain>{d}</d_gain>
      </plugin>
    </model>"""


def build_world_sdf(robot_model_sdf: str) -> str:
    """SDF-Welt: Boden, vier Wände, Deckenplatte, Tisch mit Baugruppe und der Roboter."""
    half = ROOM_SIZE_XY / 2
    wall_h = CEILING_Z
    wall_t = 0.05

    # Wände und Decke sind transparent (niedrige Alpha + <transparency>, Ogre2 braucht beides),
    # damit der an der Decke hängende Roboter aus jedem Kamerawinkel sichtbar bleibt.
    # Kollisionsgeometrie bleibt unverändert vorhanden, nur nicht mehr blickdicht.
    WALL_ALPHA = 0.15
    CEILING_ALPHA = 0.15

    def wall(name: str, pose: str, size: str) -> str:
        return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{size}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{size}</size></box></geometry>
          <material>
            <ambient>0.85 0.85 0.82 {WALL_ALPHA}</ambient>
            <diffuse>0.85 0.85 0.82 {WALL_ALPHA}</diffuse>
          </material>
          <transparency>{1 - WALL_ALPHA}</transparency>
        </visual>
      </link>
    </model>"""

    # 'wall_west' (-X-Seite) fehlt bewusst: gz-sim's Standard-Kamera startet bei (-6,0,6) und
    # blickt entlang +X auf den Ursprung - offen auf dieser Seite blickt man von Anfang an hinein.
    walls = "".join([
        wall("wall_north", f"0 {half} {wall_h / 2} 0 0 0", f"{ROOM_SIZE_XY} {wall_t} {wall_h}"),
        wall("wall_south", f"0 {-half} {wall_h / 2} 0 0 0", f"{ROOM_SIZE_XY} {wall_t} {wall_h}"),
        wall("wall_east", f"{half} 0 {wall_h / 2} 0 0 0", f"{wall_t} {ROOM_SIZE_XY} {wall_h}"),
    ])

    # Baugruppe (STL-Mesh, siehe ASSEMBLY_*-Konstanten) steht auf einem Tisch, der etwas
    # größer als ihre Grundfläche ist und von TABLE_TOP_Z bis zum Boden reicht.
    assembly_x = TABLE_CENTER[0] - ASSEMBLY_XY_OFFSET_M[0]
    assembly_y = TABLE_CENTER[1] - ASSEMBLY_XY_OFFSET_M[1]
    assembly_z = TABLE_TOP_Z - ASSEMBLY_MIN_Z_M
    table_tx, table_ty = TABLE_CENTER
    table_sx, table_sy = TABLE_SIZE_XY
    table_pose_z = TABLE_TOP_Z / 2

    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{WORLD_NAME}">
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 {CEILING_Z + 1} 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal></plane></geometry></collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>{ROOM_SIZE_XY} {ROOM_SIZE_XY}</size></plane></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="ceiling">
      <static>true</static>
      <pose>0 0 {CEILING_Z + wall_t / 2} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{ROOM_SIZE_XY} {ROOM_SIZE_XY} {wall_t}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{ROOM_SIZE_XY} {ROOM_SIZE_XY} {wall_t}</size></box></geometry>
          <material>
            <ambient>0.9 0.9 0.9 {CEILING_ALPHA}</ambient>
            <diffuse>0.9 0.9 0.9 {CEILING_ALPHA}</diffuse>
          </material>
          <transparency>{1 - CEILING_ALPHA}</transparency>
        </visual>
      </link>
    </model>
{walls}

    <model name="table">
      <static>true</static>
      <pose>{table_tx} {table_ty} {table_pose_z} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{table_sx} {table_sy} {TABLE_TOP_Z}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{table_sx} {table_sy} {TABLE_TOP_Z}</size></box></geometry>
          <material><ambient>0.55 0.35 0.2 1</ambient><diffuse>0.55 0.35 0.2 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="assembly">
      <static>true</static>
      <pose>{assembly_x} {assembly_y} {assembly_z} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><mesh><uri>file://{ASSEMBLY_MESH_PATH}</uri><scale>{ASSEMBLY_MESH_SCALE} {ASSEMBLY_MESH_SCALE} {ASSEMBLY_MESH_SCALE}</scale></mesh></geometry>
        </collision>
        <visual name="visual">
          <geometry><mesh><uri>file://{ASSEMBLY_MESH_PATH}</uri><scale>{ASSEMBLY_MESH_SCALE} {ASSEMBLY_MESH_SCALE} {ASSEMBLY_MESH_SCALE}</scale></mesh></geometry>
          <material><ambient>0.7 0.7 0.72 1</ambient><diffuse>0.7 0.7 0.72 1</diffuse></material>
        </visual>
      </link>
    </model>

{build_bending_head_model_sdf()}

{robot_model_sdf}
  </world>
</sdf>
"""


def wait_for_world(world_name: str, timeout: float = 60.0) -> None:
    """Wartet, bis gz-sim die Welt geladen hat. Wirft TimeoutError statt sys.exit(), da
    SystemExit von "except Exception" in Hintergrund-Threads (z.B. gui/my_gui.py) nicht
    gefangen würde. Timeout bewusst hoch (60s) für langsamere Rechner beim ersten Laden."""
    deadline = time.time() + timeout
    service = f"/world/{world_name}/control"
    while time.time() < deadline:
        listed = subprocess.run(["gz", "service", "-l"], capture_output=True, text=True)
        if service in listed.stdout.splitlines():
            return
        time.sleep(0.5)
    raise TimeoutError(f"Zeitüberschreitung: gz-sim World '{world_name}' ist nicht gestartet.")


def send_joint_pose(pose: dict, publishers: dict | None = None) -> None:
    """Fährt den Arm über gz-transport in eine feste Gelenkpose. Ohne `publishers` wird ein
    einmaliger eigener gz-transport-Node aufgebaut (Standalone-Aufruf); mit `publishers`
    (wie launch_simulation()/robot_program.py sie aufbauen) werden diese wiederverwendet."""
    Double = import_gz("msgs", "double_pb2").Double

    if publishers is None:
        Node = import_gz("transport").Node

        node = Node()
        publishers = {j: node.advertise(f"/{MODEL_NAME}/{j}/cmd_pos", Double) for j in JOINT_NAMES}

    # Wiederholt senden, damit keine Nachricht durch die noch nicht vollständig
    # aufgebaute Discovery verloren geht.
    for _ in range(5):
        for joint, angle in pose.items():
            msg = Double()
            msg.data = angle
            publishers[joint].publish(msg)
        time.sleep(0.3)


def send_to_safe_staging_pose(publishers: dict | None = None) -> None:
    """Fährt den Arm aus der physikalischen Null-Stellung sicher an SAFE_STAGING_POSE heran -
    MUSS als erste Bewegung nach dem gz-sim-Start passieren (siehe Kommentar bei
    SAFE_STAGING_POSE: Null -> LOOK_AT_TABLE_POSE direkt kollidiert mit dem Tisch)."""
    send_joint_pose(SAFE_STAGING_POSE, publishers)
    time.sleep(1.0)  # Zeit, die Pose tatsächlich zu erreichen


def send_look_at_table_pose(publishers: dict | None = None) -> None:
    """Fährt den Arm in die Blick-auf-Tisch-Pose. Setzt voraus, dass der Arm bereits bei
    SAFE_STAGING_POSE steht - ein direkter Sprung aus der Null-Stellung würde kollidieren."""
    send_joint_pose(LOOK_AT_TABLE_POSE, publishers)


def launch_simulation(headless: bool = False) -> tuple[subprocess.Popen, dict]:
    """Baut URDF/SDF/Welt, startet gz-sim und die gz-transport-Publisher pro Gelenk - der
    komplette Ablauf, den main() hier, robot_program.py und gui/my_gui.py brauchen. Sendet
    bewusst noch KEINE Pose (Roboter steht in der physikalischen Null-Stellung) - Aufrufer
    entscheiden selbst, ob/wann sie z.B. send_to_safe_staging_pose() hinterherschicken.

    `publishers` enthält einen Eintrag pro FR20-Armgelenk (j1..j6) sowie "bending_head" für
    die Biegekopf-Testachse - genauso per Double-Nachricht ansteuerbar wie die Armgelenke.

    Gibt (gz_proc, publishers) zurück; zum Beenden shutdown_simulation() verwenden.

    Mit FR20_SIM=rviz (SIM_BACKEND) wird stattdessen RViz2 gestartet - gleiche Rückgabe,
    siehe rviz_backend.launch().
    """
    if SIM_BACKEND == "rviz":
        import rviz_backend
        return rviz_backend.launch(headless=headless)
    if SIM_BACKEND != "gazebo":
        raise ValueError(f"Unbekanntes FR20_SIM='{SIM_BACKEND}' - erlaubt: gazebo, rviz")
    for tool in ("gz", "ros2"):
        check_tool(tool)
    isolate_gz_partition()

    WORK_DIR.mkdir(exist_ok=True)
    urdf_path = WORK_DIR / f"{MODEL_NAME}.urdf"
    world_path = WORK_DIR / "world.sdf"

    write_mount_urdf(urdf_path)
    append_joint_position_controllers(urdf_path)
    robot_model_sdf = urdf_to_sdf_model(urdf_path)
    world_path.write_text(build_world_sdf(robot_model_sdf))

    gz_cmd = ["gz", "sim", "-r", str(world_path)]
    if headless:
        gz_cmd.insert(2, "-s")
    gz_proc = subprocess.Popen(gz_cmd, env=gazebo_env_with_fr20_meshes())

    wait_for_world(WORLD_NAME)
    time.sleep(2.0)  # bis die Gelenk-Controller aktiv sind

    Node = import_gz("transport").Node
    Double = import_gz("msgs", "double_pb2").Double
    node = Node()
    publishers = {j: node.advertise(f"/{MODEL_NAME}/{j}/cmd_pos", Double) for j in JOINT_NAMES}
    publishers["bending_head"] = node.advertise(
        f"/{BENDING_HEAD_MODEL_NAME}/{BENDING_HEAD_JOINT_NAME}/cmd_pos", Double
    )
    return gz_proc, publishers


def shutdown_simulation(gz_proc: subprocess.Popen) -> None:
    """Beendet einen mit launch_simulation() gestarteten gz-sim-Prozess sauber - erst
    freundlich, nach 10s notfalls erzwungen."""
    if gz_proc.poll() is None:
        gz_proc.terminate()
        try:
            gz_proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            gz_proc.kill()


# ---------------------------------------------------------------------------
# Hauptprogramm
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="gz-sim ohne GUI starten")
    args = parser.parse_args()

    print("Starte Gazebo Jetty (gz-sim) ...")
    gz_proc, publishers = launch_simulation(headless=args.headless)

    try:
        print("Fahre zunächst in die sichere Zwischenpose (Kollisionsvermeidung) ...")
        send_to_safe_staging_pose(publishers)

        print("Fahre in die Blick-auf-Tisch-Pose ...")
        send_look_at_table_pose(publishers)

        print(
            "\nFertig. Der FR20 hängt fix an der Decke und blickt mit "
            "seinem Werkzeug (tool0) auf die Tischmitte.\n"
            "Gazebo-Fenster schließen oder Strg+C hier drücken, um zu beenden."
        )
        gz_proc.wait()
    except KeyboardInterrupt:
        print("\nBeende Gazebo ...")
    finally:
        shutdown_simulation(gz_proc)


if __name__ == "__main__":
    main()
