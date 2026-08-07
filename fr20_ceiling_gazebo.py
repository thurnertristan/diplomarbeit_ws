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

Voraussetzungen (ROS 2, hier getestet mit Jazzy + Gazebo Jetty / gz-sim10):
    sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
    sudo apt install python3-gz-transport15 python3-gz-msgs12   # meist schon dabei

Vor dem Start ROS 2 sourcen:
    source /opt/ros/jazzy/setup.bash
    python3 fr20_ceiling_gazebo.py

Optionen:
    --headless     startet gz-sim ohne GUI (Server-only, z.B. für CI)
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

# ---------------------------------------------------------------------------
# Konfiguration
# ---------------------------------------------------------------------------

MODEL_NAME = "fr20"
WORLD_NAME = "ceiling_fr20_world"

REPO_ROOT = Path(__file__).resolve().parent
FR20_SOURCE_URDF = REPO_ROOT / "fairino_description" / "urdf" / "fairino20_v6.urdf"

ROOM_SIZE_XY = 4.0          # Grundfläche des Raums (Quadrat) in m
CEILING_Z = 2.0             # Deckenhöhe = Montagehöhe der Roboterbasis
MOUNT_XY = (0.0, 0.0)       # Position der Deckenhalterung über dem Boden

TABLE_CENTER = MOUNT_XY     # x, y Position, an der die Baugruppe steht (mittig unter dem Roboter)
TABLE_TOP_Z = 0.75          # Höhe, auf der die Unterseite der Baugruppe aufliegt

# Anstelle des früheren Tisch-Quaders steht dort jetzt die STL-Baugruppe des
# Nutzers (Downloads/Baugruppe1.stl, hier vendored). Das Mesh liegt in mm vor
# (typischer SolidWorks-STL-Export) und ist nicht um seinen Ursprung
# zentriert - Skalierung sowie Bounding-Box-Werte unten wurden einmalig aus
# der Datei ausgelesen (siehe ASSEMBLY_BBOX_MM):
#   min (mm): (-375.21, -857.37, -21.0)   max (mm): (324.79, 847.63, 24.0)
ASSEMBLY_MESH_PATH = REPO_ROOT / "meshes" / "baugruppe1" / "Baugruppe1.stl"
ASSEMBLY_MESH_SCALE = 0.001  # mm -> m
ASSEMBLY_XY_OFFSET_M = (-0.0252107, -0.0048693)  # Bounding-Box-Mitte (x,y) relativ zum Mesh-Ursprung
ASSEMBLY_MIN_Z_M = -0.021                        # tiefster Punkt der Baugruppe relativ zum Mesh-Ursprung
ASSEMBLY_SIZE_XY_M = (0.7, 1.705)                # Grundfläche der Baugruppe (Bounding Box x/y)

# Die Baugruppe steht nicht frei in der Luft, sondern auf einem Tisch, der
# etwas größer ist als ihre Grundfläche und von TABLE_TOP_Z bis zum Boden
# reicht. TABLE_MARGIN_M gilt rundum, an den beiden kurzen Enden (entlang der
# 1705mm-Längsachse der Baugruppe, y-Richtung) kommt zusätzlich
# TABLE_END_OVERHANG_M dazu, damit der Tisch dort deutlich länger ist.
TABLE_MARGIN_M = 0.05
TABLE_END_OVERHANG_M = 0.15
TABLE_SIZE_XY = (
    ASSEMBLY_SIZE_XY_M[0] + 2 * TABLE_MARGIN_M,
    ASSEMBLY_SIZE_XY_M[1] + 2 * TABLE_MARGIN_M + 2 * TABLE_END_OVERHANG_M,
)

# Gelenkreihenfolge, wie sie fairino_description für den FR20 verwendet
# (base_link -> j1 -> shoulder_link -> j2 -> upperarm_link -> j3 ->
# forearm_link -> j4 -> wrist1_link -> j5 -> wrist2_link -> j6 -> wrist3_link).
JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]

# Pose, in der der Endeffektor (tool0) senkrecht von der Decke auf die
# Tischmitte herabschaut. Per IK aus fr20_kinematics berechnet und gegen
# die realen FR20-Gelenkgrenzen validiert.
#
# Zielpunkt bewusst NICHT exakt TABLE_CENTER (0, 0, 1.05): das liegt genau
# auf der j1-Rotationsachse (Basis hängt direkt über TABLE_CENTER), eine
# kinematische Singularität, für die die IK keine Lösung findet - zusätzlich
# gibt es um diese Achse herum eine reale Reichweiten-Totzone von ca. 0.3 m
# Radius (Armgeometrie kommt geometrisch nicht näher heran, unabhängig von
# den Gelenkgrenzen). Der Zielpunkt liegt daher 0.3/-0.1 m neben der Mitte -
# visuell weiterhin "an der Tischmitte", aber IK-lösbar.
LOOK_AT_TABLE_POSE = {
    "j1": 0.87427,
    "j2": -0.95155,
    "j3": -2.30106,
    "j4": -1.45978,
    "j5": -1.57081,
    "j6": -2.44506,
}

# gz-sim spawnt den Roboter physikalisch immer in der URDF-Null-Stellung
# (alle Gelenke 0), unabhängig davon, welche Pose als "aktuelle" Pose im
# Python-Code angenommen wird. Bei Null-Stellung liegt der Arm gestreckt bei
# r=1.72 m/z=1.92 m (sicher, weit über/neben dem Tisch) - ein direkter Sprung
# von dort zu LOOK_AT_TABLE_POSE lässt Unterarm/Handgelenk aber mitten durch
# den Tisch schwingen (per Sweep-Analyse über fr20_kinematics verifiziert:
# Radius<0.45 m UND Höhe<0.75 m gleichzeitig, unabhängig von TABLE_MARGIN_M).
# SAFE_STAGING_POSE ist eine Zwischenpose, für die sowohl "Null -> hier" als
# auch "hier -> LOOK_AT_TABLE_POSE" kollisionsfrei sind (ebenfalls per Sweep
# verifiziert) - jeder Ablauf (CLI, GUI-Buttons, robot_program.py) fährt sie
# zuerst an, bevor irgendein eigentliches Programm/Zielpunkt gesendet wird.
SAFE_STAGING_POSE = {
    "j1": 0.87400,
    "j2": 0.00000,
    "j3": -2.20909,
    "j4": -1.50000,
    "j5": -1.57080,
    "j6": -2.44500,
}

# Höhe, auf die robot_program.py beim Wechsel zwischen zwei kartesischen
# Punkten zurückzieht, bevor seitlich verfahren wird (siehe run_program in
# robot_program.py). Grund: Zwei für sich genommen sichere Punkte können
# unterschiedliche Ellbogen-Konfigurationen brauchen - die reine
# Gelenkraum-Interpolation zwischen ihnen schwingt dann oft tief durch den
# Tisch, obwohl Start und Ziel beide sicher über der Tischplatte liegen
# (siehe programs/beispiel_figur.json, mehrfach per Sweep-Analyse
# nachgewiesen). Deutlich über TABLE_TOP_Z, damit auch bei einem
# Ellbogen-Wechsel während des seitlichen Verfahrens genug Abstand bleibt.
SAFE_TRAVEL_Z = 1.4

# PID-Gains für die gz-sim JointPositionController-Plugins. Die schulternahen
# Gelenke tragen mehr Last (Eigengewicht des hängenden Arms, FR20 = 20kg
# Traglast, deutlich schwerere Glieder als beim UR10e) und brauchen daher
# höhere Gains, damit die Pose stabil gehalten wird.
JOINT_GAINS = {
    "j1": (6000, 0, 300),
    "j2": (12000, 0, 600),
    "j3": (6000, 0, 300),
    "j4": (1000, 0, 40),
    "j5": (1000, 0, 40),
    "j6": (400, 0, 20),
}

WORK_DIR = Path(__file__).resolve().parent / "gazebo_fr20_ceiling_generated"


# ---------------------------------------------------------------------------
# Hilfsfunktionen
# ---------------------------------------------------------------------------

def isolate_gz_partition() -> None:
    """Isoliert diesen Lauf in einer eigenen gz-transport-Partition.

    gz-transport findet Prozesse standardmäßig per Multicast über die ganze
    Maschine hinweg - läuft (z.B. von einem vorherigen Versuch) noch ein
    anderer 'gz sim'-Prozess mit gleichem Weltnamen, kann der Spawn-Aufruf
    bei DIESEM alten Prozess landen statt beim neu gestarteten Fenster, das
    man gerade sieht ("kein Roboter da"). GZ_PARTITION kapselt Server, GUI,
    'gz'/'ros2'-CLI-Aufrufe und die gz-transport-Python-Bindings dieses
    einen Laufs in ihr eigenes Netz. Muss VOR dem Start von gz-sim und vor
    jeder gz.transport/gz-CLI-Nutzung gesetzt sein.
    """
    os.environ["GZ_PARTITION"] = f"fr20_ceiling_{os.getpid()}"


def gazebo_env_with_fr20_meshes() -> dict:
    """Ergänzt GZ_SIM_RESOURCE_PATH um den Ordner, der 'fairino_description' enthält.

    Beim Einlesen des URDF wandelt SDFormat 'package://fairino_description/...'
    Mesh-URIs in 'model://fairino_description/...' um. gz-sim sucht solche
    model://-URIs nur in Verzeichnissen, die in GZ_SIM_RESOURCE_PATH stehen -
    fairino_description ist hier kein installiertes ROS-Paket, sondern liegt
    direkt im Workspace, daher muss dessen *übergeordneter* Ordner (also
    dieses Repo) dort eingetragen werden, sonst bleiben Meshes leer.
    """
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
    if shutil.which(name) is None:
        sys.exit(
            f"Fehler: '{name}' wurde nicht gefunden. Bitte ROS 2 sourcen "
            f"(z.B. 'source /opt/ros/jazzy/setup.bash') und die benötigten "
            f"Pakete installieren (siehe Docstring am Kopf dieser Datei)."
        )


def run(cmd: list[str], **kwargs) -> subprocess.CompletedProcess:
    print(f"$ {' '.join(cmd)}")
    return subprocess.run(cmd, check=True, **kwargs)


def write_mount_urdf(path: Path) -> None:
    """Hängt den FR20 kopfüber an die Decke.

    Das Original-URDF aus fairino_description kennt weder einen 'world'-
    Verankerungslink (wie ur_description ihn per Xacro-Makro anbietet) noch
    ein 'tool0'-Werkzeugframe. Beides wird hier direkt ins geparste XML
    eingefügt:
      - 'world' -> 'base_link': fixes Gelenk, per rpy="pi 0 0" um 180° um
        die X-Achse gekippt (Arm hängt nach unten statt nach oben zu
        stehen), xyz setzt die Deckenposition.
      - 'wrist3_link' -> 'tool0': fixes Gelenk mit Identitäts-Transformation,
        damit fr20_kinematics.py (analog zur UR-Kette) eine 'tool0'-Endpose
        berechnen kann.
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
    """Hängt pro Gelenk ein gz-sim JointPositionController-Plugin an.

    Damit lassen sich die 6 Armgelenke einzeln über gz-transport-Topics
    ansteuern, ganz ohne ros2_control/Controller-Manager - für eine feste
    Zielpose reicht das völlig aus.
    """
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
    """Wandelt das URDF vorab in ein SDF <model>-Element um.

    Wichtig: Der Roboter wird NICHT erst nach dem gz-sim-Start per
    'ros_gz_sim create'-Service nachträglich gespawnt. Das führte in Tests
    dazu, dass das Modell zwar physikalisch existierte (in Pose-Topics
    sichtbar), aber die GUI-Szene (Ogre2/Entity-Tree) es nie anzeigte - ein
    Sync-Problem beim nachträglichen Spawnen. Ist der Roboter dagegen von
    Anfang an Teil der world.sdf (wie Boden/Tisch/Wände), rendert er zuverlässig.
    """
    env = gazebo_env_with_fr20_meshes()
    result = run(["gz", "sdf", "-p", str(urdf_path)], capture_output=True, text=True, env=env)
    sdf_text = result.stdout
    start = sdf_text.index("<model")
    end = sdf_text.rindex("</model>") + len("</model>")
    return sdf_text[start:end]


def build_world_sdf(robot_model_sdf: str) -> str:
    """SDF-Welt: Boden, vier Wände, Deckenplatte, Tisch mit Baugruppe und der Roboter."""
    half = ROOM_SIZE_XY / 2
    wall_h = CEILING_Z
    wall_t = 0.05

    # Wände und Decke werden transparent dargestellt (niedrige Alpha +
    # <transparency>, Ogre2 braucht beides), damit der an der Decke hängende
    # Roboter aus jedem Kamerawinkel sichtbar bleibt, statt von einer Wand
    # oder der Decke selbst verdeckt zu werden. Die Kollisionsgeometrie
    # bleibt jeweils unverändert (weiterhin real vorhanden, nur nicht mehr
    # blickdicht).
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

    # 'wall_west' (auf der -X-Seite) bleibt bewusst weg: gz-sim's Standard-GUI-
    # Kamera startet bei Pose (-6, 0, 6, pitch 0.5) und blickt entlang +X auf
    # den Ursprung. Mit einer geschlossenen Box aus 4 Wänden + Decke würde die
    # Kamera nur die Außenseite dieser einen Wand sehen (Raum wirkt komplett
    # leer/schwarz-weiß, kein Roboter sichtbar) - offen auf dieser Seite blickt
    # man von Anfang an direkt hinein.
    walls = "".join([
        wall("wall_north", f"0 {half} {wall_h / 2} 0 0 0", f"{ROOM_SIZE_XY} {wall_t} {wall_h}"),
        wall("wall_south", f"0 {-half} {wall_h / 2} 0 0 0", f"{ROOM_SIZE_XY} {wall_t} {wall_h}"),
        wall("wall_east", f"{half} 0 {wall_h / 2} 0 0 0", f"{wall_t} {ROOM_SIZE_XY} {wall_h}"),
    ])

    # Baugruppe: STL-Mesh, siehe ASSEMBLY_*-Konstanten oben für Herkunft der
    # Bounding-Box-Werte. Steht auf einem Tisch (Quader, TABLE_SIZE_XY), der
    # etwas größer als ihre Grundfläche ist und von TABLE_TOP_Z bis zum Boden
    # reicht - die Baugruppe schwebt also nicht mehr frei in der Luft.
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

{robot_model_sdf}
  </world>
</sdf>
"""


def wait_for_world(world_name: str, timeout: float = 30.0) -> None:
    """Wartet, bis gz-sim die Welt (inkl. des darin enthaltenen Roboters) geladen hat."""
    deadline = time.time() + timeout
    service = f"/world/{world_name}/control"
    while time.time() < deadline:
        listed = subprocess.run(["gz", "service", "-l"], capture_output=True, text=True)
        if service in listed.stdout.splitlines():
            return
        time.sleep(0.5)
    sys.exit(f"Zeitüberschreitung: gz-sim World '{world_name}' ist nicht gestartet.")


def send_joint_pose(pose: dict, publishers: dict | None = None) -> None:
    """Fährt den Arm über gz-transport in eine feste Gelenkpose.

    Ohne `publishers` wird ein einmaliger eigener gz-transport-Node
    aufgebaut (Standalone-Aufruf, z.B. aus main()). Mit `publishers`
    (wie sie robot_gui.py/robot_program.py aufbauen) werden die
    übergebenen Publisher wiederverwendet.
    """
    from gz.msgs.double_pb2 import Double

    if publishers is None:
        from gz.transport import Node

        node = Node()
        publishers = {j: node.advertise(f"/{MODEL_NAME}/{j}/cmd_pos", Double) for j in JOINT_NAMES}

    # Ein paar Wiederholungen, damit keine der ersten Nachrichten durch die
    # noch nicht vollständig aufgebaute Discovery verloren geht.
    for _ in range(5):
        for joint, angle in pose.items():
            msg = Double()
            msg.data = angle
            publishers[joint].publish(msg)
        time.sleep(0.3)


def send_to_safe_staging_pose(publishers: dict | None = None) -> None:
    """Fährt den Arm aus der (physikalischen) Null-Stellung sicher an
    SAFE_STAGING_POSE heran - MUSS als erste Bewegung nach dem gz-sim-Start
    passieren, bevor irgendein Zielpunkt/Programm gesendet wird (siehe
    Kommentar bei SAFE_STAGING_POSE: Null -> LOOK_AT_TABLE_POSE direkt
    kollidiert mit dem Tisch)."""
    send_joint_pose(SAFE_STAGING_POSE, publishers)
    time.sleep(1.0)  # Zeit, die Pose tatsächlich zu erreichen, bevor es weitergeht


def send_look_at_table_pose() -> None:
    """Fährt den Arm über gz-transport in die Blick-auf-Tisch-Pose.

    Setzt voraus, dass der Arm bereits bei SAFE_STAGING_POSE steht (siehe
    send_to_safe_staging_pose) - ein direkter Sprung aus der Null-Stellung
    würde mit dem Tisch kollidieren.
    """
    send_joint_pose(LOOK_AT_TABLE_POSE)


# ---------------------------------------------------------------------------
# Hauptprogramm
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="gz-sim ohne GUI starten")
    args = parser.parse_args()

    for tool in ("gz", "ros2"):
        check_tool(tool)

    isolate_gz_partition()

    WORK_DIR.mkdir(exist_ok=True)
    urdf_path = WORK_DIR / f"{MODEL_NAME}.urdf"
    world_path = WORK_DIR / "world.sdf"

    print("Erzeuge URDF für den kopfüber montierten FR20 ...")
    write_mount_urdf(urdf_path)
    append_joint_position_controllers(urdf_path)

    print("Konvertiere URDF zu SDF, damit der Roboter von Anfang an Teil der Welt ist ...")
    robot_model_sdf = urdf_to_sdf_model(urdf_path)

    print("Erzeuge SDF-Welt (Raum + Tisch + Roboter) ...")
    world_path.write_text(build_world_sdf(robot_model_sdf))

    gz_cmd = ["gz", "sim", "-r", str(world_path)]
    if args.headless:
        gz_cmd.insert(2, "-s")
    print("Starte Gazebo Jetty (gz-sim) ...")
    gz_proc = subprocess.Popen(gz_cmd, env=gazebo_env_with_fr20_meshes())

    try:
        wait_for_world(WORLD_NAME)
        print("Warte, bis die Gelenk-Controller aktiv sind ...")
        time.sleep(2.0)

        print("Fahre zunächst in die sichere Zwischenpose (Kollisionsvermeidung) ...")
        send_to_safe_staging_pose()

        print("Fahre in die Blick-auf-Tisch-Pose ...")
        send_look_at_table_pose()

        print(
            "\nFertig. Der FR20 hängt fix an der Decke und blickt mit "
            "seinem Werkzeug (tool0) auf die Tischmitte.\n"
            "Gazebo-Fenster schließen oder Strg+C hier drücken, um zu beenden."
        )
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
