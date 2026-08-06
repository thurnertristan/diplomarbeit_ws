#!/usr/bin/env python3
"""UR10e an der Decke montiert, blickt auf einen Tisch (Gazebo Jetty / gz-sim 10).

Baut zur Laufzeit eine kleine SDF-Welt (Raum mit Boden, Wänden, Decke und
Tisch) sowie ein URDF eines Universal-Robots UR10e, dessen Basis kopfüber
an der Decke fixiert ist (der `world`-Link aus ur_description wird per
Xacro-Origin um 180° gedreht und an die Deckenposition verschoben - das
Gelenk "world -> base_link" ist fix, der Arm selbst bleibt über seine 6
Drehgelenke frei beweglich). Der Roboter wird direkt als Teil der Welt-SDF
geladen (nicht nachträglich reingespawnt) und dann per gz-transport in
eine Pose gefahren, in der das Werkzeug (tool0) von oben auf die
Tischmitte herabschaut.

Voraussetzungen (ROS 2, hier getestet mit Jazzy + Gazebo Jetty / gz-sim10):
    sudo apt install ros-jazzy-ur-description ros-jazzy-xacro \\
                      ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
    sudo apt install python3-gz-transport15 python3-gz-msgs12   # meist schon dabei

Vor dem Start ROS 2 sourcen:
    source /opt/ros/jazzy/setup.bash
    python3 ur10e_ceiling_gazebo.py

Optionen:
    --headless     startet gz-sim ohne GUI (Server-only, z.B. für CI)
"""

from __future__ import annotations

import argparse
import math
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

# ---------------------------------------------------------------------------
# Konfiguration
# ---------------------------------------------------------------------------

UR_TYPE = "ur10e"
MODEL_NAME = "ur10e"
WORLD_NAME = "ceiling_ur10e_world"

ROOM_SIZE_XY = 4.0          # Grundfläche des Raums (Quadrat) in m
CEILING_Z = 2.0             # Deckenhöhe = Montagehöhe der Roboterbasis
MOUNT_XY = (0.0, 0.0)       # Position der Deckenhalterung über dem Boden

TABLE_CENTER = (0.6, -0.1)  # x, y Position der Tischmitte
TABLE_TOP_Z = 0.75          # Höhe der Tischoberfläche über dem Boden
TABLE_SIZE = (1.0, 0.6)     # Länge x Breite der Tischplatte

# Gelenkreihenfolge, wie sie ur_description für den UR10e verwendet.
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Pose, in der der Endeffektor (tool0) senkrecht von der Decke auf die
# Tischmitte herabschaut. Empirisch in gz-sim ermittelt und verifiziert
# (Basis hängt kopfüber an der Decke bei CEILING_Z):
#   tool0 landet bei ca. (0.68, -0.18, 1.07) -> ca. 0.32 m über der
#   Tischplatte, mit der Werkzeug-Z-Achse nahezu exakt nach unten (-z).
LOOK_AT_TABLE_POSE = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -math.pi / 2,
    "elbow_joint": math.pi / 2,
    "wrist_1_joint": -math.pi / 2,
    "wrist_2_joint": math.pi / 2,
    "wrist_3_joint": 0.0,
}

# PID-Gains für die gz-sim JointPositionController-Plugins. Die schulternahen
# Gelenke tragen mehr Last (Eigengewicht des hängenden Arms) und brauchen
# daher höhere Gains, damit die Pose stabil gehalten wird.
JOINT_GAINS = {
    "shoulder_pan_joint": (2000, 0, 100),
    "shoulder_lift_joint": (4000, 0, 200),
    "elbow_joint": (2000, 0, 100),
    "wrist_1_joint": (500, 0, 20),
    "wrist_2_joint": (500, 0, 20),
    "wrist_3_joint": (200, 0, 10),
}

WORK_DIR = Path(__file__).resolve().parent / "gazebo_ur10e_ceiling_generated"


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
    os.environ["GZ_PARTITION"] = f"ur10e_ceiling_{os.getpid()}"


def gazebo_env_with_ur_meshes() -> dict:
    """Ergänzt GZ_SIM_RESOURCE_PATH um den Share-Ordner von ur_description.

    Beim Einlesen des URDF wandelt SDFormat 'package://ur_description/...'
    Mesh-URIs in 'model://ur_description/...' um. gz-sim sucht solche
    model://-URIs nur in Verzeichnissen, die in GZ_SIM_RESOURCE_PATH stehen -
    daher muss der *übergeordnete* Ordner von ur_description (also der
    ROS-'share'-Ordner) dort eingetragen werden, sonst bleiben Meshes leer.
    """
    from ament_index_python.packages import get_package_share_directory

    ur_share = Path(get_package_share_directory("ur_description"))
    resource_root = str(ur_share.parent)

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


def write_mount_xacro(path: Path) -> None:
    """Schreibt ein Xacro, das den UR10e kopfüber an die Decke hängt.

    Der 'world'-Link ist der Standard-Verankerungspunkt von ur_description:
    Ein Link namens 'world' wird von gz-sim/SDFormat speziell behandelt und
    fixiert das angehängte Kind (base_link) unbeweglich an der angegebenen
    Pose in der Simulation. Durch rpy="pi 0 0" wird die Basis um 180° um die
    X-Achse gekippt, sodass der Arm nach unten hängt statt nach oben zu
    stehen; xyz setzt die Deckenposition.
    """
    mount_x, mount_y = MOUNT_XY
    content = f"""<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="{MODEL_NAME}">
  <xacro:include filename="$(find ur_description)/urdf/ur_macro.xacro"/>

  <!-- 'world' ist der von SDFormat/gz-sim erkannte Verankerungs-Link:
       alles, was fest daran hängt, bleibt unbeweglich im Raum. -->
  <link name="world"/>

  <xacro:ur_robot
    name="{MODEL_NAME}"
    tf_prefix=""
    parent="world"
    joint_limits_parameters_file="$(find ur_description)/config/{UR_TYPE}/joint_limits.yaml"
    kinematics_parameters_file="$(find ur_description)/config/{UR_TYPE}/default_kinematics.yaml"
    physical_parameters_file="$(find ur_description)/config/{UR_TYPE}/physical_parameters.yaml"
    visual_parameters_file="$(find ur_description)/config/{UR_TYPE}/visual_parameters.yaml"
    >
    <!-- 180°-Drehung um X = kopfüber an der Decke montiert -->
    <origin xyz="{mount_x} {mount_y} {CEILING_Z}" rpy="3.14159265358979 0 0"/>
  </xacro:ur_robot>
</robot>
"""
    path.write_text(content)


def xacro_to_urdf(xacro_path: Path, urdf_path: Path) -> None:
    result = run(["xacro", str(xacro_path)], capture_output=True, text=True)
    urdf_path.write_text(result.stdout)


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
    env = gazebo_env_with_ur_meshes()
    result = run(["gz", "sdf", "-p", str(urdf_path)], capture_output=True, text=True, env=env)
    sdf_text = result.stdout
    start = sdf_text.index("<model")
    end = sdf_text.rindex("</model>") + len("</model>")
    return sdf_text[start:end]


def build_world_sdf(robot_model_sdf: str) -> str:
    """SDF-Welt: Boden, vier Wände, Deckenplatte, ein Tisch und der Roboter."""
    half = ROOM_SIZE_XY / 2
    wall_h = CEILING_Z
    wall_t = 0.05
    tx, ty = TABLE_CENTER
    tl, tw = TABLE_SIZE
    table_z = TABLE_TOP_Z / 2

    def wall(name: str, pose: str, size: str) -> str:
        return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{size}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{size}</size></box></geometry>
          <material><ambient>0.85 0.85 0.82 1</ambient><diffuse>0.85 0.85 0.82 1</diffuse></material>
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
          <material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material>
        </visual>
      </link>
    </model>
{walls}

    <model name="table">
      <static>true</static>
      <pose>{tx} {ty} {table_z} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{tl} {tw} {TABLE_TOP_Z}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{tl} {tw} {TABLE_TOP_Z}</size></box></geometry>
          <material><ambient>0.55 0.35 0.2 1</ambient><diffuse>0.55 0.35 0.2 1</diffuse></material>
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


def send_look_at_table_pose() -> None:
    """Fährt den Arm über gz-transport in die Blick-auf-Tisch-Pose."""
    from gz.transport import Node
    from gz.msgs.double_pb2 import Double

    node = Node()
    pubs = {j: node.advertise(f"/{MODEL_NAME}/{j}/cmd_pos", Double) for j in JOINT_NAMES}

    # Ein paar Wiederholungen, damit keine der ersten Nachrichten durch die
    # noch nicht vollständig aufgebaute Discovery verloren geht.
    for _ in range(5):
        for joint, angle in LOOK_AT_TABLE_POSE.items():
            msg = Double()
            msg.data = angle
            pubs[joint].publish(msg)
        time.sleep(0.3)


# ---------------------------------------------------------------------------
# Hauptprogramm
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="gz-sim ohne GUI starten")
    args = parser.parse_args()

    for tool in ("xacro", "gz", "ros2"):
        check_tool(tool)

    isolate_gz_partition()

    WORK_DIR.mkdir(exist_ok=True)
    xacro_path = WORK_DIR / f"{MODEL_NAME}.urdf.xacro"
    urdf_path = WORK_DIR / f"{MODEL_NAME}.urdf"
    world_path = WORK_DIR / "world.sdf"

    print("Erzeuge Xacro/URDF für den kopfüber montierten UR10e ...")
    write_mount_xacro(xacro_path)
    xacro_to_urdf(xacro_path, urdf_path)
    append_joint_position_controllers(urdf_path)

    print("Konvertiere URDF zu SDF, damit der Roboter von Anfang an Teil der Welt ist ...")
    robot_model_sdf = urdf_to_sdf_model(urdf_path)

    print("Erzeuge SDF-Welt (Raum + Tisch + Roboter) ...")
    world_path.write_text(build_world_sdf(robot_model_sdf))

    gz_cmd = ["gz", "sim", "-r", str(world_path)]
    if args.headless:
        gz_cmd.insert(2, "-s")
    print("Starte Gazebo Jetty (gz-sim) ...")
    gz_proc = subprocess.Popen(gz_cmd, env=gazebo_env_with_ur_meshes())

    try:
        wait_for_world(WORLD_NAME)
        print("Warte, bis die Gelenk-Controller aktiv sind ...")
        time.sleep(2.0)

        print("Fahre in die Blick-auf-Tisch-Pose ...")
        send_look_at_table_pose()

        print(
            "\nFertig. Der UR10e hängt fix an der Decke und blickt mit "
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
