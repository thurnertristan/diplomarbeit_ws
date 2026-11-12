#!/usr/bin/env python3
"""RViz2-Backend: leichte Alternative zu Gazebo - nur 3D-Anzeige, keine Physik.

Aktivieren mit der Umgebungsvariable FR20_SIM=rviz (siehe
fr20_ceiling_gazebo.launch_simulation), z.B.:
    FR20_SIM=rviz ./start_gui.sh
    FR20_SIM=rviz python3 robot_program.py programs/beispiel_tisch.json

Ablauf:
  1. Aus demselben URDF wie für Gazebo (write_mount_urdf) wird ein RViz-URDF erzeugt:
     Mesh-Pfade package:// -> file:// (fairino_description ist kein installiertes
     ROS-Paket), dazu Tisch, Baugruppe und Biegekopf-Testachse als URDF-Links, damit die
     Szene wie in Gazebo aussieht.
  2. robot_state_publisher rechnet aus URDF + /joint_states die TF-Frames.
  3. Ein eigener rclpy-Node veröffentlicht /joint_states mit 30 Hz. Gelenke fahren dabei
     mit ihrer Maximalgeschwindigkeit (JOINT_VELOCITY_LIMIT_RAD_S) auf den Sollwert zu,
     statt zu springen - ähnlich wie die Gelenkregler in Gazebo.
  4. rviz2 zeigt das Ganze an (Konfiguration: rviz/fr20.rviz).

Schnittstelle identisch zu Gazebo: launch() liefert (proc, publishers); `publishers[gelenk]
.publish(msg)` nimmt eine Nachricht mit `.data` (Sollwinkel in rad) - so funktionieren GUI,
Jog, robot_program.py und bending_head_motor.py ohne Änderung.

Unterschied zu Gazebo: keine Physik, keine Kollisionen, kein Überschwingen. Gut zum Prüfen
von Bahnen, Posen und Erreichbarkeit, nicht für Dynamik.
"""

from __future__ import annotations

import math
import os
import signal
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import fr20_ceiling_gazebo as sim

RVIZ_CONFIG = sim.REPO_ROOT / "rviz" / "fr20.rviz"
RVIZ_URDF = sim.WORK_DIR / f"{sim.MODEL_NAME}_rviz.urdf"
PUBLISH_RATE_HZ = 30.0
BENDING_JOINT = "bending_head"  # gleicher Schlüssel wie in den Gazebo-publishers


class Double:
    """Ersatz für gz.msgs.Double, falls die gz-Python-Pakete nicht installiert sind
    (RViz-Betrieb braucht Gazebo nicht) - siehe sim.import_gz."""

    def __init__(self) -> None:
        self.data = 0.0


# ---------------------------------------------------------------------------
# URDF für RViz
# ---------------------------------------------------------------------------

def _add_fixed_link(robot: ET.Element, name: str, parent: str, xyz, geometry: ET.Element, rgba: str) -> None:
    link = ET.SubElement(robot, "link", {"name": name})
    visual = ET.SubElement(link, "visual")
    visual.append(geometry)
    material = ET.SubElement(visual, "material", {"name": f"{name}_material"})
    ET.SubElement(material, "color", {"rgba": rgba})
    joint = ET.SubElement(robot, "joint", {"name": f"{parent}_to_{name}", "type": "fixed"})
    ET.SubElement(joint, "origin", {"xyz": " ".join(f"{v:.6f}" for v in xyz), "rpy": "0 0 0"})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": name})


def _box(sx: float, sy: float, sz: float) -> ET.Element:
    geometry = ET.Element("geometry")
    ET.SubElement(geometry, "box", {"size": f"{sx} {sy} {sz}"})
    return geometry


def write_rviz_urdf(path: Path) -> None:
    """Roboter-URDF wie für Gazebo, plus Umgebung (Tisch, Baugruppe, Biegekopf-Testachse)."""
    sim.write_mount_urdf(path)
    tree = ET.parse(path)
    robot = tree.getroot()

    # package://fairino_description/... -> file:///.../fairino_description/...
    for mesh in robot.iter("mesh"):
        filename = mesh.get("filename", "")
        if filename.startswith("package://"):
            mesh.set("filename", "file://" + str(sim.REPO_ROOT / filename[len("package://"):]))

    # Tisch (Box vom Boden bis TABLE_TOP_Z), Positionen wie in build_world_sdf().
    table_sx, table_sy = sim.TABLE_SIZE_XY
    _add_fixed_link(
        robot, "table", "world",
        (sim.TABLE_CENTER[0], sim.TABLE_CENTER[1], sim.TABLE_TOP_Z / 2),
        _box(table_sx, table_sy, sim.TABLE_TOP_Z), "0.55 0.35 0.2 1",
    )

    # Baugruppe (STL in mm).
    geometry = ET.Element("geometry")
    s = sim.ASSEMBLY_MESH_SCALE
    ET.SubElement(geometry, "mesh", {"filename": f"file://{sim.ASSEMBLY_MESH_PATH}", "scale": f"{s} {s} {s}"})
    _add_fixed_link(
        robot, "assembly", "world",
        (
            sim.TABLE_CENTER[0] - sim.ASSEMBLY_XY_OFFSET_M[0],
            sim.TABLE_CENTER[1] - sim.ASSEMBLY_XY_OFFSET_M[1],
            sim.TABLE_TOP_Z - sim.ASSEMBLY_MIN_Z_M,
        ),
        geometry, "0.7 0.7 0.72 1",
    )

    # Biegekopf-Testachse: Ständer + drehbare Welle mit roter Fahne (wie in Gazebo).
    stand_h = sim.BENDING_HEAD_Z
    shaft_len = sim.BENDING_HEAD_SHAFT_LENGTH_M
    bx, by = sim.BENDING_HEAD_XY
    _add_fixed_link(robot, "bending_stand", "world", (bx, by, stand_h / 2), _box(0.1, 0.1, stand_h), "0.3 0.3 0.32 1")

    shaft = ET.SubElement(robot, "link", {"name": "bending_shaft"})
    visual = ET.SubElement(shaft, "visual")
    ET.SubElement(visual, "origin", {"xyz": f"0 {shaft_len / 2} 0", "rpy": "1.5708 0 0"})
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "cylinder", {"radius": "0.02", "length": str(shaft_len)})
    ET.SubElement(ET.SubElement(visual, "material", {"name": "shaft_material"}), "color", {"rgba": "0.8 0.6 0.1 1"})
    marker = ET.SubElement(shaft, "visual")
    ET.SubElement(marker, "origin", {"xyz": f"0.06 {shaft_len * 0.75 - 0.02} 0", "rpy": "0 0 0"})
    marker.append(_box(0.08, shaft_len * 0.5, 0.04))
    ET.SubElement(ET.SubElement(marker, "material", {"name": "marker_material"}), "color", {"rgba": "0.85 0.1 0.1 1"})

    joint = ET.SubElement(robot, "joint", {"name": BENDING_JOINT, "type": "continuous"})
    ET.SubElement(joint, "origin", {"xyz": f"0 0 {stand_h / 2}", "rpy": "0 0 0"})
    ET.SubElement(joint, "parent", {"link": "bending_stand"})
    ET.SubElement(joint, "child", {"link": "bending_shaft"})
    ET.SubElement(joint, "axis", {"xyz": "0 1 0"})

    ET.indent(tree)
    tree.write(path, xml_declaration=True, encoding="unicode")


# ---------------------------------------------------------------------------
# Joint-State-Publisher (ersetzt die Gazebo-Gelenkregler)
# ---------------------------------------------------------------------------

class _JointCommand:
    """Verhält sich wie ein gz-transport-Publisher: publish(msg) setzt den Sollwinkel."""

    def __init__(self, state: "_JointStateLoop", joint: str) -> None:
        self._state = state
        self._joint = joint

    def publish(self, msg) -> None:
        self._state.set_target(self._joint, float(msg.data))


class _JointStateLoop:
    """rclpy-Node in eigenem Thread: fährt Ist- auf Sollwinkel (geschwindigkeitsbegrenzt)
    und veröffentlicht /joint_states."""

    def __init__(self, start_pose: dict[str, float]) -> None:
        import rclpy
        from sensor_msgs.msg import JointState

        self._rclpy = rclpy
        self._JointState = JointState
        self._lock = threading.Lock()
        self._current = dict(start_pose)
        self._target = dict(start_pose)
        self._velocity = dict(sim.JOINT_VELOCITY_LIMIT_RAD_S)
        self._velocity[BENDING_JOINT] = sim.BENDING_HEAD_VELOCITY_RAD_S
        self._stop = threading.Event()

        if not rclpy.ok():
            # Kein eigener Signal-Handler von rclpy: Strg+C/Beenden regelt das Hauptprogramm
            # (sonst wird der Kontext mitten im Publizieren ungültig).
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self._node = rclpy.create_node("fr20_joint_state_sim")
        self._pub = self._node.create_publisher(JointState, "joint_states", 10)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def set_target(self, joint: str, angle: float) -> None:
        with self._lock:
            self._target[joint] = angle

    def _run(self) -> None:
        dt = 1.0 / PUBLISH_RATE_HZ
        while not self._stop.is_set():
            with self._lock:
                for joint, target in self._target.items():
                    step = self._velocity.get(joint, 1.0) * dt
                    diff = target - self._current[joint]
                    self._current[joint] += max(-step, min(step, diff))
                names = list(self._current)
                positions = [self._current[j] for j in names]
            msg = self._JointState()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.name = names
            msg.position = positions
            try:
                self._pub.publish(msg)
            except Exception:  # Kontext beim Beenden schon zu - Thread still beenden
                return
            time.sleep(dt)

    def shutdown(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()


class RvizProcess:
    """Bündelt robot_state_publisher, rviz2 und den Joint-State-Thread - mit derselben
    Schnittstelle wie subprocess.Popen (poll/terminate/wait/kill), damit
    sim.shutdown_simulation() und `gz_proc.wait()` unverändert funktionieren."""

    def __init__(self, procs: list[subprocess.Popen], loop: _JointStateLoop, main_proc: subprocess.Popen) -> None:
        self._procs = procs
        self._loop = loop
        self._main = main_proc  # rviz2 (bzw. robot_state_publisher ohne Anzeige)

    def poll(self):
        return self._main.poll()

    def wait(self, timeout: float | None = None):
        return self._main.wait(timeout=timeout)

    def _signal_all(self, sig: int) -> None:
        # Ganze Prozessgruppe: "ros2 run" startet robot_state_publisher als Kindprozess,
        # ein Signal nur an "ros2 run" ließe ihn sonst weiterlaufen.
        for proc in self._procs:
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                pass

    def terminate(self) -> None:
        self._signal_all(signal.SIGTERM)
        self._loop.shutdown()

    def kill(self) -> None:
        self._signal_all(signal.SIGKILL)


def _ros_env() -> dict:
    """ROS-Kommunikation auf diesen Rechner beschränken - sonst würden im Schulnetz andere
    ROS-Rechner unsere /joint_states sehen (und umgekehrt)."""
    os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    return os.environ.copy()


def launch(headless: bool = False) -> tuple[RvizProcess, dict]:
    """Gegenstück zu sim.launch_simulation() für RViz2."""
    for tool in ("ros2", "rviz2"):
        sim.check_tool(tool)
    env = _ros_env()

    sim.WORK_DIR.mkdir(exist_ok=True)
    write_rviz_urdf(RVIZ_URDF)
    # URDF als Parameterdatei übergeben - direkt auf der Kommandozeile zerlegt der
    # YAML-Parser den XML-Text.
    params_path = sim.WORK_DIR / "robot_state_publisher_params.yaml"
    urdf_text = RVIZ_URDF.read_text().replace("\n", "\n      ")
    params_path.write_text(
        "robot_state_publisher:\n  ros__parameters:\n    robot_description: |\n      " + urdf_text + "\n"
    )

    start_pose = {j: 0.0 for j in sim.JOINT_NAMES}
    start_pose[BENDING_JOINT] = 0.0
    loop = _JointStateLoop(start_pose)

    procs = [subprocess.Popen(
        ["ros2", "run", "robot_state_publisher", "robot_state_publisher",
         "--ros-args", "--params-file", str(params_path)],
        env=env, start_new_session=True,
    )]
    main_proc = procs[0]
    if not headless:
        main_proc = subprocess.Popen(["rviz2", "-d", str(RVIZ_CONFIG)], env=env, start_new_session=True)
        procs.append(main_proc)
    time.sleep(2.0)  # bis robot_state_publisher läuft

    publishers = {j: _JointCommand(loop, j) for j in [*sim.JOINT_NAMES, BENDING_JOINT]}
    return RvizProcess(procs, loop, main_proc), publishers


if __name__ == "__main__":
    proc, pubs = launch()
    print("RViz2 läuft - fahre in die sichere Zwischenpose und dann auf den Tisch.")
    sim.send_to_safe_staging_pose(pubs)
    sim.send_look_at_table_pose(pubs)
    try:
        proc.wait()
    except KeyboardInterrupt:
        pass
    finally:
        sim.shutdown_simulation(proc)
