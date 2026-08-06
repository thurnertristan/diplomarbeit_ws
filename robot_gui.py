#!/usr/bin/env python3
"""Einfache GUI zur Steuerung des Fairino FR20 in der Deckenmontage-Simulation.

Startet dieselbe Gazebo-Welt wie fr20_ceiling_gazebo.py (Raum, Tisch,
kopfüber montierter FR20) und öffnet zusätzlich ein Tkinter-Fenster mit
einem Schieberegler pro Gelenk. Jeder Regler sendet live per gz-transport
auf das vom Simulationsskript angelegte JointPositionController-Topic
(/fr20/<gelenk>/cmd_pos).

Voraussetzungen: siehe Docstring von fr20_ceiling_gazebo.py (ROS 2
gesourced, Gazebo-Pakete installiert; fairino_description liegt vendored
im Workspace).

Start:
    source /opt/ros/jazzy/setup.bash
    python3 robot_gui.py
"""

from __future__ import annotations

import math
import subprocess
import time
import tkinter as tk
from tkinter import ttk

import numpy as np

import fr20_ceiling_gazebo as sim
import fr20_kinematics as kin

# Schieberegler-Bereich je Gelenk. Bewusst auf ±180° begrenzt (statt der
# realen FR20-Gelenkgrenzen von teils über ±180° hinaus), damit sich der an der Decke
# hängende Arm über die GUI nicht in sich selbst oder in die Wände verdrehen
# kann - für eine "einfache" Steuerung ist das ausreichend Bewegungsspielraum.
JOINT_LIMIT_DEG = 180


class RobotGui(tk.Tk):
    def __init__(self, publishers: dict, gz_proc: subprocess.Popen) -> None:
        super().__init__()
        self.title("FR20 Steuerung")
        self.publishers = publishers
        self.gz_proc = gz_proc
        self.sliders: dict[str, tk.Scale] = {}

        self.protocol("WM_DELETE_WINDOW", self.on_close)

        container = ttk.Frame(self, padding=12)
        container.pack(fill="both", expand=True)

        for joint in sim.JOINT_NAMES:
            row = ttk.Frame(container)
            row.pack(fill="x", pady=4)
            ttk.Label(row, text=joint, width=22).pack(side="left")
            slider = tk.Scale(
                row,
                from_=-JOINT_LIMIT_DEG,
                to=JOINT_LIMIT_DEG,
                orient="horizontal",
                length=320,
                resolution=1,
                label=None,
                command=lambda deg, j=joint: self.on_slide(j, deg),
            )
            slider.set(round(math.degrees(sim.LOOK_AT_TABLE_POSE[joint])))
            slider.pack(side="left", padx=8)
            self.sliders[joint] = slider

        buttons = ttk.Frame(container)
        buttons.pack(fill="x", pady=(12, 0))
        ttk.Button(buttons, text="Blick auf Tisch", command=self.go_look_at_table).pack(side="left", padx=4)
        ttk.Button(buttons, text="Home (0°)", command=self.go_home).pack(side="left", padx=4)
        ttk.Button(buttons, text="Beenden", command=self.on_close).pack(side="right", padx=4)

        ttk.Separator(container).pack(fill="x", pady=12)

        coords = ttk.Frame(container)
        coords.pack(fill="x")
        ttk.Label(coords, text="Zielposition (Raumkoordinaten, Meter):").pack(anchor="w")

        start_pose = kin.forward_kinematics(sim.LOOK_AT_TABLE_POSE)
        start_xyz = start_pose[:3, 3]
        self.target_orientation = start_pose[:3, :3]

        entry_row = ttk.Frame(coords)
        entry_row.pack(fill="x", pady=4)
        self.coord_vars: dict[str, tk.StringVar] = {}
        for axis, value in zip("xyz", start_xyz):
            ttk.Label(entry_row, text=f"{axis}:").pack(side="left")
            var = tk.StringVar(value=f"{value:.3f}")
            ttk.Entry(entry_row, textvariable=var, width=8).pack(side="left", padx=(2, 10))
            self.coord_vars[axis] = var

        ttk.Button(entry_row, text="Fahre zu Position", command=self.go_to_position).pack(side="left", padx=4)

        self.coord_status = ttk.Label(coords, text="")
        self.coord_status.pack(anchor="w", pady=(4, 0))

        self.send_pose(sim.LOOK_AT_TABLE_POSE)

    def on_slide(self, joint: str, degrees_str: str) -> None:
        self.publish(joint, math.radians(float(degrees_str)))

    def publish(self, joint: str, radians: float) -> None:
        from gz.msgs.double_pb2 import Double

        msg = Double()
        msg.data = radians
        self.publishers[joint].publish(msg)

    def send_pose(self, pose: dict) -> None:
        for joint, angle in pose.items():
            self.sliders[joint].set(round(math.degrees(angle)))
            self.publish(joint, angle)

    def go_safely(self, target_pose: dict) -> None:
        """Fährt zunächst die kollisionsfreie Zwischenpose an, bevor das
        eigentliche Ziel gesendet wird (siehe SAFE_STAGING_POSE in
        fr20_ceiling_gazebo.py) - ein direkter Sprung würde je nach
        Startkonfiguration mitten durch den Tisch schwingen."""
        self.send_pose(sim.SAFE_STAGING_POSE)
        self.update()
        time.sleep(1.0)
        self.send_pose(target_pose)

    def go_look_at_table(self) -> None:
        self.go_safely(sim.LOOK_AT_TABLE_POSE)

    def go_home(self) -> None:
        self.go_safely({j: 0.0 for j in sim.JOINT_NAMES})

    def go_to_position(self) -> None:
        try:
            xyz = [float(self.coord_vars[axis].get()) for axis in "xyz"]
        except ValueError:
            self.coord_status.config(text="Ungültige Koordinate - bitte Zahlen eingeben.")
            return

        target_pose = np.eye(4)
        target_pose[:3, :3] = self.target_orientation
        target_pose[:3, 3] = xyz

        seed = {j: math.radians(self.sliders[j].get()) for j in sim.JOINT_NAMES}
        solution = kin.inverse_kinematics(target_pose, seed)
        if solution is None:
            self.coord_status.config(text="Keine Lösung gefunden (Punkt evtl. außerhalb der Reichweite).")
            return

        self.coord_status.config(text="Position erreicht.")
        self.send_pose(solution)

    def on_close(self) -> None:
        if self.gz_proc.poll() is None:
            self.gz_proc.terminate()
            try:
                self.gz_proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.gz_proc.kill()
        self.destroy()


def main() -> None:
    for tool in ("gz", "ros2"):
        sim.check_tool(tool)

    sim.isolate_gz_partition()

    sim.WORK_DIR.mkdir(exist_ok=True)
    urdf_path = sim.WORK_DIR / f"{sim.MODEL_NAME}.urdf"
    world_path = sim.WORK_DIR / "world.sdf"

    print("Erzeuge URDF für den kopfüber montierten FR20 ...")
    sim.write_mount_urdf(urdf_path)
    sim.append_joint_position_controllers(urdf_path)

    print("Konvertiere URDF zu SDF, damit der Roboter von Anfang an Teil der Welt ist ...")
    robot_model_sdf = sim.urdf_to_sdf_model(urdf_path)

    print("Erzeuge SDF-Welt (Raum + Tisch + Roboter) ...")
    world_path.write_text(sim.build_world_sdf(robot_model_sdf))

    print("Starte Gazebo Jetty (gz-sim) ...")
    gz_proc = subprocess.Popen(
        ["gz", "sim", "-r", str(world_path)], env=sim.gazebo_env_with_fr20_meshes()
    )

    try:
        sim.wait_for_world(sim.WORLD_NAME)
        print("Warte, bis die Gelenk-Controller aktiv sind ...")
        time.sleep(2.0)

        from gz.transport import Node
        from gz.msgs.double_pb2 import Double

        node = Node()
        publishers = {
            j: node.advertise(f"/{sim.MODEL_NAME}/{j}/cmd_pos", Double) for j in sim.JOINT_NAMES
        }

        print("Fahre zunächst in die sichere Zwischenpose (Kollisionsvermeidung) ...")
        sim.send_to_safe_staging_pose(publishers)

        print("Öffne Steuerungs-GUI ...")
        app = RobotGui(publishers, gz_proc)
        app.mainloop()
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
