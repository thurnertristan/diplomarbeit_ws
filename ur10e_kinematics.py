"""Vorwärts-/inverse Kinematik für den deckenmontierten UR10e.

Baut die Kinematikkette nicht aus manuell abgetippten DH-Parametern auf,
sondern liest sie direkt aus dem von ur10e_ceiling_gazebo.py erzeugten
URDF (world -> ... -> tool0). So stimmt die Kette garantiert mit dem
Modell überein, das tatsächlich in Gazebo simuliert wird - inklusive der
Deckenmontage-Transformation und aller fixen Zwischen-Joints
(base_link_inertia, flange, ...), die ein von Hand erstelltes DH-Modell
leicht übersieht.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

import numpy as np

import ur10e_ceiling_gazebo as sim


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def _origin_transform(origin_elem: ET.Element | None) -> np.ndarray:
    t = np.eye(4)
    if origin_elem is None:
        return t
    xyz = [float(v) for v in origin_elem.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in origin_elem.get("rpy", "0 0 0").split()]
    t[:3, :3] = _rpy_to_matrix(*rpy)
    t[:3, 3] = xyz
    return t


def _axis_rotation(axis_elem: ET.Element | None, angle: float) -> np.ndarray:
    axis = [1.0, 0.0, 0.0]
    if axis_elem is not None:
        axis = [float(v) for v in axis_elem.get("xyz", "1 0 0").split()]
    axis = np.array(axis) / np.linalg.norm(axis)
    x, y, z = axis
    c, s, one_c = np.cos(angle), np.sin(angle), 1 - np.cos(angle)
    r = np.array([
        [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
        [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
        [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
    ])
    t = np.eye(4)
    t[:3, :3] = r
    return t


class UrdfChain:
    """Kette von Joints von `root_link` bis `tip_link`, aus einem URDF geparst."""

    def __init__(self, urdf_path, root_link: str = "world", tip_link: str = "tool0") -> None:
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        joints_by_parent: dict[str, list[ET.Element]] = {}
        for joint in root.findall("joint"):
            parent = joint.find("parent").get("link")
            joints_by_parent.setdefault(parent, []).append(joint)

        chain = self._find_chain(joints_by_parent, root_link, tip_link)
        if chain is None:
            raise ValueError(f"Kein Pfad von '{root_link}' zu '{tip_link}' im URDF gefunden.")
        self.chain = chain
        self.movable_joint_names = [j.get("name") for j in chain if j.get("type") != "fixed"]

    @staticmethod
    def _find_chain(
        joints_by_parent: dict[str, list[ET.Element]], link: str, tip_link: str
    ) -> list[ET.Element] | None:
        """Tiefensuche, da ein Link (z.B. wrist_3_link) mehrere Kind-Joints haben kann,
        von denen nur einer auf einem Pfad zu `tip_link` liegt (z.B. ft_frame ist eine
        Sackgasse, flange->tool0 nicht)."""
        if link == tip_link:
            return []
        for joint in joints_by_parent.get(link, []):
            child = joint.find("child").get("link")
            sub_chain = UrdfChain._find_chain(joints_by_parent, child, tip_link)
            if sub_chain is not None:
                return [joint] + sub_chain
        return None

    def forward_kinematics(self, joint_angles: dict) -> np.ndarray:
        t = np.eye(4)
        for joint in self.chain:
            t = t @ _origin_transform(joint.find("origin"))
            if joint.get("type") != "fixed":
                angle = joint_angles[joint.get("name")]
                t = t @ _axis_rotation(joint.find("axis"), angle)
        return t


def _load_chain() -> UrdfChain:
    urdf_path = sim.WORK_DIR / f"{sim.MODEL_NAME}.urdf"
    if not urdf_path.exists():
        raise FileNotFoundError(
            f"{urdf_path} nicht gefunden - das URDF wird beim Start der Simulation erzeugt."
        )
    return UrdfChain(urdf_path)


def forward_kinematics(joint_angles: dict) -> np.ndarray:
    """Pose von tool0 im Weltkoordinatensystem für gegebene Gelenkwinkel (rad)."""
    return _load_chain().forward_kinematics(joint_angles)


def _pose_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    """6D-Fehler (Position + Rotationsvektor) zwischen zwei Posen."""
    pos_err = target[:3, 3] - current[:3, 3]
    r_err = target[:3, :3] @ current[:3, :3].T
    angle = np.arccos(np.clip((np.trace(r_err) - 1.0) / 2.0, -1.0, 1.0))
    if angle < 1e-8:
        rot_err = np.zeros(3)
    else:
        axis = np.array([
            r_err[2, 1] - r_err[1, 2],
            r_err[0, 2] - r_err[2, 0],
            r_err[1, 0] - r_err[0, 1],
        ]) / (2.0 * np.sin(angle))
        rot_err = axis * angle
    return np.concatenate([pos_err, rot_err])


def inverse_kinematics(
    target_pose: np.ndarray,
    seed: dict,
    joint_limit_rad: float = np.pi,
    max_iterations: int = 500,
    tolerance: float = 1e-4,
    max_step_rad: float = 0.2,
) -> dict | None:
    """Numerische IK (gedämpfte Least-Squares) ausgehend von der aktuellen Pose.

    Liefert die nächstgelegene Lösung zur Startpose `seed` (statt einer der
    bis zu acht analytischen UR-Lösungen) - für eine interaktive GUI, bei der
    der Arm möglichst nah an der aktuellen Konfiguration bleiben soll, ist das
    das gewünschte Verhalten.
    """
    chain = _load_chain()
    names = chain.movable_joint_names
    angles = np.array([seed[j] for j in names], dtype=float)
    damping = 0.05
    delta = 1e-6

    for _ in range(max_iterations):
        joint_dict = dict(zip(names, angles))
        current_pose = chain.forward_kinematics(joint_dict)
        error = _pose_error(current_pose, target_pose)
        if np.linalg.norm(error) < tolerance:
            return {j: float(a) for j, a in zip(names, angles)}

        jacobian = np.zeros((6, len(names)))
        for i in range(len(names)):
            perturbed = angles.copy()
            perturbed[i] += delta
            perturbed_pose = chain.forward_kinematics(dict(zip(names, perturbed)))
            pos_diff = (perturbed_pose[:3, 3] - current_pose[:3, 3]) / delta
            # (dR/dtheta) @ R^T ist schiefsymmetrisch und liefert die Winkel-
            # geschwindigkeit im Weltsystem; die rohe Matrixdifferenz allein wäre
            # nicht schiefsymmetrisch und würde eine falsche Jacobi-Spalte ergeben.
            rot_diff_matrix = (perturbed_pose[:3, :3] - current_pose[:3, :3]) @ current_pose[:3, :3].T / delta
            rot_diff = np.array([
                rot_diff_matrix[2, 1] - rot_diff_matrix[1, 2],
                rot_diff_matrix[0, 2] - rot_diff_matrix[2, 0],
                rot_diff_matrix[1, 0] - rot_diff_matrix[0, 1],
            ]) / 2.0
            jacobian[:3, i] = pos_diff
            jacobian[3:, i] = rot_diff

        jjt = jacobian @ jacobian.T + (damping ** 2) * np.eye(6)
        delta_angles = jacobian.T @ np.linalg.solve(jjt, error)
        step_norm = np.max(np.abs(delta_angles))
        if step_norm > max_step_rad:
            delta_angles *= max_step_rad / step_norm
        angles = np.clip(angles + delta_angles, -joint_limit_rad, joint_limit_rad)

    return None
