"""Vorwärts-/inverse Kinematik für den deckenmontierten Fairino FR20.

Baut die Kinematikkette nicht aus manuell abgetippten DH-Parametern auf,
sondern liest sie direkt aus dem von fr20_scene.py erzeugten
URDF (world -> ... -> tool0). So stimmt die Kette garantiert mit dem
Modell überein, das in RViz angezeigt wird - inklusive der
Deckenmontage-Transformation und des künstlich angehängten tool0-Frames,
die ein von Hand erstelltes DH-Modell leicht übersieht.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

import numpy as np

import fr20_scene as sim


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    # Roll/Pitch/Yaw -> 3x3 Rotationsmatrix
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def _origin_transform(origin_elem: ET.Element | None) -> np.ndarray:
    # baut die 4x4-Transformation aus dem <origin>-Element eines Joints
    t = np.eye(4)
    if origin_elem is None:
        return t
    xyz = [float(v) for v in origin_elem.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in origin_elem.get("rpy", "0 0 0").split()]
    t[:3, :3] = _rpy_to_matrix(*rpy)
    t[:3, 3] = xyz
    return t


_axis_cache: dict[int, tuple[float, float, float]] = {}


def _axis_rotation(axis_elem: ET.Element | None, angle: float) -> np.ndarray:
    # Rotation eines beweglichen Gelenks um seine Achse mit gegebenem Winkel
    key = id(axis_elem)
    if key not in _axis_cache:
        axis = [1.0, 0.0, 0.0]
        if axis_elem is not None:
            axis = [float(v) for v in axis_elem.get("xyz", "1 0 0").split()]
        axis = np.array(axis) / np.linalg.norm(axis)
        _axis_cache[key] = tuple(float(v) for v in axis)
    x, y, z = _axis_cache[key]
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
        # URDF laden und alle Joints nach Elternglied sortieren
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        joints_by_parent: dict[str, list[ET.Element]] = {}
        for joint in root.findall("joint"):
            parent = joint.find("parent").get("link")
            joints_by_parent.setdefault(parent, []).append(joint)

        # Kette von root_link bis tip_link finden
        chain = self._find_chain(joints_by_parent, root_link, tip_link)
        if chain is None:
            raise ValueError(f"Kein Pfad von '{root_link}' zu '{tip_link}' im URDF gefunden.")
        self.chain = chain
        self.movable_joint_names = [j.get("name") for j in chain if j.get("type") != "fixed"]
        # Vorab ausgewertet (statt pro FK-Aufruf XML-Strings zu parsen): pro Joint
        # (Gelenkname oder None bei fix, Ursprungs-Transformation, Achse, Kind-Link).
        self.steps: list[tuple[str | None, np.ndarray, ET.Element | None, str]] = [
            (
                None if j.get("type") == "fixed" else j.get("name"),
                _origin_transform(j.find("origin")),
                j.find("axis"),
                j.find("child").get("link"),
            )
            for j in chain
        ]

    @staticmethod
    def _find_chain(
        joints_by_parent: dict[str, list[ET.Element]], link: str, tip_link: str
    ) -> list[ET.Element] | None:
        """Tiefensuche, da ein Link (z.B. wrist_3_link) mehrere Kind-Joints haben kann,
        von denen nur einer auf einem Pfad zu `tip_link` liegt (z.B. ft_frame ist eine
        Sackgasse, flange->tool0 nicht)."""
        if link == tip_link:
            return []  # Ziel gefunden
        for joint in joints_by_parent.get(link, []):
            child = joint.find("child").get("link")
            sub_chain = UrdfChain._find_chain(joints_by_parent, child, tip_link)
            if sub_chain is not None:
                return [joint] + sub_chain
        return None  # Sackgasse

    def link_frames(self, joint_angles: dict) -> dict[str, np.ndarray]:
        """Weltpose (4x4) jedes Kind-Links entlang der Kette."""
        frames = {}
        t = np.eye(4)
        for name, origin, axis, child in self.steps:
            t = t @ origin
            if name is not None:
                t = t @ _axis_rotation(axis, joint_angles[name])
            frames[child] = t
        return frames

    def forward_kinematics(self, joint_angles: dict) -> np.ndarray:
        # multipliziert alle Kettenglieder durch, um die Werkzeugpose zu erhalten
        t = np.eye(4)
        for name, origin, axis, _child in self.steps:
            t = t @ origin
            if name is not None:
                t = t @ _axis_rotation(axis, joint_angles[name])
        return t


_chain_cache: UrdfChain | None = None


def load_chain() -> UrdfChain:
    """Lädt (und cached) die Gelenkkette aus dem generierten URDF - öffentlich,
    da robot_program.py sie ebenfalls direkt braucht. Cache spart wiederholtes
    URDF-Parsen bei den vielen IK-Aufrufen pro Punkt (resolve_point). Das URDF wird
    beim ersten Aufruf pro Prozess neu erzeugt, damit Änderungen in fr20_scene (z.B.
    FLANGE_OFFSET_M) nie an einer veralteten Datei vorbeilaufen."""
    global _chain_cache
    if _chain_cache is None:
        urdf_path = sim.WORK_DIR / f"{sim.MODEL_NAME}.urdf"
        sim.WORK_DIR.mkdir(exist_ok=True)
        sim.write_mount_urdf(urdf_path)
        _chain_cache = UrdfChain(urdf_path)
    return _chain_cache


def forward_kinematics(joint_angles: dict) -> np.ndarray:
    """Pose von tool0 im Weltkoordinatensystem für gegebene Gelenkwinkel (rad)."""
    return load_chain().forward_kinematics(joint_angles)


def _pose_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    """6D-Fehler (Position + Rotationsvektor) zwischen zwei Posen."""
    pos_err = target[:3, 3] - current[:3, 3]   # Positionsdifferenz
    # Rotationsdifferenz als Achse * Winkel
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


def _wrap_into_limits(angles: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    """Dreht Winkel außerhalb der Gelenkgrenzen um ±360° zurück, wenn sie dann passen (gleiche
    Stellung, z.B. j6 = -254° -> +106°), sonst wird auf die Grenze begrenzt."""
    two_pi = 2 * np.pi
    wrapped = angles.copy()
    for i, a in enumerate(angles):
        for cand in (a, a - two_pi, a + two_pi):
            if lower[i] <= cand <= upper[i]:
                wrapped[i] = cand
                break
        else:
            wrapped[i] = min(max(a, lower[i]), upper[i])
    return wrapped


def inverse_kinematics(
    target_pose: np.ndarray,
    seed: dict,
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
    chain = load_chain()
    names = chain.movable_joint_names
    # Gelenkgrenzen laut URDF (z.B. j2/j4: -265°..+85°) - Lösungen außerhalb wären am echten
    # Roboter nicht fahrbar.
    lower = np.array([sim.JOINT_LIMITS_RAD[j][0] for j in names])
    upper = np.array([sim.JOINT_LIMITS_RAD[j][1] for j in names])
    angles = np.clip(np.array([seed[j] for j in names], dtype=float), lower, upper)  # Startwerte
    damping = 0.05
    delta = 1e-6  # kleine Störung für die numerische Ableitung

    for _ in range(max_iterations):
        # aktuelle Pose berechnen und mit Ziel vergleichen
        joint_dict = dict(zip(names, angles))
        current_pose = chain.forward_kinematics(joint_dict)
        error = _pose_error(current_pose, target_pose)
        if np.linalg.norm(error) < tolerance:
            return {j: float(a) for j, a in zip(names, angles)}  # Ziel erreicht

        # Jacobi-Matrix numerisch bestimmen
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

        # gedämpfte Least-Squares-Lösung, Schritt begrenzen, Grenzen einhalten
        jjt = jacobian @ jacobian.T + (damping ** 2) * np.eye(6)
        delta_angles = jacobian.T @ np.linalg.solve(jjt, error)
        step_norm = np.max(np.abs(delta_angles))
        if step_norm > max_step_rad:
            delta_angles *= max_step_rad / step_norm
        angles = _wrap_into_limits(angles + delta_angles, lower, upper)

    return None  # keine Lösung gefunden
