"""Kollisionsprüfung des FR20-Arms gegen Tisch, Baugruppe und Biegekopf-Testachse.

RViz hat keine Physik - ohne diese Prüfung fährt der Arm in der Anzeige (und
am echten Roboter!) einfach durch den Tisch. Geprüft wird mit der echten
Geometrie der Armglieder: Aus den STL-Meshes von fairino_description wird
pro Glied eine Punktwolke (Voxel-Raster, PointCloud) gebildet, per
Vorwärtskinematik in Weltkoordinaten transformiert und gegen
achsparallele Hindernis-Quader (Tisch, Baugruppe, Biegekopf) geprüft.

Eine Pose gilt als sicher, wenn jeder Punkt mindestens CLEARANCE_MARGIN_M
Abstand zu jedem Hindernis hat. Eine Bewegung a -> b gilt als sicher, wenn
alle Zwischenposen der linearen Gelenkraum-Interpolation sicher sind - das
ist genau die Bahn, die rviz_backend (synchrone Gelenkfahrt) bzw. ein
PTP/MoveJ am echten Roboter fährt.

Grenzen: Der Voxel-Abstand (MESH_VOXEL_M) ist bewusst kleiner als
CLEARANCE_MARGIN_M, sodass auch eine Fläche zwischen zwei Punkten den
Mindestabstand nicht unbemerkt unterschreitet. Ein Werkzeug/Greifer am
Flansch ist noch NICHT modelliert - sobald es feststeht, als eigenes Glied
in TOOL_BOXES ergänzen. Ersetzt keine sicherheitsbewertete Überwachung
(siehe SICHERHEIT.md).
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import fr20_kinematics as kin
import fr20_scene as sim

# Mindestabstand (m) zwischen Armgeometrie und Hindernissen.
CLEARANCE_MARGIN_M = 0.03

# Rasterweite für die Punktwolken der Armglieder.
MESH_VOXEL_M = 0.02

# Maximale Gelenkänderung (rad) zwischen zwei geprüften Zwischenposen einer Bewegung.
# 0.5° bewegt die Handgelenkspitze bei voll gestrecktem Arm (~1.9 m) um ca. 1.7 cm.
PATH_STEP_RAD = np.radians(0.5)

# Obergrenze für den Abstand eines Armpunkts von jeder Gelenkachse (m): Summe der
# Gliedlängen 0.215 + 1.0 + 0.716 + 0.166 + 0.138 + 0.12 plus Mesh-Überstand.
MAX_REACH_M = 2.6

# Armglieder, deren Geometrie geprüft wird (base_link hängt fest an der Decke).
CHECKED_LINKS = ("shoulder_link", "upperarm_link", "forearm_link", "wrist1_link", "wrist2_link", "wrist3_link")


@dataclass(frozen=True)
class Box:
    """Achsparalleler Quader in Weltkoordinaten (min/max-Ecke, m)."""

    name: str
    lo: tuple[float, float, float]
    hi: tuple[float, float, float]


def obstacles(offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> list[Box]:
    """Hindernisse der Zelle. `offset_xyz` = kalibrierter Tisch-Offset (verschiebt Tisch und
    Baugruppe mit, siehe sim.load_table_offset)."""
    dx, dy, _ = offset_xyz
    tx, ty = sim.TABLE_CENTER[0] + dx, sim.TABLE_CENTER[1] + dy
    hx, hy = sim.TABLE_SIZE_XY[0] / 2, sim.TABLE_SIZE_XY[1] / 2
    ax, ay = sim.ASSEMBLY_SIZE_XY_M[0] / 2, sim.ASSEMBLY_SIZE_XY_M[1] / 2
    bx, by = sim.BENDING_HEAD_XY
    shaft = sim.BENDING_HEAD_SHAFT_LENGTH_M
    return [
        Box("Tisch", (tx - hx, ty - hy, 0.0), (tx + hx, ty + hy, sim.TABLE_TOP_Z)),
        Box("Baugruppe", (tx - ax, ty - ay, sim.TABLE_TOP_Z), (tx + ax, ty + ay, sim.ASSEMBLY_TOP_Z)),
        # Ständer + Welle (mit Fahne, Radius ~0.1 m) der Biegekopf-Testachse.
        Box("Biegekopf", (bx - 0.1, by - 0.05, 0.0), (bx + 0.1, by + shaft, sim.BENDING_HEAD_Z + 0.1)),
        Box("Boden", (-sim.ROOM_SIZE_XY, -sim.ROOM_SIZE_XY, -1.0), (sim.ROOM_SIZE_XY, sim.ROOM_SIZE_XY, 0.0)),
    ]


# ---------------------------------------------------------------------------
# Punktwolken der Armglieder
# ---------------------------------------------------------------------------

def _read_binary_stl_vertices(path: Path) -> np.ndarray:
    data = path.read_bytes()
    (count,) = struct.unpack_from("<I", data, 80)
    # Pro Dreieck: Normale (3 float) + 3 Eckpunkte (9 float) + 2 Byte Attribut = 50 Byte.
    record = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    tris = np.frombuffer(data, dtype=record, count=count, offset=84)
    return tris["v"].reshape(-1, 3).astype(float)


def _voxel_downsample(points: np.ndarray, voxel: float) -> np.ndarray:
    keys = np.floor(points / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(idx)]


def _mesh_path(link: str) -> Path:
    return sim.REPO_ROOT / "fairino_description" / "meshes" / "fairino20_v6" / f"{link}.STL"


_link_points_cache: dict[str, np.ndarray] | None = None


def link_points() -> dict[str, np.ndarray]:
    """Punktwolke (N x 3, im Glied-Koordinatensystem) pro geprüftem Armglied, gecacht.
    Die Visual-Ursprünge im FR20-URDF sind alle Identität, daher direkt im Gliedframe."""
    global _link_points_cache
    if _link_points_cache is None:
        _link_points_cache = {
            link: _voxel_downsample(_read_binary_stl_vertices(_mesh_path(link)), MESH_VOXEL_M)
            for link in CHECKED_LINKS
        }
    return _link_points_cache


# ---------------------------------------------------------------------------
# Abstand / Kollision
# ---------------------------------------------------------------------------

def link_frames(joints: dict[str, float]) -> dict[str, np.ndarray]:
    """Weltpose (4x4) jedes Glieds entlang der Kette world -> tool0."""
    return kin.load_chain().link_frames(joints)


def _world_points(joints: dict[str, float]) -> tuple[np.ndarray, list[tuple[str, int]]]:
    frames = link_frames(joints)
    clouds = []
    spans = []
    for link, pts in link_points().items():
        t = frames[link]
        clouds.append(pts @ t[:3, :3].T + t[:3, 3])
        spans.append((link, len(pts)))
    return np.vstack(clouds), spans


def _box_distance(points: np.ndarray, box: Box) -> np.ndarray:
    """Vorzeichenbehafteter Abstand jedes Punkts zum Quader (negativ = innen)."""
    lo, hi = np.array(box.lo), np.array(box.hi)
    center, half = (lo + hi) / 2, (hi - lo) / 2
    q = np.abs(points - center) - half
    outside = np.linalg.norm(np.maximum(q, 0.0), axis=1)
    inside = np.minimum(q.max(axis=1), 0.0)
    return outside + inside


@dataclass
class Clearance:
    """Kleinster Abstand einer Pose zu den Hindernissen und wer ihn verursacht."""

    distance_m: float
    link: str
    obstacle: str

    @property
    def ok(self) -> bool:
        return self.distance_m >= CLEARANCE_MARGIN_M

    def __str__(self) -> str:
        return f"{self.link} -> {self.obstacle}: {self.distance_m * 1000:.0f} mm"


def pose_clearance(joints: dict[str, float], boxes: list[Box] | None = None) -> Clearance:
    """Kleinster Abstand der Armgeometrie in Pose `joints` (rad) zu allen Hindernissen."""
    boxes = boxes if boxes is not None else obstacles()
    points, spans = _world_points(joints)
    dists = np.stack([_box_distance(points, box) for box in boxes])  # (Hindernisse, Punkte)
    bi, pi = np.unravel_index(int(np.argmin(dists)), dists.shape)
    acc = 0
    for link, n in spans:  # Punktindex -> Gliedname
        if pi < acc + n:
            break
        acc += n
    return Clearance(float(dists[bi, pi]), link, boxes[bi].name)


def is_pose_safe(joints: dict[str, float], boxes: list[Box] | None = None) -> bool:
    return pose_clearance(joints, boxes).ok


def path_clearance(
    a: dict[str, float], b: dict[str, float], boxes: list[Box] | None = None
) -> Clearance:
    """Kleinster Abstand entlang der linearen Gelenkraum-Bahn a -> b (inkl. Endpunkte).

    Schrittweite adaptiv ("conservative advancement"): Ist an einer Zwischenpose der Abstand
    d, kann sich kein Armpunkt bis zur nächsten Probe um mehr als d - CLEARANCE_MARGIN_M
    bewegen (Abschätzung über MAX_REACH_M und die Summe der Gelenkänderungen). Weit weg vom
    Tisch wird so in großen Schritten geprüft, nah dran fein - höchstens PATH_STEP_RAD pro
    Gelenk zwischen zwei Proben."""
    boxes = boxes if boxes is not None else obstacles()
    deltas = [abs(b[j] - a[j]) for j in sim.JOINT_NAMES]
    max_delta, l1 = max(deltas), sum(deltas)
    min_step = 1.0 if max_delta <= PATH_STEP_RAD else PATH_STEP_RAD / max_delta
    worst = Clearance(float("inf"), "", "")
    frac = 0.0
    while True:
        inter = {j: a[j] + (b[j] - a[j]) * frac for j in sim.JOINT_NAMES}
        c = pose_clearance(inter, boxes)
        if c.distance_m < worst.distance_m:
            worst = c
            if not c.ok:
                return worst  # erste Kollision genügt
        if frac >= 1.0:
            return worst
        safe_step = (c.distance_m - CLEARANCE_MARGIN_M) / (MAX_REACH_M * l1) if l1 > 0 else 1.0
        frac = min(1.0, frac + max(min_step, safe_step))


def is_path_safe(a: dict[str, float], b: dict[str, float], boxes: list[Box] | None = None) -> bool:
    return path_clearance(a, b, boxes).ok
