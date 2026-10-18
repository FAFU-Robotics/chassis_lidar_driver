"""
RSView-style point cloud visualization for RoboSense Airy (WRS / Panda3D).
"""
from __future__ import annotations

import math

import numpy as np
from panda3d.core import (
    Geom, GeomEnums, GeomNode, GeomPoints, GeomVertexArrayFormat,
    GeomVertexData, GeomVertexFormat, InternalName, LineSegs,
    NodePath, OmniBoundingVolume, TransparencyAttrib, WindowProperties,
)

# RSView dark navy background
RSVIEW_BG = (0.04, 0.05, 0.10)
DEFAULT_POINT_SIZE_PX = 2.5
DEFAULT_MAX_POINTS = 30_000
EGO_FILTER_RADIUS = 0.30


def intensity_colormap(intensity: np.ndarray,
                       vmin: float = 0.0,
                       vmax: float = 255.0) -> np.ndarray:
    """Jet colormap: blue (low) → cyan → green → yellow (high)."""
    t = np.clip(
        (intensity.astype(np.float32) - vmin) / max(vmax - vmin, 1.0),
        0.0, 1.0)
    rgba = np.empty((len(t), 4), dtype=np.float32)
    rgba[:, 0] = np.clip(1.5 - np.abs(4.0 * t - 3.0), 0.0, 1.0)
    rgba[:, 1] = np.clip(1.5 - np.abs(4.0 * t - 2.0), 0.0, 1.0)
    rgba[:, 2] = np.clip(1.5 - np.abs(4.0 * t - 1.0), 0.0, 1.0)
    rgba[:, 3] = 1.0
    return rgba


def auto_intensity_range(intensity: np.ndarray) -> tuple[float, float]:
    """Fast intensity stretch — min/max on the display set."""
    if len(intensity) < 16:
        return 0.0, 255.0
    vmin = float(intensity.min())
    vmax = float(intensity.max())
    if vmax <= vmin + 1.0:
        vmax = vmin + 1.0
    return vmin, vmax


def filter_near_origin(pcd: np.ndarray, intensity: np.ndarray,
                       min_radius: float = EGO_FILTER_RADIUS
                       ) -> tuple[np.ndarray, np.ndarray]:
    if len(pcd) == 0:
        return pcd, intensity
    r2 = min_radius * min_radius
    dist2 = np.einsum("ij,ij->i", pcd, pcd)
    mask = dist2 >= r2
    return pcd[mask], intensity[mask]


def subsample(pcd: np.ndarray, intensity: np.ndarray,
              max_points: int) -> tuple[np.ndarray, np.ndarray]:
    """
    Deterministic subsampling.
    Avoid random sampling every frame because it causes
    visible point-cloud flicker.
    """
    n = len(pcd)
    if n <= max_points:
        return pcd, intensity
    step = max(1, n // max_points)
    idx = np.arange(0, n, step, dtype=np.int64)
    if len(idx) > max_points:
        idx = idx[:max_points]
    return pcd[idx], intensity[idx]


def intensity_colormap_u8(intensity: np.ndarray, out: np.ndarray,
                          vmin: float = 0.0,
                          vmax: float = 255.0) -> None:
    """Write jet RGBA directly into *out* (N,4) uint8."""
    t = (intensity.astype(np.float32) - vmin) / max(vmax - vmin, 1.0)
    np.clip(t, 0.0, 1.0, out=t)
    out[:, 0] = (np.clip(1.5 - np.abs(4.0 * t - 3.0), 0.0, 1.0) * 255.0).astype(np.uint8)
    out[:, 1] = (np.clip(1.5 - np.abs(4.0 * t - 2.0), 0.0, 1.0) * 255.0).astype(np.uint8)
    out[:, 2] = (np.clip(1.5 - np.abs(4.0 * t - 1.0), 0.0, 1.0) * 255.0).astype(np.uint8)
    out[:, 3] = 255


def prepare_display_frame(pcd: np.ndarray, intensity: np.ndarray,
                          max_points: int = DEFAULT_MAX_POINTS,
                          ego_radius: float = EGO_FILTER_RADIUS
                          ) -> tuple[np.ndarray, np.ndarray]:
    """Filter + downsample on the UDP thread (keeps Panda3D main thread light)."""
    pcd, intensity = filter_near_origin(pcd, intensity, ego_radius)
    return subsample(pcd, intensity, max_points)


def apply_rsview_scene(base,
                       bg: tuple[float, float, float] = RSVIEW_BG,
                       title: str = "RoboSense Airy — RSView"):
    """
    Switch WRS World from robot-planning defaults to lidar-viewer mode.

    Critical: disable CartoonInk post-filter (causes white bg + yellow blob).
    """
    base.setBackgroundColor(*bg)

    if hasattr(base, "filter") and base.filter is not None:
        base.filter.delCartoonInk()

    base.render.clearLight()
    base.render.setLightOff()

    if hasattr(base, "inputmgr"):
        base.inputmgr.toggle_rotcenter = False
        if hasattr(base.inputmgr, "rot_center"):
            base.inputmgr.rot_center.remove()

    if hasattr(base, "win") and base.win is not None:
        props = WindowProperties(base.win.getProperties())
        props.setTitle("RoboSense AirView")
        base.win.requestProperties(props)

def make_ground_grid(half_extent: float = 30.0,
                     step: float = 5.0,
                     rings: tuple[float, ...] = (10.0, 20.0)) -> NodePath:
    """Subtle grid for dark background (RSView style)."""
    ls = LineSegs()
    ls.setThickness(1.0)
    ls.setColor(0.18, 0.22, 0.30, 0.45)

    n = int(half_extent / step)
    for i in range(-n, n + 1):
        v = i * step
        ls.moveTo(v, -half_extent, 0.0)
        ls.drawTo(v, half_extent, 0.0)
        ls.moveTo(-half_extent, v, 0.0)
        ls.drawTo(half_extent, v, 0.0)

    ls.setColor(0.25, 0.30, 0.40, 0.55)
    for radius in rings:
        for k in range(36):
            a0 = 2.0 * math.pi * k / 36
            a1 = 2.0 * math.pi * (k + 1) / 36
            ls.moveTo(radius * math.cos(a0), radius * math.sin(a0), 0.0)
            ls.drawTo(radius * math.cos(a1), radius * math.sin(a1), 0.0)

    np_grid = NodePath(ls.create())
    np_grid.setPos(0.0, 0.0, 0.0)
    np_grid.setHpr(0.0, 0.0, 0.0)
    np_grid.setTransparency(TransparencyAttrib.MDual)
    np_grid.setLightOff()
    np_grid.setBin("background", 0)
    return np_grid


class IntensityPointCloudRenderer:
    """Native GeomPoints — bulk numpy buffer update, no geometry rebuild."""

    def __init__(self, base, max_points: int = DEFAULT_MAX_POINTS,
                 point_size_px: float = DEFAULT_POINT_SIZE_PX):
        self.max_points = max_points
        self._auto_stretch = True

        vformat = GeomVertexFormat()
        af_v = GeomVertexArrayFormat()
        af_v.addColumn(InternalName.getVertex(), 3,
                       GeomEnums.NTFloat32, GeomEnums.CPoint)
        vformat.addArray(af_v)
        af_c = GeomVertexArrayFormat()
        af_c.addColumn(InternalName.getColor(), 4,
                       GeomEnums.NTUint8, GeomEnums.CColor)
        vformat.addArray(af_c)
        vformat = GeomVertexFormat.registerFormat(vformat)

        self._vdata = GeomVertexData(
            "airy_intensity_pc", vformat, GeomEnums.UHDynamic)
        self._vdata.setNumRows(max_points)

        self._geom = Geom(self._vdata)
        self._prim = GeomPoints(GeomEnums.UHStatic)
        self._geom.addPrimitive(self._prim)

        self._node = GeomNode("airy_intensity_pc_node")
        self._node.addGeom(self._geom)
        self._node.setBounds(OmniBoundingVolume())
        self._node.setFinal(True)

        self._np = base.render.attachNewNode(self._node)
        self._np.setPos(0.0, 0.0, 0.0)
        self._np.setHpr(0.0, 0.0, 0.0)  # 与 Pitch 矫正后的底盘水平系对齐
        self._np.setRenderModeThickness(point_size_px)
        self._np.setLightOff()
        self._np.setBin("fixed", 40)
        self._np.setDepthWrite(False)

        self._buf_v = np.zeros((max_points, 3), dtype=np.float32)
        self._buf_c = np.zeros((max_points, 4), dtype=np.uint8)

    def update(self, points: np.ndarray, intensity: np.ndarray,
               vmin: float | None = None, vmax: float | None = None,
               danger_mask: np.ndarray | None = None):
        n = min(len(points), self.max_points)
        if n == 0:
            self._prim.clearVertices()
            return

        if vmin is None or vmax is None:
            if self._auto_stretch:
                vmin, vmax = auto_intensity_range(intensity[:n])
            else:
                vmin, vmax = 0.0, 255.0

        np.copyto(self._buf_v[:n], points[:n])
        intensity_colormap_u8(intensity[:n], self._buf_c[:n],
                              vmin=vmin, vmax=vmax)

        if danger_mask is not None and len(danger_mask) >= n:
            danger = danger_mask[:n]
            self._buf_c[:n][danger, 0] = 255
            self._buf_c[:n][danger, 1] = 0
            self._buf_c[:n][danger, 2] = 0
            self._buf_c[:n][danger, 3] = 255

        self._vdata.setNumRows(n)
        self._vdata.modifyArrayHandle(0).copyDataFrom(self._buf_v[:n])
        self._vdata.modifyArrayHandle(1).copyDataFrom(self._buf_c[:n])

        self._prim.clearVertices()
        self._prim.addNextVertices(n)

    def detach(self):
        self._np.removeNode()
