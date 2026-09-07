#!/usr/bin/python3
"""Capture live ROS2 stills into ppt_materials/generated/. No chassis, no CAN."""
from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("ROS_HOME", "/tmp/ros_ppt_stills")
os.makedirs(os.environ["ROS_HOME"] + "/log", exist_ok=True)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import Normalize
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

OUT = Path("/home/fafu_robot/Desktop/chassis_lidar_drivers/ppt_materials/generated")
OUT.mkdir(parents=True, exist_ok=True)


def xyz_from_cloud(msg):
    n = int(msg.width) * int(msg.height)
    if n <= 0:
        return np.zeros((0, 3), np.float32)
    off = {f.name: f.offset for f in msg.fields}
    if not all(k in off for k in "xyz"):
        return np.zeros((0, 3), np.float32)
    dt = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": ["<f4", "<f4", "<f4"],
            "offsets": [off["x"], off["y"], off["z"]],
            "itemsize": int(msg.point_step),
        }
    )
    rec = np.frombuffer(bytes(msg.data), dtype=dt, count=n)
    xyz = np.column_stack((rec["x"], rec["y"], rec["z"]))
    return xyz[np.isfinite(xyz).all(axis=1)]


def rslidar_to_base(xyz_s):
    """Verified Airy mount: x_fwd=z_s, y_left=x_s, z_up=y_s+0.365."""
    if xyz_s.size == 0:
        return xyz_s
    x_s, y_s, z_s = xyz_s[:, 0], xyz_s[:, 1], xyz_s[:, 2]
    return np.column_stack((z_s, x_s, y_s + 0.365)).astype(np.float64)


def quat_to_rot(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def yaw_from_q(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def filter_cloud(xyz, rmin=0.45, rmax=12.0, zmin=-0.4, zmax=3.2):
    if xyz.size == 0:
        return xyz
    r = np.linalg.norm(xyz, axis=1)
    keep = (r >= rmin) & (r <= rmax) & (xyz[:, 2] >= zmin) & (xyz[:, 2] <= zmax)
    xyz = xyz[keep]
    if xyz.shape[0] < 50:
        return xyz
    # light statistical outlier on range
    med = np.median(r[keep] if keep.size else r)
    mad = np.median(np.abs(r[keep] - med)) + 1e-6
    keep2 = np.abs(np.linalg.norm(xyz, axis=1) - med) < (8.0 * 1.4826 * mad + 6.0)
    xyz = xyz[keep2]
    return xyz


def downsample(xyz, max_n=45000):
    n = xyz.shape[0]
    if n <= max_n:
        return xyz
    rng = np.random.default_rng(0)
    idx = rng.choice(n, size=max_n, replace=False)
    return xyz[idx]


def style_3d(ax, title):
    ax.set_facecolor("black")
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor((1, 1, 1, 0.15))
    ax.yaxis.pane.set_edgecolor((1, 1, 1, 0.15))
    ax.zaxis.pane.set_edgecolor((1, 1, 1, 0.15))
    ax.tick_params(colors="#cccccc", labelsize=8)
    ax.set_xlabel("X (m)", color="#dddddd")
    ax.set_ylabel("Y (m)", color="#dddddd")
    ax.set_zlabel("Z (m)", color="#dddddd")
    ax.set_title(title, color="white", fontsize=13, pad=10)
    try:
        ax.set_box_aspect((1, 1, 0.45))
    except Exception:
        pass


def save_lidar_3d(xyz_base, path):
    xyz = filter_cloud(xyz_base)
    xyz = downsample(xyz, 50000)
    fig = plt.figure(figsize=(12.8, 8.0), dpi=200, facecolor="black")
    ax = fig.add_subplot(111, projection="3d", facecolor="black")
    style_3d(ax, "Real-time LiDAR 3D point cloud  (RoboSense Airy / base_link)")
    c = xyz[:, 2]
    sc = ax.scatter(
        xyz[:, 0],
        xyz[:, 1],
        xyz[:, 2],
        c=c,
        s=0.55,
        cmap="viridis",
        linewidths=0,
        alpha=0.85,
        norm=Normalize(vmin=float(np.percentile(c, 5)), vmax=float(np.percentile(c, 95))),
    )
    cb = fig.colorbar(sc, ax=ax, shrink=0.55, pad=0.08)
    cb.set_label("height Z (m)", color="#dddddd")
    cb.ax.yaxis.set_tick_params(color="#cccccc")
    plt.setp(plt.getp(cb.ax.axes, "yticklabels"), color="#cccccc")
    ax.view_init(elev=22, azim=-145)
    def _span(a, lo=1.0, hi=99.0, pad=0.4):
        p1, p99 = np.percentile(a, [lo, hi])
        if p99 - p1 < 1.0:
            mid = 0.5 * (p1 + p99)
            p1, p99 = mid - 0.8, mid + 0.8
        return p1 - pad, p99 + pad

    ax.set_xlim(*_span(xyz[:, 0], 0.5, 99.5, 0.3))
    ax.set_ylim(*_span(xyz[:, 1], 0.5, 99.5, 0.3))
    z0, z1 = _span(xyz[:, 2], 1.0, 99.0, 0.15)
    ax.set_zlim(max(-0.3, z0), min(3.2, z1))
    fig.tight_layout()
    fig.savefig(path, facecolor="black", bbox_inches="tight")
    plt.close(fig)
    return xyz.shape[0]


def save_localization(xyz_map, pose_xyz, R, path):
    xyz = filter_cloud(xyz_map, rmin=0.3, rmax=14.0, zmin=-0.6, zmax=3.5)
    # crop around robot
    dxy = np.hypot(xyz[:, 0] - pose_xyz[0], xyz[:, 1] - pose_xyz[1])
    xyz = xyz[dxy < 10.0]
    xyz = downsample(xyz, 40000)
    fig = plt.figure(figsize=(12.8, 8.0), dpi=200, facecolor="black")
    ax = fig.add_subplot(111, projection="3d", facecolor="black")
    style_3d(ax, "MOLA localization   map frame   LiDAR + base_link pose")
    c = xyz[:, 2]
    ax.scatter(
        xyz[:, 0],
        xyz[:, 1],
        xyz[:, 2],
        c=c,
        s=0.4,
        cmap="coolwarm",
        linewidths=0,
        alpha=0.8,
        norm=Normalize(vmin=float(np.percentile(c, 8)), vmax=float(np.percentile(c, 92))),
    )
    origin = np.array(pose_xyz, dtype=np.float64)
    axes = np.eye(3) * 0.85
    colors = ("#ff3333", "#33dd55", "#4488ff")
    labels = ("X base_link", "Y", "Z")
    for i in range(3):
        vec = R @ axes[:, i]
        ax.plot(
            [origin[0], origin[0] + vec[0]],
            [origin[1], origin[1] + vec[1]],
            [origin[2], origin[2] + vec[2]],
            color=colors[i],
            lw=2.4,
            label=labels[i],
        )
    ax.scatter([origin[0]], [origin[1]], [origin[2]], c="white", s=28, depthshade=False)
    ax.legend(loc="upper left", facecolor="#111111", labelcolor="white", fontsize=8)
    ax.view_init(elev=24, azim=-145)
    def _span(a, pad=0.35):
        p1, p99 = np.percentile(a, [1, 99])
        return p1 - pad, p99 + pad

    ax.set_xlim(*_span(xyz[:, 0]))
    ax.set_ylim(*_span(xyz[:, 1]))
    z0, z1 = _span(xyz[:, 2], 0.12)
    ax.set_zlim(max(-0.3, z0), min(3.0, z1))
    fig.tight_layout()
    fig.savefig(path, facecolor="black", bbox_inches="tight")
    plt.close(fig)
    return xyz.shape[0]


def save_localmap(xyz, pose_xyz, path):
    if xyz.shape[0] < 2500:
        return False, xyz.shape[0]
    fig = plt.figure(figsize=(12.8, 8.0), dpi=200, facecolor="black")
    ax = fig.add_subplot(111, projection="3d", facecolor="black")
    style_3d(ax, "MOLA local map   /lidar_odometry/localmap_points")
    c = xyz[:, 2]
    ax.scatter(
        xyz[:, 0],
        xyz[:, 1],
        xyz[:, 2],
        c=c,
        s=2.2,
        cmap="plasma",
        linewidths=0,
        alpha=0.9,
    )
    if pose_xyz is not None:
        ax.scatter([pose_xyz[0]], [pose_xyz[1]], [pose_xyz[2]], c="cyan", s=40)
    ax.view_init(elev=48, azim=-70)
    fig.tight_layout()
    fig.savefig(path, facecolor="black", bbox_inches="tight")
    plt.close(fig)
    return True, xyz.shape[0]


def save_grid(grid_msg, pose, path):
    w, h = grid_msg.info.width, grid_msg.info.height
    res = grid_msg.info.resolution
    ox = grid_msg.info.origin.position.x
    oy = grid_msg.info.origin.position.y
    data = np.array(grid_msg.data, dtype=np.int16).reshape((h, w))
    img = np.zeros((h, w, 3), dtype=np.float64)
    img[data < 0] = (0.18, 0.18, 0.18)
    img[data == 0] = (0.86, 0.90, 0.82)
    img[(data > 0) & (data < 100)] = (0.95, 0.72, 0.28)
    img[data >= 100] = (0.82, 0.18, 0.16)
    fig = plt.figure(figsize=(10.5, 9.0), dpi=180, facecolor="white")
    ax = fig.add_subplot(111)
    ext = [ox, ox + w * res, oy, oy + h * res]
    ax.imshow(img, origin="lower", extent=ext, interpolation="nearest")
    px, py, yaw = pose
    ax.plot(px, py, marker=(3, 0, math.degrees(yaw) - 90), markersize=16, color="#1565c0")
    ax.arrow(
        px,
        py,
        0.55 * math.cos(yaw),
        0.55 * math.sin(yaw),
        head_width=0.18,
        head_length=0.22,
        fc="#1565c0",
        ec="#0d47a1",
        length_includes_head=True,
    )
    ax.set_aspect("equal")
    ax.set_xlabel("map X (m)")
    ax.set_ylabel("map Y (m)")
    ax.set_title("nav_demo obstacle_grid   occupied / inflated / free")
    from matplotlib.patches import Patch

    ax.legend(
        handles=[
            Patch(facecolor=(0.82, 0.18, 0.16), label="occupied"),
            Patch(facecolor=(0.95, 0.72, 0.28), label="inflated"),
            Patch(facecolor=(0.86, 0.90, 0.82), label="free"),
            Patch(facecolor=(0.18, 0.18, 0.18), label="unknown"),
        ],
        loc="upper right",
        fontsize=8,
    )
    ys, xs = np.where((data >= 0))
    if xs.size:
        m = 0.8
        ax.set_xlim(ox + xs.min() * res - m, ox + (xs.max() + 1) * res + m)
        ax.set_ylim(oy + ys.min() * res - m, oy + (ys.max() + 1) * res + m)
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    occ = int((data >= 100).sum())
    inf = int(((data > 0) & (data < 100)).sum())
    free = int((data == 0).sum())
    return occ, inf, free


def grab_topics(timeout=10.0):
    import rclpy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Float32

    qos_tl = QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )
    qos_be = QoSProfile(
        depth=8,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )
    qos_rel = QoSProfile(
        depth=8,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )

    class N(Node):
        def __init__(self):
            super().__init__("ppt_capture_stills")
            self.raw = self.desk = self.lm = self.pose = self.grid = None
            self.q = None
            self.n_pose = 0
            self.t0 = time.monotonic()
            self.create_subscription(PointCloud2, "/rslidar_points", self._raw, qos_be)
            self.create_subscription(PointCloud2, "/rslidar_points", self._raw, qos_rel)
            self.create_subscription(PointCloud2, "/lidar_odometry/deskewed_scan_points", self._desk, qos_tl)
            self.create_subscription(PointCloud2, "/lidar_odometry/deskewed_scan_points", self._desk, qos_be)
            self.create_subscription(PointCloud2, "/lidar_odometry/localmap_points", self._lm, qos_tl)
            self.create_subscription(Odometry, "/lidar_odometry/pose", self._pose, qos_tl)
            self.create_subscription(Float32, "/lidar_odometry/pose_quality", self._q, qos_tl)
            self.create_subscription(OccupancyGrid, "/nav_demo/obstacle_grid", self._g, qos_tl)

        def _raw(self, m):
            self.raw = m

        def _desk(self, m):
            self.desk = m

        def _lm(self, m):
            self.lm = m

        def _pose(self, m):
            self.pose = m
            self.n_pose += 1

        def _q(self, m):
            self.q = float(m.data)

        def _g(self, m):
            self.grid = m

    rclpy.init()
    n = N()
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        rclpy.spin_once(n, timeout_sec=0.15)
        if n.raw is not None and n.pose is not None and n.grid is not None:
            # wait a bit more for localmap/quality
            if time.monotonic() - t0 > 3.0:
                break
    dt = max(1e-3, time.monotonic() - n.t0)
    hz = n.n_pose / dt
    box = dict(raw=n.raw, desk=n.desk, lm=n.lm, pose=n.pose, grid=n.grid, q=n.q, pose_hz=hz, n_pose=n.n_pose)
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return box


def main() -> int:
    print("PPT still capture — listen only, no CAN", flush=True)
    box = grab_topics(12.0)
    if box["raw"] is None:
        print("FAIL: no /rslidar_points")
        return 2
    if box["pose"] is None:
        print("FAIL: no /lidar_odometry/pose")
        return 2

    raw = xyz_from_cloud(box["raw"])
    base = rslidar_to_base(raw)
    p = box["pose"].pose.pose.position
    q = box["pose"].pose.pose.orientation
    R = quat_to_rot(q.x, q.y, q.z, q.w)
    t = np.array([p.x, p.y, p.z], dtype=np.float64)
    yaw = yaw_from_q(q)
    xyz_map = (base @ R.T) + t.reshape(1, 3)

    n1 = save_lidar_3d(base, OUT / "lidar_3d_pointcloud.png")
    print(f"WROTE {OUT / 'lidar_3d_pointcloud.png'} plotted={n1} raw={raw.shape[0]} frame={box['raw'].header.frame_id}")

    n2 = save_localization(xyz_map, t, R, OUT / "mola_localization.png")
    print(
        f"WROTE {OUT / 'mola_localization.png'} plotted={n2} pose=({p.x:.3f},{p.y:.3f},{p.z:.3f}) "
        f"yaw={math.degrees(yaw):.1f} q={box['q']} hz={box['pose_hz']:.2f} "
        f"frame={box['pose'].header.frame_id}->{box['pose'].child_frame_id}"
    )

    lm_n = 0
    lm_ok = False
    if box["lm"] is not None:
        lm = xyz_from_cloud(box["lm"])
        lm_n = int(lm.shape[0])
        lm_ok, _ = save_localmap(lm, t, OUT / "mola_localmap.png")
        if lm_ok:
            print(f"WROTE {OUT / 'mola_localmap.png'} n={lm_n} frame={box['lm'].header.frame_id}")
        else:
            print(f"SKIP mola_localmap.png sparse n={lm_n} (not suitable as PPT main figure)")
    else:
        print("SKIP mola_localmap.png: no /lidar_odometry/localmap_points")

    if box["grid"] is None:
        print("SKIP obstacle_grid.png: no /nav_demo/obstacle_grid")
    else:
        occ, inf, free = save_grid(box["grid"], (p.x, p.y, yaw), OUT / "obstacle_grid.png")
        print(f"WROTE {OUT / 'obstacle_grid.png'} occ={occ} inf={inf} free={free}")

    meta = OUT / "capture_meta.txt"
    meta.write_text(
        f"pose_hz={box['pose_hz']:.3f}\n"
        f"pose_quality={box['q']}\n"
        f"pose_frame={box['pose'].header.frame_id}\n"
        f"child_frame={box['pose'].child_frame_id}\n"
        f"pose_xyz={p.x:.4f},{p.y:.4f},{p.z:.4f}\n"
        f"yaw_deg={math.degrees(yaw):.2f}\n"
        f"raw_points={raw.shape[0]}\n"
        f"localmap_points={lm_n}\n"
        f"localmap_suitable={lm_ok}\n"
    )
    print(f"WROTE {meta}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
