#!/usr/bin/python3
"""Render live /lidar_odometry/localmap_points as a PPT 3D map. No CAN."""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

os.environ.setdefault("ROS_HOME", "/tmp/ros_ppt_localmap_render")
os.makedirs(os.environ["ROS_HOME"] + "/log", exist_ok=True)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

OUT = Path("/home/fafu_robot/Desktop/chassis_lidar_drivers/ppt_materials/generated/mola_localmap.png")


def xyz_from_cloud(msg):
    n = int(msg.width) * int(msg.height)
    if n <= 0:
        return np.zeros((0, 3), np.float32)
    off = {f.name: f.offset for f in msg.fields}
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
    return xyz[np.isfinite(xyz).all(axis=1)].astype(np.float64)


def grab():
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2

    qos_tl = QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )
    rclpy.init()
    node = Node("ppt_localmap_render")
    box = {"lm": None, "pose": None}

    def on_lm(m):
        box["lm"] = m

    def on_pose(m):
        box["pose"] = m

    node.create_subscription(PointCloud2, "/lidar_odometry/localmap_points", on_lm, qos_tl)
    node.create_subscription(Odometry, "/lidar_odometry/pose", on_pose, qos_tl)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 8.0:
        rclpy.spin_once(node, timeout_sec=0.2)
        if box["lm"] is not None:
            break
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return box["lm"], box["pose"]


def voxel_centers(xyz, vox=0.12):
    if xyz.shape[0] == 0:
        return xyz
    ijk = np.floor(xyz / vox).astype(np.int32)
    # unique voxels
    keys = ijk[:, 0].astype(np.int64) * 1000003 + ijk[:, 1].astype(np.int64) * 10007 + ijk[:, 2].astype(np.int64)
    _, idx = np.unique(keys, return_index=True)
    ijk = ijk[idx]
    # thin ground: keep at most 35% of lowest-z voxels
    z = ijk[:, 2]
    zcut = np.percentile(z, 18)
    ground = z <= zcut
    struct = ~ground
    g_idx = np.where(ground)[0]
    if g_idx.size > 400:
        rng = np.random.default_rng(1)
        keep_g = rng.choice(g_idx, size=400, replace=False)
        sel = np.concatenate([np.where(struct)[0], keep_g])
        ijk = ijk[sel]
    centers = (ijk.astype(np.float64) + 0.5) * vox
    return centers, vox


def cube_faces(c, s):
    h = s * 0.5
    x, y, z = c
    p = np.array(
        [
            [x - h, y - h, z - h],
            [x + h, y - h, z - h],
            [x + h, y + h, z - h],
            [x - h, y + h, z - h],
            [x - h, y - h, z + h],
            [x + h, y - h, z + h],
            [x + h, y + h, z + h],
            [x - h, y + h, z + h],
        ]
    )
    faces = [
        [p[0], p[1], p[2], p[3]],
        [p[4], p[5], p[6], p[7]],
        [p[0], p[1], p[5], p[4]],
        [p[2], p[3], p[7], p[6]],
        [p[1], p[2], p[6], p[5]],
        [p[0], p[3], p[7], p[4]],
    ]
    return faces


def render(xyz, pose_xyz, path: Path) -> int:
    z0, z1 = np.percentile(xyz[:, 2], [5, 95])
    keep = (xyz[:, 2] >= z0 - 0.15) & (xyz[:, 2] <= min(z1 + 0.2, z0 + 3.2))
    xyz = xyz[keep]
    centers, vox = voxel_centers(xyz, vox=0.14 if xyz.shape[0] > 8000 else 0.10)
    if centers.shape[0] > 6000:
        rng = np.random.default_rng(2)
        centers = centers[rng.choice(centers.shape[0], size=6000, replace=False)]
    fig = plt.figure(figsize=(12.8, 8.2), dpi=200, facecolor="#0b0d10")
    ax = fig.add_subplot(111, projection="3d", facecolor="#0b0d10")
    ax.set_axis_off()
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.grid(False)
    zmin, zmax = float(centers[:, 2].min()), float(centers[:, 2].max())
    span = max(zmax - zmin, 1e-3)
    faces_all = []
    colors = []
    for c in centers:
        t = (c[2] - zmin) / span
        # muted stone/wall tones, not rainbow scatter
        col = (0.45 + 0.40 * t, 0.48 + 0.22 * t, 0.42 + 0.10 * (1.0 - t), 0.92)
        for f in cube_faces(c, vox * 0.92):
            faces_all.append(f)
            colors.append(col)
    mesh = Poly3DCollection(faces_all, facecolors=colors, edgecolors=(0.12, 0.12, 0.12, 0.18), linewidths=0.15)
    ax.add_collection3d(mesh)
    if pose_xyz is not None:
        ax.scatter([pose_xyz[0]], [pose_xyz[1]], [pose_xyz[2] + 0.05], c="#4fc3f7", s=36, depthshade=False)
    ax.view_init(elev=42, azim=-62)
    pad = 0.35
    ax.set_xlim(centers[:, 0].min() - pad, centers[:, 0].max() + pad)
    ax.set_ylim(centers[:, 1].min() - pad, centers[:, 1].max() + pad)
    ax.set_zlim(centers[:, 2].min() - 0.15, centers[:, 2].max() + 0.25)
    try:
        ax.set_box_aspect(
            (
                max(centers[:, 0].ptp(), 0.5),
                max(centers[:, 1].ptp(), 0.5),
                max(centers[:, 2].ptp() * 0.7, 0.4),
            )
        )
    except Exception:
        pass
    fig.tight_layout(pad=0.15)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, facecolor="#0b0d10", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)
    return int(centers.shape[0])


def main() -> int:
    print("render MOLA localmap — listen only", flush=True)
    msg, pose = grab()
    if msg is None:
        print("FAIL: no localmap")
        return 2
    xyz = xyz_from_cloud(msg)
    print(f"localmap n={xyz.shape[0]} frame={msg.header.frame_id}")
    pose_xyz = None
    if pose is not None:
        p = pose.pose.pose.position
        pose_xyz = np.array([p.x, p.y, p.z], dtype=np.float64)
        print(f"pose ({p.x:.3f},{p.y:.3f},{p.z:.3f})")
    n = render(xyz, pose_xyz, OUT)
    print(f"WROTE {OUT} voxels={n}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
