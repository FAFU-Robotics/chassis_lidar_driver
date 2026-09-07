#!/usr/bin/python3
"""Accumulate live Airy scans and render a PPT p3 still of the real lab.

Listen-only: no CAN, no MOLA/TF changes.
"""
from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("ROS_HOME", "/tmp/ros_ppt_p3")
os.makedirs(os.environ["ROS_HOME"] + "/log", exist_ok=True)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import Normalize
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT = Path("/home/fafu_robot/Desktop/chassis_lidar_drivers")
OUT_DIR = ROOT / "ppt_materials" / "generated"
A_DIR = ROOT / "ppt_materials" / "A_perception"
OUT_DIR.mkdir(parents=True, exist_ok=True)
A_DIR.mkdir(parents=True, exist_ok=True)


def xyz_from_cloud(msg):
    n = int(msg.width) * int(msg.height)
    if n <= 0:
        return np.zeros((0, 3), np.float64)
    off = {f.name: f.offset for f in msg.fields}
    dt = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": ["<f4", "<f4", "<f4"],
            "offsets": [off["x"], off["y"], off["z"]],
            "itemsize": int(msg.point_step),
        }
    )
    rec = np.frombuffer(msg.data, dtype=dt, count=n)
    xyz = np.column_stack((rec["x"], rec["y"], rec["z"]))
    return xyz[np.isfinite(xyz).all(axis=1)]


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


def rslidar_to_base(xyz_s):
    if xyz_s.size == 0:
        return xyz_s
    x_s, y_s, z_s = xyz_s[:, 0], xyz_s[:, 1], xyz_s[:, 2]
    return np.column_stack((z_s, x_s, y_s + 0.365))


def voxel_mean(xyz, vox=0.03):
    if xyz.shape[0] == 0:
        return xyz
    ijk = np.floor(xyz / vox).astype(np.int32)
    key = (
        ijk[:, 0].astype(np.int64) * 73856093
        ^ ijk[:, 1].astype(np.int64) * 19349663
        ^ ijk[:, 2].astype(np.int64) * 83492791
    )
    order = np.argsort(key)
    key_s = key[order]
    xyz_s = xyz[order]
    cuts = np.flatnonzero(np.diff(key_s)) + 1
    starts = np.r_[0, cuts]
    counts = np.diff(np.r_[starts, key_s.size]).astype(np.float32).reshape(-1, 1)
    return np.add.reduceat(xyz_s, starts) / counts


def merge_voxels(acc, incoming, vox):
    if incoming.shape[0] == 0:
        return acc
    incoming = voxel_mean(incoming.astype(np.float32, copy=False), vox)
    if acc.shape[0] == 0:
        return incoming
    return voxel_mean(np.vstack((acc, incoming)), vox)


def draw_chassis(ax, origin, R):
    # Bunker Mini 2.0 approx body, lidar at z=0.365 already in points.
    lx, ly, lz = 0.58, 0.50, 0.22
    corners = np.array(
        [
            [lx / 2, ly / 2, 0.0],
            [lx / 2, -ly / 2, 0.0],
            [-lx / 2, -ly / 2, 0.0],
            [-lx / 2, ly / 2, 0.0],
            [lx / 2, ly / 2, lz],
            [lx / 2, -ly / 2, lz],
            [-lx / 2, -ly / 2, lz],
            [-lx / 2, ly / 2, lz],
        ]
    )
    pts = (R @ corners.T).T + origin.reshape(1, 3)
    faces_i = (
        (0, 1, 2, 3),
        (4, 5, 6, 7),
        (0, 1, 5, 4),
        (2, 3, 7, 6),
        (0, 3, 7, 4),
        (1, 2, 6, 5),
    )
    verts = [[pts[i] for i in face] for face in faces_i]
    ax.add_collection3d(
        Poly3DCollection(
            verts,
            facecolors=(0.82, 0.82, 0.84, 0.92),
            edgecolors=(0.55, 0.55, 0.58, 0.9),
            linewidths=0.4,
        )
    )
    # lidar dome
    top = origin + R @ np.array([0.0, 0.0, 0.365])
    ax.scatter([top[0]], [top[1]], [top[2]], c="#dddddd", s=18, depthshade=False)
    fwd = origin + R @ np.array([1.15, 0.0, 0.06])
    ax.plot(
        [origin[0], fwd[0]],
        [origin[1], fwd[1]],
        [origin[2] + 0.06, fwd[2]],
        color="#ff4d3a",
        lw=2.0,
    )


def draw_grid(ax, xmin, xmax, ymin, ymax, step=1.0):
    xs = np.arange(math.floor(xmin), math.ceil(xmax) + 1e-6, step)
    ys = np.arange(math.floor(ymin), math.ceil(ymax) + 1e-6, step)
    for x in xs:
        ax.plot([x, x], [ymin, ymax], [0, 0], color=(0.42, 0.42, 0.45), lw=0.35, alpha=0.7)
    for y in ys:
        ax.plot([xmin, xmax], [y, y], [0, 0], color=(0.42, 0.42, 0.45), lw=0.35, alpha=0.7)


def render(xyz, origin, R, yaw, path, elev=64.0, azim_off=-90.0, path_xy=None):
    z = xyz[:, 2]
    keep = (z > -0.05) & (z < 3.15)
    xyz = xyz[keep]
    if xyz.shape[0] < 200:
        raise SystemExit(f"too few points after z filter: {xyz.shape[0]}")

    dxy = np.hypot(xyz[:, 0] - origin[0], xyz[:, 1] - origin[1])
    xyz = xyz[dxy < 11.0]
    if xyz.shape[0] > 180000:
        rng = np.random.default_rng(1)
        xyz = xyz[rng.choice(xyz.shape[0], 180000, replace=False)]

    fig = plt.figure(figsize=(13.5, 8.4), dpi=160, facecolor="black")
    ax = fig.add_subplot(111, projection="3d", facecolor="black")
    ax.set_axis_off()
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor((0, 0, 0, 0))
    ax.yaxis.pane.set_edgecolor((0, 0, 0, 0))
    ax.zaxis.pane.set_edgecolor((0, 0, 0, 0))
    ax.grid(False)

    xmin, xmax = np.percentile(xyz[:, 0], [1, 99])
    ymin, ymax = np.percentile(xyz[:, 1], [1, 99])
    pad = 0.8
    xmin, xmax = xmin - pad, xmax + pad
    ymin, ymax = ymin - pad, ymax + pad
    draw_grid(ax, xmin, xmax, ymin, ymax, step=1.0)

    c = xyz[:, 2]
    ax.scatter(
        xyz[:, 0],
        xyz[:, 1],
        xyz[:, 2],
        c=c,
        s=0.18,
        cmap="jet",
        linewidths=0,
        alpha=0.94,
        rasterized=True,
        norm=Normalize(vmin=0.0, vmax=float(min(2.6, np.percentile(c, 97)))),
    )
    if path_xy is not None and len(path_xy) >= 2:
        pp = np.asarray(path_xy, dtype=np.float64)
        ax.plot(pp[:, 0], pp[:, 1], np.full(pp.shape[0], 0.04), color="#ff4d3a", lw=2.4)
    draw_chassis(ax, origin, R)

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_zlim(0.0, 2.7)
    try:
        ax.set_box_aspect((xmax - xmin, ymax - ymin, 1.85))
    except Exception:
        pass

    azim = math.degrees(yaw) + azim_off
    ax.view_init(elev=elev, azim=azim)
    ax.set_proj_type("persp")
    fig.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=1.0)
    fig.savefig(path, facecolor="black", dpi=160)
    plt.close(fig)
    return xyz.shape[0]


def grab_accumulate(seconds=3.2, vox=0.03, stride=2):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from tf2_ros import Buffer, TransformListener

    qos_be = QoSProfile(
        depth=8,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )
    qos_tl = QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )

    class N(Node):
        def __init__(self):
            super().__init__("ppt_p3_lab_cloud")
            self.buf = Buffer()
            self.tl = TransformListener(self.buf, self)
            self.acc = np.zeros((0, 3), np.float32)
            self.pending = []
            self.n_raw = 0
            self.n_seen = 0
            self.n_pts_in = 0
            self.pose = None
            self.lm = None
            self.vox = float(vox)
            self.stride = max(1, int(stride))
            # One subscription only. Dual QoS doubled RAM last time.
            self.create_subscription(PointCloud2, "/rslidar_points", self._on_raw, qos_be)
            self.create_subscription(Odometry, "/lidar_odometry/pose", self._on_pose, qos_tl)
            self.create_subscription(
                PointCloud2, "/lidar_odometry/localmap_points", self._on_lm, qos_tl
            )

        def _on_lm(self, m):
            self.lm = m

        def _on_pose(self, m):
            self.pose = m

        def _tf_map_from_rslidar(self):
            try:
                return self.buf.lookup_transform("map", "rslidar", rclpy.time.Time())
            except Exception:
                return None

        def _on_raw(self, msg):
            self.n_seen += 1
            if (self.n_seen % self.stride) != 0:
                return
            xyz_s = xyz_from_cloud(msg)
            if xyz_s.shape[0] < 200:
                return
            rng = np.linalg.norm(xyz_s, axis=1)
            xyz_s = xyz_s[(rng > 0.50) & (rng < 12.0)]
            if xyz_s.shape[0] < 200:
                return
            tf = self._tf_map_from_rslidar()
            if tf is not None:
                t = tf.transform.translation
                q = tf.transform.rotation
                Rm = quat_to_rot(q.x, q.y, q.z, q.w)
                xyz = (Rm @ xyz_s.T).T + np.array([t.x, t.y, t.z])
            elif self.pose is not None:
                base = rslidar_to_base(xyz_s)
                p = self.pose.pose.pose.position
                q = self.pose.pose.pose.orientation
                Rm = quat_to_rot(q.x, q.y, q.z, q.w)
                t = np.array([p.x, p.y, p.z])
                xyz = (base @ Rm.T) + t.reshape(1, 3)
            else:
                return
            # drop robot body in map by transforming origin-relative in base if pose known
            if self.pose is not None:
                p = self.pose.pose.pose.position
                q = self.pose.pose.pose.orientation
                Rm = quat_to_rot(q.x, q.y, q.z, q.w)
                t = np.array([p.x, p.y, p.z])
                rel = (xyz - t.reshape(1, 3)) @ Rm
                body = (rel[:, 0] > -0.40) & (rel[:, 0] < 0.40) & (np.abs(rel[:, 1]) < 0.32) & (rel[:, 2] < 0.50)
                xyz = xyz[~body]
            xyz = xyz[(xyz[:, 2] > -0.2) & (xyz[:, 2] < 3.4)]
            if xyz.shape[0] < 100:
                return
            self.pending.append(xyz.astype(np.float32, copy=False))
            self.n_raw += 1
            self.n_pts_in += int(xyz.shape[0])
            if len(self.pending) >= 6:
                self._flush()

        def _flush(self):
            if not self.pending:
                return
            batch = np.vstack(self.pending)
            self.pending.clear()
            self.acc = merge_voxels(self.acc, batch, self.vox)

    rclpy.init()
    node = N()
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    pose = node.pose
    n_raw = node.n_raw
    n_pts_in = node.n_pts_in
    lm_msg = node.lm
    vox = node.vox
    node._flush()
    xyz = node.acc
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    lm_n = 0
    if lm_msg is not None:
        lm = xyz_from_cloud(lm_msg)
        lm = lm[(lm[:, 2] > -0.2) & (lm[:, 2] < 3.4)]
        lm_n = int(lm.shape[0])
        xyz = merge_voxels(xyz, lm, vox)
    if xyz.shape[0] == 0:
        raise SystemExit("no lidar frames accumulated")
    return xyz, pose, n_raw, n_pts_in, lm_n


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_of_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rotz(yaw: float, xyz: np.ndarray) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    return np.column_stack((c * x - s * y, s * x + c * y, z))


def grab_spin_accumulate(vox=0.03, stride=2, yaw_target_deg=200.0, timeout_s=40.0):
    """In-place yaw, stitch live clouds in robot-centric frame. Ignores smeared map XY."""
    sys.path.insert(0, str(ROOT / "nav_demo"))
    from official_mods import ensure_package
    from run_first_live import can_control_mode, halt_chassis

    ensure_package()
    from bunker_mini.teleop_tcp import DEFAULT_PORT, DEFAULT_TOKEN, TeleopTcpClient

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Float32

    mode = can_control_mode()
    print(f"SPIN control_mode={mode}", flush=True)
    if mode != 1:
        raise SystemExit(f"control_mode={mode}, not spinning")

    qos_be = QoSProfile(
        depth=4,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )
    qos_tl = QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )

    qos_wheel = QoSProfile(
        depth=20,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )

    class N(Node):
        def __init__(self):
            super().__init__("ppt_p3_spin_cloud")
            self.acc = np.zeros((0, 3), np.float32)
            self.pending = []
            self.n_raw = 0
            self.n_seen = 0
            self.n_pts_in = 0
            self.vox = float(vox)
            self.stride = max(1, int(stride))
            self.quality = None
            self.yaw0 = None
            self.wheel_yaw = None
            self.wheel_unwrapped = 0.0
            self._last_wheel = None
            self.t_wheel = 0.0
            self.cmd_unwrapped = 0.0
            self.w_active = 0.0
            self.t_last_int = None
            self.ready = False
            self.sign = 1.0
            self.mola_unwrapped = 0.0
            self._last_mola = None
            self.t_mola = 0.0
            self.yaw_lock = None
            self.create_subscription(PointCloud2, "/rslidar_points", self._on_raw, qos_be)
            self.create_subscription(Odometry, "/wheel_odom", self._on_wheel, qos_wheel)
            self.create_subscription(Odometry, "/lidar_odometry/pose", self._on_mola, qos_tl)
            self.create_subscription(Float32, "/lidar_odometry/pose_quality", self._on_q, qos_tl)

        def _on_q(self, m):
            self.quality = float(m.data)

        def _on_wheel(self, m):
            y = yaw_of_quat(m.pose.pose.orientation)
            now = time.monotonic()
            if self._last_wheel is None:
                self.wheel_unwrapped = 0.0
                self._last_wheel = y
                self.wheel_yaw = y
                self.t_wheel = now
                return
            dy = wrap(y - self._last_wheel)
            if abs(dy) > math.radians(8.0):
                return
            if dy * self.sign < -math.radians(0.25):
                return
            self.wheel_unwrapped += dy
            self._last_wheel = y
            self.wheel_yaw = y
            self.t_wheel = now

        def integrate_cmd(self, now: float) -> None:
            if self.t_last_int is None:
                self.t_last_int = now
                return
            dt = now - self.t_last_int
            self.t_last_int = now
            if dt <= 0.0 or dt > 0.25:
                return
            if abs(self.w_active) > 1e-6:
                self.cmd_unwrapped += self.w_active * dt

        def _on_mola(self, m):
            y = yaw_of_quat(m.pose.pose.orientation)
            self.t_mola = time.monotonic()
            if self._last_mola is None:
                self._last_mola = y
                return
            dy = wrap(y - self._last_mola)
            # In-place spin: ICP can jump backward. Keep stitch monotonic.
            if abs(dy) > math.radians(7.0):
                return
            if dy * self.sign < -math.radians(0.30):
                return
            self.mola_unwrapped += dy
            self._last_mola = y

        def yaw_now(self):
            # Stop-and-shoot: lock wheel yaw at rest. MOLA ICP is jumpy while spinning.
            if self._last_wheel is not None:
                return self.wheel_unwrapped
            return self.mola_unwrapped

        def _flush(self):
            if not self.pending:
                return
            batch = np.vstack(self.pending)
            self.pending.clear()
            self.acc = merge_voxels(self.acc, batch, self.vox)

        def _on_raw(self, msg):
            if not self.ready:
                return
            self.n_seen += 1
            if (self.n_seen % self.stride) != 0:
                return
            xyz_s = xyz_from_cloud(msg)
            if xyz_s.shape[0] < 200:
                return
            rng = np.linalg.norm(xyz_s, axis=1)
            xyz_s = xyz_s[(rng > 0.50) & (rng < 12.0)]
            if xyz_s.shape[0] < 200:
                return
            base = rslidar_to_base(xyz_s)
            body = (
                (base[:, 0] > -0.40)
                & (base[:, 0] < 0.40)
                & (np.abs(base[:, 1]) < 0.32)
                & (base[:, 2] < 0.50)
            )
            base = base[~body]
            base = base[(base[:, 2] > -0.05) & (base[:, 2] < 3.2)]
            if base.shape[0] < 100:
                return
            xyz = rotz(
                self.yaw_lock if self.yaw_lock is not None else self.yaw_now(),
                base.astype(np.float32, copy=False),
            )
            self.pending.append(xyz.astype(np.float32, copy=False))
            self.n_raw += 1
            self.n_pts_in += int(xyz.shape[0])
            if len(self.pending) >= 5:
                self._flush()

    cli = TeleopTcpClient("127.0.0.1", DEFAULT_PORT, DEFAULT_TOKEN)
    cli.hello()
    cli.stick(0.0, 0.0)

    rclpy.init()
    node = N()
    t_wait = time.monotonic()
    while time.monotonic() - t_wait < 4.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node._last_wheel is not None:
            break
    if node._last_wheel is None:
        halt_chassis(cli, "no wheel_odom")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit("no /wheel_odom for spin stitch")

    node.wheel_unwrapped = 0.0
    node.cmd_unwrapped = 0.0
    node.mola_unwrapped = 0.0
    node.sign = 1.0
    node.ready = False
    print(
        f"SPIN stitch=wheel_stop_shot start_wheel={math.degrees(node.wheel_yaw or 0.0):.1f} q={node.quality}",
        flush=True,
    )

    w_cmd = 0.12
    pulse_s = 0.28
    period_s = 0.24
    target = math.radians(float(yaw_target_deg))
    n_pulse = 0
    reason = "timeout"
    n_shot = 0
    try:
        t_run = time.monotonic()
        while time.monotonic() - t_run < float(timeout_s):
            # Shoot only when stopped, with yaw frozen — no intra-burst smear.
            node.w_active = 0.0
            try:
                cli.stick(0.0, 0.0)
            except Exception:
                pass
            t_settle = time.monotonic() + 0.85
            while time.monotonic() < t_settle:
                rclpy.spin_once(node, timeout_sec=0.02)
            node.yaw_lock = float(node.yaw_now())
            node.ready = True
            t_shot = time.monotonic() + 1.05
            while time.monotonic() < t_shot:
                rclpy.spin_once(node, timeout_sec=0.02)
            node.ready = False
            node._flush()
            n_shot += 1
            dyaw = abs(node.yaw_now())
            print(
                f"SHOT n={n_shot} dyaw={math.degrees(dyaw):.1f} "
                f"wheel={math.degrees(node.wheel_unwrapped):.1f} "
                f"q={node.quality} voxels={node.acc.shape[0]}",
                flush=True,
            )
            if dyaw >= target or n_shot >= 9:
                reason = f"yaw_done {math.degrees(dyaw):.1f}deg shots={n_shot}"
                break
            node.yaw_lock = None
            last_pulse = 0.0
            t_turn = time.monotonic() + 2.2
            while time.monotonic() < t_turn:
                rclpy.spin_once(node, timeout_sec=0.02)
                now = time.monotonic()
                if now - last_pulse >= period_s:
                    cli.cmd(
                        {
                            "action": "move",
                            "v": 0.0,
                            "w": float(w_cmd),
                            "duration": pulse_s,
                            "bypassGuard": True,
                            "ts": int(time.time() * 1000),
                        },
                        timeout=0.8,
                    )
                    n_pulse += 1
                    last_pulse = now
            try:
                cli.stick(0.0, 0.0)
            except Exception:
                pass
            time.sleep(0.15)
        else:
            reason = "timeout"
    except Exception as exc:
        reason = f"exception {exc}"
    finally:
        halt_chassis(cli, reason)
        time.sleep(0.25)
        try:
            cli.close()
        except Exception:
            pass

    node._flush()
    xyz = node.acc
    n_raw = node.n_raw
    n_pts_in = node.n_pts_in
    print(
        f"SPIN end reason={reason!r} pulses={n_pulse} frames={n_raw} pts_in={n_pts_in} voxels={xyz.shape[0]}",
        flush=True,
    )
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if xyz.shape[0] == 0:
        raise SystemExit("spin captured no points")
    return xyz, n_raw, n_pts_in, reason


def main() -> int:
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=3.5)
    ap.add_argument("--vox", type=float, default=0.03)
    ap.add_argument("--stride", type=int, default=2, help="keep 1 of N lidar frames")
    ap.add_argument("--spin", action="store_true", help="in-place yaw stitch in robot frame")
    ap.add_argument(
        "--replace-slide",
        action="store_true",
        help="copy timestamped PNG onto p2/p3/A stills (never touches _approved)",
    )
    args = ap.parse_args()
    vox = max(0.02, float(args.vox))
    if args.spin:
        print("P2 in-place spin — v=0 only, no forward", flush=True)
        xyz, n_raw, n_pts_in, reason = grab_spin_accumulate(
            vox=vox, stride=1, yaw_target_deg=80.0, timeout_s=42.0
        )
        print(f"kept_frames={n_raw} pts_in={n_pts_in} voxels={xyz.shape[0]} {reason}", flush=True)
        origin = np.zeros(3)
        R = np.eye(3)
        yaw = 0.0
    else:
        print("P3 lab cloud capture — listen only, no CAN", flush=True)
        xyz, pose, n_raw, n_pts_in, lm_n = grab_accumulate(
            max(1.5, float(args.seconds)), vox=vox, stride=int(args.stride)
        )
        print(
            f"kept_frames={n_raw} pts_in={n_pts_in} localmap={lm_n} voxels={xyz.shape[0]}",
            flush=True,
        )
        if pose is None:
            origin = np.zeros(3)
            R = np.eye(3)
            yaw = 0.0
        else:
            p = pose.pose.pose.position
            q = pose.pose.pose.orientation
            origin = np.array([p.x, p.y, p.z], dtype=np.float64)
            R = quat_to_rot(q.x, q.y, q.z, q.w)
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    z = xyz[:, 2]
    print(
        f"z_bands ground={int((z<0.25).sum())} mid={int(((z>=0.25)&(z<1.2)).sum())} "
        f"wall={int((z>=1.2).sum())} bbox_xy=({xyz[:,0].min():.1f},{xyz[:,1].min():.1f})-"
        f"({xyz[:,0].max():.1f},{xyz[:,1].max():.1f})",
        flush=True,
    )
    np.save(OUT_DIR / "p2_lab_cloud.npy", xyz)

    # Never overwrite the user-locked still.
    stamp = time.strftime("%H%M%S")
    out = OUT_DIR / f"p2_lab_perception_{stamp}.png"
    n = render(xyz, origin, R, yaw, out, elev=64.0, azim_off=-90.0)
    iso = OUT_DIR / f"p2_lab_perception_iso_{stamp}.png"
    render(xyz, origin, R, yaw, iso, elev=52.0, azim_off=-125.0)
    print(f"WROTE {out} plotted={n}")
    print(f"WROTE {iso}")
    if args.replace_slide:
        p2 = OUT_DIR / "p2_lab_perception.png"
        p3 = OUT_DIR / "p3_lab_perception.png"
        dest_a = A_DIR / "D_cloud_still.png"
        blob = out.read_bytes()
        p2.write_bytes(blob)
        p3.write_bytes(blob)
        dest_a.write_bytes(blob)
        print(f"WROTE {p2}")
        print(f"WROTE {p3}")
        print(f"WROTE {dest_a}")
    else:
        print("slide files not replaced (pass --replace-slide after inspecting)")
    print("approved still untouched: ppt_materials/generated/p3_lab_perception_approved.png")
    print(
        f"pose=({origin[0]:.3f},{origin[1]:.3f},{origin[2]:.3f}) yaw_deg={math.degrees(yaw):.1f}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
