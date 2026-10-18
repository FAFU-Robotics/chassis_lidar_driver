#!/usr/bin/python3
"""Slow U-track for PPT slide 02: record /lidar_odometry/pose only.

No point-cloud stitch. ESTOP only at the end. Uses existing teleop.
"""
from __future__ import annotations

import csv
import math
import os
import sys
import time
from pathlib import Path

ROOT = Path("/home/fafu_robot/Desktop/chassis_lidar_drivers")
sys.path.insert(0, str(ROOT / "ppt_materials" / "scripts"))
sys.path.insert(0, str(ROOT / "nav_demo"))

os.environ.setdefault("ROS_HOME", "/tmp/ros_ppt_u_pose")
os.makedirs(os.environ["ROS_HOME"] + "/log", exist_ok=True)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from capture_p3_lab_cloud import rslidar_to_base, wrap, xyz_from_cloud, yaw_of_quat

OUT = ROOT / "ppt_materials" / "generated"
B_DIR = ROOT / "ppt_materials" / "B_localization"
OUT.mkdir(parents=True, exist_ok=True)
B_DIR.mkdir(parents=True, exist_ok=True)

NEAR_ABORT_M = 0.32
FRONT_TURN_M = 0.48
MIN_FWD_M = 2.2
LANE2_M = 2.4
V_FWD = 0.12
W_TURN = 0.12
V_TURN = 0.08


def plot_traj(rows, path):
    xs = [r["mx"] for r in rows]
    ys = [r["my"] for r in rows]
    fig, ax = plt.subplots(figsize=(9.2, 6.4), dpi=140, facecolor="#0b0f14")
    ax.set_facecolor("#0b0f14")
    ax.plot(xs, ys, color="#3dff7a", lw=2.4, solid_capstyle="round", label="/lidar_odometry/pose")
    ax.scatter([xs[0]], [ys[0]], c="#4da3ff", s=70, zorder=5, label="start")
    ax.scatter([xs[-1]], [ys[-1]], c="#ff5c5c", s=70, zorder=5, label="end")
    ax.set_xlabel("map X (m)", color="#d0d4da")
    ax.set_ylabel("map Y (m)", color="#d0d4da")
    ax.tick_params(colors="#9aa3ad")
    for spine in ax.spines.values():
        spine.set_color("#3a414a")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="#2a3138", lw=0.5)
    ax.legend(facecolor="#151b22", edgecolor="#3a414a", labelcolor="#d0d4da")
    ax.set_title("MOLA pose trajectory (real /lidar_odometry/pose)", color="#e8edf2")
    fig.tight_layout()
    fig.savefig(path, facecolor=fig.get_facecolor())
    plt.close(fig)


def run() -> int:
    from official_mods import ensure_package
    from run_first_live import can_control_mode, halt_chassis

    ensure_package()
    from bunker_mini.teleop_tcp import DEFAULT_PORT, DEFAULT_TOKEN, TeleopTcpClient

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2

    mode = can_control_mode()
    print(f"U-POSE control_mode={mode}", flush=True)
    if mode != 1:
        print("FAIL control_mode!=1")
        return 8

    qos_be = QoSProfile(
        depth=4,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )
    qos_rel = QoSProfile(
        depth=20,
        reliability=ReliabilityPolicy.RELIABLE,
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
            super().__init__("ppt_u_pose")
            self.x = 0.0
            self.y = 0.0
            self.yaw = 0.0
            self.yaw_unwrapped = 0.0
            self._last_yaw = None
            self.x0 = None
            self.y0 = None
            self.yaw0 = None
            self.front = 99.0
            self.have_lidar = False
            self.odom_dist = 0.0
            self._last_xy = None
            self.mola = []
            self.quality = None
            self.create_subscription(PointCloud2, "/rslidar_points", self._on_raw, qos_be)
            self.create_subscription(Odometry, "/wheel_odom", self._on_wheel, qos_rel)
            self.create_subscription(Odometry, "/lidar_odometry/pose", self._on_mola, qos_tl)

        def _on_mola(self, m):
            p = m.pose.pose.position
            y = yaw_of_quat(m.pose.pose.orientation)
            self.mola.append(
                {
                    "t": time.monotonic(),
                    "mx": float(p.x),
                    "my": float(p.y),
                    "mz": float(p.z),
                    "yaw": y,
                }
            )

        def _on_wheel(self, m):
            p = m.pose.pose.position
            y = yaw_of_quat(m.pose.pose.orientation)
            if self._last_yaw is None:
                self.x0, self.y0, self.yaw0 = float(p.x), float(p.y), y
                self._last_yaw = y
                self.yaw_unwrapped = 0.0
                self.x = self.y = self.yaw = 0.0
                return
            dy = wrap(y - self._last_yaw)
            if abs(dy) <= math.radians(25.0):
                self.yaw_unwrapped += dy
                self._last_yaw = y
            dx = float(p.x) - self.x0
            dyw = float(p.y) - self.y0
            c, s = math.cos(-self.yaw0), math.sin(-self.yaw0)
            self.x = c * dx - s * dyw
            self.y = s * dx + c * dyw
            self.yaw = self.yaw_unwrapped
            if self._last_xy is None:
                self._last_xy = (self.x, self.y)
            else:
                self.odom_dist += math.hypot(self.x - self._last_xy[0], self.y - self._last_xy[1])
                self._last_xy = (self.x, self.y)

        def _on_raw(self, msg):
            xyz_s = xyz_from_cloud(msg)
            if xyz_s.shape[0] < 200:
                return
            rng = np.linalg.norm(xyz_s, axis=1)
            xyz_s = xyz_s[(rng > 0.45) & (rng < 11.0)]
            if xyz_s.shape[0] < 200:
                return
            self.have_lidar = True
            base = rslidar_to_base(xyz_s)
            body = (
                (base[:, 0] > -0.40)
                & (base[:, 0] < 0.40)
                & (np.abs(base[:, 1]) < 0.32)
                & (base[:, 2] < 0.50)
            )
            base = base[~body]
            fwd = (
                (base[:, 0] > 0.40)
                & (np.abs(base[:, 1]) < 0.16)
                & (base[:, 2] > 0.12)
                & (base[:, 2] < 1.05)
            )
            if fwd.any():
                self.front = float(np.min(np.hypot(base[fwd, 0], base[fwd, 1])))
            else:
                self.front = 99.0

    cli = TeleopTcpClient("127.0.0.1", DEFAULT_PORT, DEFAULT_TOKEN)
    cli.hello()
    cli.stick(0.0, 0.0)

    rclpy.init()
    node = N()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 6.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.x0 is not None and node.have_lidar:
            break
    if node.x0 is None or not node.have_lidar:
        halt_chassis(cli, "no wheel_odom or lidar")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 2

    print(
        f"START front={node.front:.2f} x={node.x:.2f} y={node.y:.2f} mola_n={len(node.mola)}",
        flush=True,
    )

    def stick0():
        try:
            cli.stick(0.0, 0.0)
        except Exception:
            pass

    def pulse(v, w, duration, period=0.24, check_front=True):
        last = 0.0
        t_end = time.monotonic() + duration
        while time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.02)
            if check_front and v > 0.0 and node.front < NEAR_ABORT_M:
                stick0()
                return "near"
            now = time.monotonic()
            if now - last >= period:
                cli.cmd(
                    {
                        "action": "move",
                        "v": float(v),
                        "w": float(w),
                        "duration": 0.28,
                        "bypassGuard": True,
                        "ts": int(time.time() * 1000),
                    },
                    timeout=0.8,
                )
                last = now
        stick0()
        return "ok"

    reason = "ok"
    try:
        n_rev = 0
        while node.front < 0.90 and n_rev < 8:
            print(f"REVERSE n={n_rev} front={node.front:.2f}", flush=True)
            pulse(-0.11, 0.0, 1.20, check_front=False)
            n_rev += 1
            t_wait = time.monotonic() + 0.40
            while time.monotonic() < t_wait:
                rclpy.spin_once(node, timeout_sec=0.03)
        print(f"AFTER_REV front={node.front:.2f} dist={node.odom_dist:.2f}", flush=True)

        dist_at_start = node.odom_dist
        while (time.monotonic() - t0) < 140.0:
            gone = node.odom_dist - dist_at_start
            if node.front < FRONT_TURN_M and gone > 0.8:
                print(f"TURN trigger dead_ahead={node.front:.2f} gone={gone:.2f}", flush=True)
                break
            if gone > MIN_FWD_M:
                print(f"TURN trigger dist={gone:.2f}", flush=True)
                break
            st = pulse(V_FWD, 0.0, 2.70)
            if st == "near":
                reason = "near_fwd"
                break
            print(
                f"FWD gone={gone:.2f} front={node.front:.2f} yaw={math.degrees(node.yaw):.1f}",
                flush=True,
            )

        while abs(node.yaw) < math.radians(165.0) and reason == "ok":
            if (time.monotonic() - t0) > 140.0:
                reason = "timeout"
                break
            v = V_TURN if node.front > 0.40 else 0.05
            st = pulse(v, W_TURN, 2.20)
            if st == "near":
                st = pulse(0.05, W_TURN, 1.60)
                if st == "near":
                    reason = "near_turn"
                    break
            print(
                f"TURN yaw={math.degrees(node.yaw):.1f} front={node.front:.2f} dist={node.odom_dist:.2f}",
                flush=True,
            )

        dist0 = node.odom_dist
        while reason == "ok":
            if (time.monotonic() - t0) > 145.0:
                reason = "timeout"
                break
            extra = node.odom_dist - dist0
            if extra > LANE2_M or node.front < 0.42:
                break
            st = pulse(V_FWD, 0.0, 2.70)
            if st == "near":
                reason = "near_lane2"
                break
            print(
                f"LANE2 extra={extra:.2f} front={node.front:.2f} yaw={math.degrees(node.yaw):.1f}",
                flush=True,
            )
    except Exception as exc:
        reason = f"exception {exc}"
    finally:
        halt_chassis(cli, reason)
        time.sleep(0.3)
        t_end = time.monotonic() + 0.8
        while time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.03)
        try:
            cli.close()
        except Exception:
            pass

    rows = list(node.mola)
    odom_dist = node.odom_dist
    yaw_deg = math.degrees(node.yaw)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    print(
        f"END reason={reason!r} odom_dist={odom_dist:.2f}m wheel_yaw={yaw_deg:.1f} "
        f"mola_samples={len(rows)}",
        flush=True,
    )
    if len(rows) < 8:
        print("FAIL too few MOLA poses")
        return 3

    stamp = time.strftime("%H%M%S")
    csv_path = OUT / f"mola_pose_traj_{stamp}.csv"
    with csv_path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["t", "mx", "my", "mz", "yaw"])
        w.writeheader()
        w.writerows(rows)
    png = OUT / f"mola_pose_traj_{stamp}.png"
    plot_traj(rows, png)
    slide = OUT / "mola_pose_traj.png"
    slide.write_bytes(png.read_bytes())
    dest = B_DIR / "mola_pose_traj.png"
    dest.write_bytes(png.read_bytes())
    path_len = 0.0
    for a, b in zip(rows, rows[1:]):
        path_len += math.hypot(b["mx"] - a["mx"], b["my"] - a["my"])
    print(f"WROTE {csv_path}")
    print(f"WROTE {png}")
    print(f"WROTE {slide}")
    print(f"WROTE {dest}")
    print(f"mola_path_len={path_len:.2f}m disp={math.hypot(rows[-1]['mx']-rows[0]['mx'], rows[-1]['my']-rows[0]['my']):.2f}m")
    return 0


if __name__ == "__main__":
    sys.exit(run())
