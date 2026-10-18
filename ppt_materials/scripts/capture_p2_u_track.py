#!/usr/bin/python3
"""Follow the lab U-track (red line): forward, left 180, forward.

Clean stills: NEVER accumulate while moving. Stop, wait until pose is
stable, freeze (x,y,yaw), then shoot. Wheel odom relative to start.
v<=0.06. ESTOP only at the end.
"""
from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

ROOT = Path("/home/fafu_robot/Desktop/chassis_lidar_drivers")
sys.path.insert(0, str(ROOT / "ppt_materials" / "scripts"))
sys.path.insert(0, str(ROOT / "nav_demo"))

os.environ.setdefault("ROS_HOME", "/tmp/ros_ppt_p2u")
os.makedirs(os.environ["ROS_HOME"] + "/log", exist_ok=True)

import numpy as np

from capture_p3_lab_cloud import (
    OUT_DIR,
    merge_voxels,
    quat_to_rot,
    render,
    rotz,
    rslidar_to_base,
    voxel_mean,
    wrap,
    xyz_from_cloud,
    yaw_of_quat,
)


NEAR_ABORT_M = 0.32
FRONT_TURN_M = 0.48
MIN_FWD_M = 2.2
LANE2_M = 2.4
V_FWD = 0.12
W_TURN = 0.12
V_TURN = 0.08


def pose_R(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def voxel_keys(xyz, vox):
    ijk = np.floor(xyz / vox).astype(np.int32)
    return set(map(tuple, ijk.tolist()))


def apply_xyyaw(xyz, dx, dy, yaw):
    out = rotz(yaw, xyz)
    out = out.copy()
    out[:, 0] += dx
    out[:, 1] += dy
    return out


def align_shot(src, dst, vox=0.10):
    """PPT-only: snap a stopped shot onto the accumulated cloud. No MOLA change."""
    if src.shape[0] < 80 or dst.shape[0] < 80:
        return src
    sw = src[(src[:, 2] > 0.30) & (src[:, 2] < 2.50)]
    dw = dst[(dst[:, 2] > 0.30) & (dst[:, 2] < 2.50)]
    if sw.shape[0] < 60 or dw.shape[0] < 60:
        return src
    rng = np.random.default_rng(2)
    if sw.shape[0] > 2200:
        sw = sw[rng.choice(sw.shape[0], 2200, replace=False)]
    if dw.shape[0] > 7000:
        dw = dw[rng.choice(dw.shape[0], 7000, replace=False)]

    def pack(xyz):
        ijk = np.floor(xyz / vox).astype(np.int64)
        return ijk[:, 0] * 73856093 ^ ijk[:, 1] * 19349663 ^ ijk[:, 2] * 83492791

    dkeys = np.unique(pack(dw))

    def hits(xyz):
        return int(np.isin(pack(xyz), dkeys).sum())

    best = (0.0, 0.0, 0.0, -1)
    for yaw in np.linspace(-math.radians(3.5), math.radians(3.5), 8):
        rot = rotz(float(yaw), sw)
        for dx in np.linspace(-0.10, 0.10, 6):
            for dy in np.linspace(-0.10, 0.10, 6):
                tmp = rot.copy()
                tmp[:, 0] += dx
                tmp[:, 1] += dy
                h = hits(tmp)
                if h > best[3]:
                    best = (float(dx), float(dy), float(yaw), h)
    dx, dy, yaw, hit = best
    for yaw2 in (yaw - math.radians(0.7), yaw, yaw + math.radians(0.7)):
        rot = rotz(float(yaw2), sw)
        for dx2 in (dx - 0.03, dx, dx + 0.03):
            for dy2 in (dy - 0.03, dy, dy + 0.03):
                tmp = rot.copy()
                tmp[:, 0] += dx2
                tmp[:, 1] += dy2
                h = hits(tmp)
                if h > best[3]:
                    best = (float(dx2), float(dy2), float(yaw2), h)
    dx, dy, yaw, hit = best
    print(f"  align dx={dx:.3f} dy={dy:.3f} dyaw={math.degrees(yaw):.2f} hit={hit}", flush=True)
    return apply_xyyaw(src, dx, dy, yaw)


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
    print(f"U-TRACK control_mode={mode}", flush=True)
    if mode != 1:
        print("FAIL control_mode!=1")
        return 8

    qos_be = QoSProfile(
        depth=4,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
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
            super().__init__("ppt_p2_u_track")
            self.acc = np.zeros((0, 3), np.float32)
            self.pending = []
            self.vox = 0.035
            self.ready = False
            self.lock_x = 0.0
            self.lock_y = 0.0
            self.lock_yaw = 0.0
            self.x = 0.0
            self.y = 0.0
            self.yaw = 0.0
            self.yaw_unwrapped = 0.0
            self._last_yaw = None
            self.x0 = None
            self.y0 = None
            self.yaw0 = None
            self.front = 99.0
            self.odom_dist = 0.0
            self._last_xy = None
            self.n_raw = 0
            self.path = []
            self.shot_buf = []
            self.create_subscription(PointCloud2, "/rslidar_points", self._on_raw, qos_be)
            self.create_subscription(Odometry, "/wheel_odom", self._on_wheel, qos_wheel)

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
            if abs(dy) <= math.radians(8.0) and dy >= -math.radians(0.35):
                self.yaw_unwrapped += dy
                self._last_yaw = y
            # start-frame pose (heading0 = +X)
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

        def _flush(self):
            if not self.pending:
                return
            batch = np.vstack(self.pending)
            self.pending.clear()
            self.acc = merge_voxels(self.acc, batch, self.vox)

        def _on_raw(self, msg):
            xyz_s = xyz_from_cloud(msg)
            if xyz_s.shape[0] < 200:
                return
            rng = np.linalg.norm(xyz_s, axis=1)
            xyz_s = xyz_s[(rng > 0.45) & (rng < 11.0)]
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
            if base.shape[0] < 80:
                return
            # Dead-ahead only. Inner-island plants sit to the left, not in this cone.
            fwd = (
                (base[:, 0] > 0.40)
                & (np.abs(base[:, 1]) < 0.16)
                & (base[:, 2] > 0.12)
                & (base[:, 2] < 1.05)
            )
            if fwd.any():
                self.front = float(np.min(np.hypot(base[fwd, 0], base[fwd, 1])))
            if not self.ready:
                return
            xyz = rotz(self.lock_yaw, base.astype(np.float32, copy=False))
            xyz[:, 0] += self.lock_x
            xyz[:, 1] += self.lock_y
            self.shot_buf.append(xyz.astype(np.float32, copy=False))
            self.n_raw += 1

    cli = TeleopTcpClient("127.0.0.1", DEFAULT_PORT, DEFAULT_TOKEN)
    cli.hello()
    cli.stick(0.0, 0.0)

    rclpy.init()
    node = N()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.x0 is not None and node.front < 90.0:
            break
    if node.x0 is None:
        halt_chassis(cli, "no wheel_odom")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 2

    print(
        f"START front={node.front:.2f} x={node.x:.2f} y={node.y:.2f} yaw=0",
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

    def settle_and_shoot(tag: str) -> str:
        node.ready = False
        stick0()
        t_end = time.monotonic() + 1.15
        ys, xs = [], []
        while time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.03)
            ys.append(node.yaw)
            xs.append((node.x, node.y))
        if len(ys) >= 6:
            if max(ys[-8:]) - min(ys[-8:]) > math.radians(1.4):
                t_end = time.monotonic() + 0.55
                while time.monotonic() < t_end:
                    rclpy.spin_once(node, timeout_sec=0.03)
                    ys.append(node.yaw)
        node.lock_x = float(node.x)
        node.lock_y = float(node.y)
        node.lock_yaw = float(node.yaw)
        node.path.append((node.lock_x, node.lock_y))
        node.shot_buf = []
        node.ready = True
        t_end = time.monotonic() + 1.15
        while time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.03)
        node.ready = False
        if node.shot_buf:
            shot = voxel_mean(np.vstack(node.shot_buf), node.vox)
            node.shot_buf = []
            node.acc = merge_voxels(node.acc, shot, node.vox)
        print(
            f"{tag} xy=({node.lock_x:.2f},{node.lock_y:.2f}) "
            f"yaw={math.degrees(node.lock_yaw):.1f} front={node.front:.2f} "
            f"voxels={node.acc.shape[0]}",
            flush=True,
        )
        return "ok"

    reason = "ok"
    n_shot = 0
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

        settle_and_shoot("SHOT start")
        n_shot += 1
        dist_at_start = node.odom_dist

        # Phase A: first lane — keep going until ~2.2 m unless something is dead ahead.
        while n_shot < 10 and (time.monotonic() - t0) < 140.0:
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
            settle_and_shoot(f"SHOT fwd{n_shot}")
            n_shot += 1

        # Phase B: left U with translation (do not spin in place)
        while abs(node.yaw) < math.radians(165.0) and n_shot < 22 and reason == "ok":
            if (time.monotonic() - t0) > 140.0:
                reason = "timeout"
                break
            v = V_TURN if node.front > 0.40 else 0.04
            st = pulse(v, W_TURN, 2.20)
            if st == "near":
                st = pulse(0.05, W_TURN, 1.60)
                if st == "near":
                    reason = "near_turn"
                    break
            settle_and_shoot(f"SHOT turn{n_shot}")
            n_shot += 1

        # Phase C: second lane
        dist0 = node.odom_dist
        while n_shot < 30 and reason == "ok":
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
            settle_and_shoot(f"SHOT lane2-{n_shot}")
            n_shot += 1
    except Exception as exc:
        reason = f"exception {exc}"
    finally:
        halt_chassis(cli, reason)
        time.sleep(0.2)
        try:
            cli.close()
        except Exception:
            pass

    node._flush()
    xyz = node.acc
    path = list(node.path)
    origin = np.array([node.lock_x, node.lock_y, 0.0], dtype=np.float64)
    yaw = float(node.lock_yaw)
    R = pose_R(yaw)
    n_raw = node.n_raw
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    print(
        f"END reason={reason!r} shots={n_shot} frames={n_raw} voxels={xyz.shape[0]} "
        f"path_n={len(path)} odom_dist={node.odom_dist:.2f}m yaw={math.degrees(yaw):.1f}",
        flush=True,
    )
    if xyz.shape[0] < 400:
        print("FAIL too few points")
        return 3

    stamp = time.strftime("%H%M%S")
    out = OUT_DIR / f"p2_lab_perception_{stamp}.png"
    iso = OUT_DIR / f"p2_lab_perception_iso_{stamp}.png"
    nplt = render(
        xyz, origin, R, yaw, out, elev=64.0, azim_off=-90.0, path_xy=path
    )
    render(xyz, origin, R, yaw, iso, elev=52.0, azim_off=-125.0, path_xy=path)
    slide = OUT_DIR / "p2_lab_perception.png"
    slide.write_bytes(out.read_bytes())
    np.save(OUT_DIR / "p2_lab_cloud.npy", xyz)
    print(f"WROTE {out} plotted={nplt}")
    print(f"WROTE {iso}")
    print(f"WROTE {slide}")
    return 0


if __name__ == "__main__":
    sys.exit(run())
