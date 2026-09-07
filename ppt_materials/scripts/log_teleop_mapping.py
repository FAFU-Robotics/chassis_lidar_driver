#!/usr/bin/python3
"""Listen-only mapping session log. No CAN, no stick, does not kill other processes."""
from __future__ import annotations

import csv
import json
import math
import os
import signal
import time
from pathlib import Path

os.environ.setdefault("ROS_HOME", "/tmp/ros_ppt_teleop_map")
os.makedirs(os.environ["ROS_HOME"] + "/log", exist_ok=True)

OUT = Path(
    os.environ.get(
        "PPT_TELEOP_OUT",
        "/home/fafu_robot/Desktop/chassis_lidar_drivers/ppt_materials/generated/final_20260904/teleop_map2",
    )
)
OUT.mkdir(parents=True, exist_ok=True)
CSV = OUT / "samples.csv"
STATUS = OUT / "status.json"
START = OUT / "start.json"
LOG = OUT / "session.log"
MOLA_LOG = os.environ.get("PPT_MOLA_LOG", "/tmp/nav_demo_bringup/mola_mapping.log")


def yaw_deg(q) -> float:
    return math.degrees(
        math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    )


def main() -> int:
    import numpy as np
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Float32, String
    from tf2_ros import Buffer, TransformListener

    qos_tl = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )
    qos_rel = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
    qos_be = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)

    stop = False

    def _sig(_s, _f):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    rclpy.init()
    node = Node("ppt_teleop_map_log")
    buf = Buffer()
    TransformListener(buf, node)

    t0 = time.monotonic()
    wall0 = time.strftime("%Y-%m-%d %H:%M:%S")
    lidar_ts: list[float] = []
    pose_ts: list[float] = []
    last = {
        "pose": None,
        "q": None,
        "local_n": None,
        "raw_n": None,
        "diag": None,
        "tf_ok": False,
        "pose_n": 0,
        "raw_n_msg": 0,
        "last_pose_mono": None,
        "pose_xy": None,
        "wheel": None,
        "wheel0": None,
    }
    icp_lines: list[str] = []
    freeze_s = 0.0
    qs: list[float] = []
    csv_f = CSV.open("w", newline="")
    wr = csv.writer(csv_f)
    wr.writerow(
        [
            "t",
            "x",
            "y",
            "z",
            "yaw_deg",
            "quality",
            "lidar_hz",
            "pose_hz",
            "localmap_n",
            "raw_n",
            "tf_ok",
            "pose_age_s",
            "icp_fail_n",
            "wheel_x",
            "wheel_y",
            "wheel_yaw_deg",
        ]
    )
    log_fp = LOG.open("w")

    def log(s: str) -> None:
        line = f"{time.monotonic() - t0:7.2f}  {s}"
        print(line, flush=True)
        log_fp.write(line + "\n")
        log_fp.flush()

    try:
        mola_fp = open(MOLA_LOG, "r", errors="replace")
        mola_fp.seek(0, os.SEEK_END)
    except OSError:
        mola_fp = None

    def hz(ts: list[float], window: float = 2.0) -> float:
        now = time.time()
        xs = [t for t in ts if now - t <= window]
        if len(xs) < 2:
            return 0.0
        return (len(xs) - 1) / max(1e-6, xs[-1] - xs[0])

    def drain_mola() -> None:
        if not mola_fp:
            return
        while True:
            line = mola_fp.readline()
            if not line:
                break
            if "ICP failure" in line or "Sustained ICP" in line:
                icp_lines.append(line.strip())
                log("ICP  " + line.strip()[-180:])

    def on_pose(m):
        p = m.pose.pose.position
        rec = {
            "x": float(p.x),
            "y": float(p.y),
            "z": float(p.z),
            "yaw": yaw_deg(m.pose.pose.orientation),
        }
        last["pose"] = rec
        last["pose_n"] += 1
        last["last_pose_mono"] = time.monotonic()
        pose_ts.append(time.time())
        if len(pose_ts) > 200:
            del pose_ts[:80]
        xy = (rec["x"], rec["y"])
        if last["pose_xy"] is None:
            last["pose_xy"] = xy
            START.write_text(
                json.dumps({"wall": wall0, "pose": rec, "quality": last["q"], "localmap_n": last["local_n"]}, indent=2)
            )
            log(
                f"START xy=({rec['x']:.3f},{rec['y']:.3f}) yaw={rec['yaw']:.1f} "
                f"q={last['q']} localmap={last['local_n']}"
            )

    def on_q(m):
        q = float(m.data)
        last["q"] = q
        qs.append(q)

    def on_local(m):
        last["local_n"] = int(m.width) * int(m.height)

    def on_raw(m):
        last["raw_n"] = int(m.width) * int(m.height)
        last["raw_n_msg"] += 1
        lidar_ts.append(time.time())
        if len(lidar_ts) > 200:
            del lidar_ts[:80]

    def on_diag(m):
        last["diag"] = m.data

    def on_wheel(m):
        p = m.pose.pose.position
        yaw = yaw_deg(m.pose.pose.orientation)
        if last["wheel0"] is None:
            last["wheel0"] = (float(p.x), float(p.y), yaw)
        x0, y0, yaw0 = last["wheel0"]
        # relative to logger start so a U is visible even if /wheel_odom
        # has been integrating all afternoon
        dx = float(p.x) - x0
        dy = float(p.y) - y0
        c, s = math.cos(-math.radians(yaw0)), math.sin(-math.radians(yaw0))
        last["wheel"] = {
            "x": c * dx - s * dy,
            "y": s * dx + c * dy,
            "yaw": yaw - yaw0,
        }

    node.create_subscription(Odometry, "/lidar_odometry/pose", on_pose, qos_tl)
    node.create_subscription(Odometry, "/wheel_odom", on_wheel, qos_rel)
    node.create_subscription(Float32, "/lidar_odometry/pose_quality", on_q, qos_tl)
    node.create_subscription(PointCloud2, "/lidar_odometry/localmap_points", on_local, qos_tl)
    node.create_subscription(PointCloud2, "/rslidar_points", on_raw, qos_rel)
    node.create_subscription(PointCloud2, "/rslidar_points", on_raw, qos_be)
    node.create_subscription(String, "/mola_diagnostics/lidar_odom/status", on_diag, qos_tl)

    log("RECORDING listen-only. Operator drives. No CAN from this node.")
    last_row = 0.0
    last_print = 0.0
    while not stop:
        rclpy.spin_once(node, timeout_sec=0.05)
        drain_mola()
        now = time.monotonic()
        try:
            buf.lookup_transform("map", "rslidar", rclpy.time.Time())
            last["tf_ok"] = True
        except Exception:
            last["tf_ok"] = False
        age = None if last["last_pose_mono"] is None else now - last["last_pose_mono"]
        if age is not None and age > 0.8:
            freeze_s = max(freeze_s, age)
        if now - last_row >= 0.4 and last["pose"]:
            last_row = now
            p = last["pose"]
            w = last["wheel"]
            wr.writerow(
                [
                    f"{now - t0:.3f}",
                    f"{p['x']:.6f}",
                    f"{p['y']:.6f}",
                    f"{p['z']:.6f}",
                    f"{p['yaw']:.3f}",
                    "" if last["q"] is None else f"{last['q']:.4f}",
                    f"{hz(lidar_ts):.3f}",
                    f"{hz(pose_ts):.3f}",
                    last["local_n"] if last["local_n"] is not None else "",
                    last["raw_n"] if last["raw_n"] is not None else "",
                    int(last["tf_ok"]),
                    "" if age is None else f"{age:.3f}",
                    len(icp_lines),
                    "" if w is None else f"{w['x']:.6f}",
                    "" if w is None else f"{w['y']:.6f}",
                    "" if w is None else f"{w['yaw']:.3f}",
                ]
            )
            csv_f.flush()
        if now - last_print >= 2.0:
            last_print = now
            p = last["pose"]
            freeze = age is not None and age > 0.8
            line = (
                f"t={now-t0:5.1f}s lidar={hz(lidar_ts):4.1f}Hz pose={hz(pose_ts):4.1f}Hz "
                f"q={last['q']} local={last['local_n']} raw_n={last['raw_n']} "
                f"tf={int(last['tf_ok'])} icp_fail_lines={len(icp_lines)}"
            )
            if p:
                line += f" xy=({p['x']:.2f},{p['y']:.2f}) yaw={p['yaw']:.1f}"
            if last["wheel"]:
                w = last["wheel"]
                line += f" wheel=({w['x']:.2f},{w['y']:.2f},{w['yaw']:.0f})"
            if freeze:
                line += f" POSE_STALE age={age:.2f}s"
            log(line)
            STATUS.write_text(
                json.dumps(
                    {
                        "t": now - t0,
                        "pose": last["pose"],
                        "quality": last["q"],
                        "localmap_n": last["local_n"],
                        "raw_n": last["raw_n"],
                        "lidar_hz": hz(lidar_ts),
                        "pose_hz": hz(pose_ts),
                        "tf_ok": last["tf_ok"],
                        "pose_age_s": age,
                        "icp_fail_n": len(icp_lines),
                        "qual_min": min(qs) if qs else None,
                        "qual_max": max(qs) if qs else None,
                    },
                    indent=2,
                )
            )

    p = last["pose"]
    end = {
        "wall": time.strftime("%Y-%m-%d %H:%M:%S"),
        "elapsed_s": time.monotonic() - t0,
        "pose": p,
        "quality": last["q"],
        "localmap_n": last["local_n"],
        "raw_n": last["raw_n"],
        "icp_fail_lines": icp_lines,
        "qual_min": min(qs) if qs else None,
        "qual_max": max(qs) if qs else None,
        "qual_avg": (sum(qs) / len(qs)) if qs else None,
        "max_pose_age_s": freeze_s,
        "pose_msgs": last["pose_n"],
        "raw_msgs": last["raw_n_msg"],
    }
    (OUT / "end.json").write_text(json.dumps(end, indent=2))
    log(f"STOP {json.dumps(end, ensure_ascii=False)[:400]}")
    csv_f.close()
    log_fp.close()
    if mola_fp:
        mola_fp.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
