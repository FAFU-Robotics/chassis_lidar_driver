#!/usr/bin/python3
"""Listen-only: log /lidar_odometry/pose while the operator drives. No CAN."""
from __future__ import annotations

import csv
import math
import os
import signal
import sys
import time
from pathlib import Path

os.environ.setdefault("ROS_HOME", "/tmp/ros_ppt_pose_log")
os.makedirs(os.environ["ROS_HOME"] + "/log", exist_ok=True)

OUT = Path("/home/fafu_robot/Desktop/chassis_lidar_drivers/ppt_materials/generated")
OUT.mkdir(parents=True, exist_ok=True)
CSV = OUT / "drive_pose.csv"
STATUS = OUT / "drive_status.txt"


def yaw_of_q(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main() -> int:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float64

    qos_tl = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )
    qos_vol = QoSProfile(
        depth=20,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )

    stop = {"n": False}

    def _sig(_s, _f):
        stop["n"] = True

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    rclpy.init()
    node = Node("ppt_drive_pose_log")
    rows = []
    box = {"pose": None, "q": None, "n": 0}

    def on_pose(m):
        p = m.pose.pose.position
        y = yaw_of_q(m.pose.pose.orientation)
        rec = {
            "t": time.time(),
            "x": float(p.x),
            "y": float(p.y),
            "z": float(p.z),
            "yaw": y,
            "q": box["q"],
        }
        box["pose"] = rec
        box["n"] += 1
        rows.append(rec)

    def on_q(m):
        box["q"] = float(m.data)

    node.create_subscription(Odometry, "/lidar_odometry/pose", on_pose, qos_tl)
    node.create_subscription(Float64, "/lidar_odometry/pose_quality", on_q, qos_vol)
    t0 = time.monotonic()
    last_print = 0.0
    print(f"POSE LOG {CSV} — listen only, no CAN", flush=True)
    while not stop["n"]:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        if now - last_print >= 2.0:
            last_print = now
            p = box["pose"]
            if p:
                line = (
                    f"t={now-t0:6.1f}s n={box['n']:4d} "
                    f"xy=({p['x']:.2f},{p['y']:.2f}) yaw={math.degrees(p['yaw']):6.1f} "
                    f"q={p['q']}"
                )
                print(line, flush=True)
                STATUS.write_text(line + "\n")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if rows:
        with CSV.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["t", "x", "y", "z", "yaw", "q"])
            w.writeheader()
            w.writerows(rows)
        print(f"WROTE {CSV} n={len(rows)}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
