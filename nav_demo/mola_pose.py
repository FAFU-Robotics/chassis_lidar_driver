#!/usr/bin/env python3
"""Demo-layer MOLA pose watch + ROS spinner.

Stale detection uses receive-side ``time.monotonic()``, not ``header.stamp``.
MOLA may stamp with wall/sensor time; Jetson wall clock is often wrong, and
``MOLA_ROS2_PUBLISH_IN_SIM_TIME`` can diverge from receive time. Do not use
stamp age as the safety clock.

Callers must spin this node's executor on a dedicated thread. A control loop
that ``spin_once`` then blocks on TCP/CAN/grid rebuild will starve pose
callbacks (Python rclpy SingleThreadedExecutor runs one wait-set item per
``spin_once``).
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

# Heartbeats for diagnosis only. Pose stale threshold stays 0.50 in run_first_live.
SPIN_HEARTBEAT_STALE_S = 0.25
MAIN_HEARTBEAT_STALE_S = 0.50


def yaw_from_q(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def qos_mola_pose():
    """Match mola_bridge_ros2: RELIABLE + TRANSIENT_LOCAL."""
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


@dataclass
class PoseSnap:
    x: float
    y: float
    yaw: float
    n: int
    rx_mono: float
    header_stamp_s: float
    age_s: float
    stamp_skew_s: float
    last_gap_s: float


class MolaPoseWatch:
    """Latest ``/lidar_odometry/pose``. Thread-safe. Stale clock = monotonic RX."""

    def __init__(self, node, topic: str = "/lidar_odometry/pose") -> None:
        from nav_msgs.msg import Odometry

        self._lock = threading.Lock()
        self._xyyaw: Optional[tuple[float, float, float]] = None
        self._n = 0
        self._rx_mono = 0.0
        self._stamp_s = 0.0
        self._rx_hist: list[float] = []
        self._sub = node.create_subscription(
            Odometry, topic, self._on_pose, qos_mola_pose()
        )

    def _on_pose(self, msg) -> None:
        p = msg.pose.pose.position
        yaw = yaw_from_q(msg.pose.pose.orientation)
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        now = time.monotonic()
        with self._lock:
            self._xyyaw = (float(p.x), float(p.y), float(yaw))
            self._n += 1
            self._rx_mono = now
            self._stamp_s = stamp
            self._rx_hist.append(now)
            if len(self._rx_hist) > 4000:
                del self._rx_hist[:2000]

    def rx_monotonics(self) -> list[float]:
        with self._lock:
            return list(self._rx_hist)

    def snapshot(self) -> Optional[PoseSnap]:
        wall = time.time()
        now = time.monotonic()
        with self._lock:
            if self._xyyaw is None:
                return None
            x, y, yaw = self._xyyaw
            gap = float("nan")
            if len(self._rx_hist) >= 2:
                gap = self._rx_hist[-1] - self._rx_hist[-2]
            return PoseSnap(
                x=x,
                y=y,
                yaw=yaw,
                n=self._n,
                rx_mono=self._rx_mono,
                header_stamp_s=self._stamp_s,
                age_s=now - self._rx_mono,
                stamp_skew_s=wall - self._stamp_s,
                last_gap_s=gap,
            )

    @property
    def pose(self) -> Optional[tuple[float, float, float]]:
        with self._lock:
            return self._xyyaw


class GridWatch:
    """Latest occupancy grid. Stale clock = monotonic RX."""

    def __init__(self, node, topic: str = "/nav_demo/obstacle_grid") -> None:
        from nav_msgs.msg import OccupancyGrid

        self._lock = threading.Lock()
        self._msg = None
        self._n = 0
        self._rx_mono = 0.0
        self._sub = node.create_subscription(
            OccupancyGrid, topic, self._on_grid, qos_mola_pose()
        )

    def _on_grid(self, msg) -> None:
        now = time.monotonic()
        with self._lock:
            self._msg = msg
            self._n += 1
            self._rx_mono = now

    def snapshot(self):
        now = time.monotonic()
        with self._lock:
            if self._msg is None:
                return None, 0, float("inf")
            return self._msg, self._n, now - self._rx_mono


class NodeSpinThread:
    """Drain ALL ready ROS callbacks on a side thread so TCP/grid cannot starve pose."""

    def __init__(self, node) -> None:
        self._node = node
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._last_spin = 0.0
        self._lock = threading.Lock()

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="nav-demo-ros-spin", daemon=True
        )
        self._thread.start()

    def _run(self) -> None:
        import rclpy

        while not self._stop.is_set() and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.02)
            with self._lock:
                self._last_spin = time.monotonic()

    def spin_age_s(self) -> float:
        with self._lock:
            t = self._last_spin
        if t <= 0.0:
            return float("inf")
        return time.monotonic() - t

    def is_running(self) -> bool:
        th = self._thread
        return th is not None and th.is_alive() and not self._stop.is_set()

    def stop(self) -> None:
        self._stop.set()
        th = self._thread
        if th is not None and th.is_alive() and threading.current_thread() is not th:
            th.join(timeout=1.0)
        self._thread = None


def classify_pose_rx(
    pose_age_s: float,
    spinner_age_s: float,
    main_age_s: float,
    stale_s: float = 0.50,
) -> str:
    """OK | STALE_MOLA | NO_SPIN | MAIN_STUCK. ``stale_s`` must stay 0.50 for safety."""
    if spinner_age_s > SPIN_HEARTBEAT_STALE_S:
        return "NO_SPIN"
    if pose_age_s > stale_s:
        return "STALE_MOLA"
    if main_age_s > MAIN_HEARTBEAT_STALE_S:
        return "MAIN_STUCK"
    return "OK"


def format_pose_rx(
    snap: Optional[PoseSnap],
    spinner: NodeSpinThread,
    main_tick_mono: float,
    stale_s: float = 0.50,
) -> str:
    spin_age = spinner.spin_age_s()
    main_age = time.monotonic() - main_tick_mono if main_tick_mono > 0 else float("inf")
    if snap is None:
        age, n, gap = float("inf"), 0, float("nan")
    else:
        age, n, gap = snap.age_s, snap.n, snap.last_gap_s
    state = classify_pose_rx(age, spin_age, main_age, stale_s=stale_s)
    gap_s = "nan" if gap != gap else f"{gap:.3f}"
    return (
        f"POSE_RX age={age:.3f} n_pose={n} gap={gap_s} state={state} "
        f"spin_age={spin_age:.3f} main_age={main_age:.3f}"
    )
