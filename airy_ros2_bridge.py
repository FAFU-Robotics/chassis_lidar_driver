#!/usr/bin/python3
"""
ROS 2 Humble bridge: RoboSense Airy → sensor_msgs/PointCloud2

Publishes gated full/degraded frames from the existing Airy UDP driver to:
  Topic : /airy/points
  Type  : sensor_msgs/msg/PointCloud2

IMPORTANT
---------
- Does NOT import the ``robosense_airy`` package (avoids ``__init__.py`` / WRS).
- Loads ``robosense_airy/airy_driver.py`` directly via importlib.
- Does NOT re-parse UDP / MSOP; reuses ``RoboSenseAiry.poll_frame()``.
- No chassis / CAN / obstacle / TF / SLAM logic.

Usage (ROS 2 Humble, system Python 3.10)::

    source /opt/ros/humble/setup.bash
    /usr/bin/python3 airy_ros2_bridge.py
    /usr/bin/python3 airy_ros2_bridge.py --msop-port 6699 --verbose
"""
from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path
from typing import Optional, Type

import numpy as np

# ---------------------------------------------------------------------------
# Load RoboSenseAiry WITHOUT importing package robosense_airy/__init__.py
# ---------------------------------------------------------------------------
_THIS_DIR = Path(__file__).resolve().parent
_AIRY_DRIVER_PATH = _THIS_DIR / "robosense_airy" / "airy_driver.py"


def _load_robosense_airy_class() -> Type:
    """
    Import ``RoboSenseAiry`` from the driver file as a standalone module.

    Using a unique module name (not ``robosense_airy.*``) ensures Python does
    not execute ``robosense_airy/__init__.py`` and does not pull in WRS.
    """
    if not _AIRY_DRIVER_PATH.is_file():
        raise FileNotFoundError(f"Airy driver not found: {_AIRY_DRIVER_PATH}")

    # Unique name — must NOT be under package path ``robosense_airy``
    module_name = "_chassis_airy_driver_standalone"
    spec = importlib.util.spec_from_file_location(module_name, _AIRY_DRIVER_PATH)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot create import spec for {_AIRY_DRIVER_PATH}")

    module = importlib.util.module_from_spec(spec)
    # Register before exec so dataclasses / relative lookups stay consistent
    sys.modules[module_name] = module
    spec.loader.exec_module(module)

    if not hasattr(module, "RoboSenseAiry"):
        raise AttributeError(f"{_AIRY_DRIVER_PATH} has no RoboSenseAiry")
    return module.RoboSenseAiry


# ---------------------------------------------------------------------------
# ROS 2 imports (after sys path / driver load prep; require Humble env)
# ---------------------------------------------------------------------------
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


POINT_STEP = 13  # xyz float32 (12) + intensity uint8 (1), no padding


def _numpy_xyz_intensity_to_cloud2(
        pcd: np.ndarray,
        intensity: np.ndarray,
        stamp,
        frame_id: str = "airy") -> PointCloud2:
    """
    Pack N×3 float32 XYZ + N uint8 intensity into PointCloud2.

    Layout (little-endian, point_step=13)::

        offset 0  : x          FLOAT32
        offset 4  : y          FLOAT32
        offset 8  : z          FLOAT32
        offset 12 : intensity  UINT8
    """
    if pcd.ndim != 2 or pcd.shape[1] != 3:
        raise ValueError(f"pcd must be N×3, got shape={getattr(pcd, 'shape', None)}")

    n = int(pcd.shape[0])
    if len(intensity) != n:
        raise ValueError(
            f"intensity length {len(intensity)} != point count {n}")

    # Explicit itemsize=13 prevents NumPy from inserting alignment padding.
    dtype = np.dtype({
        "names": ["x", "y", "z", "intensity"],
        "formats": ["<f4", "<f4", "<f4", "u1"],
        "offsets": [0, 4, 8, 12],
        "itemsize": POINT_STEP,
    })
    arr = np.empty(n, dtype=dtype)
    xyz = np.asarray(pcd, dtype=np.float32, order="C")
    arr["x"] = xyz[:, 0]
    arr["y"] = xyz[:, 1]
    arr["z"] = xyz[:, 2]
    arr["intensity"] = np.asarray(intensity, dtype=np.uint8).reshape(-1)

    msg = PointCloud2()
    msg.header = Header()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = n
    msg.is_bigendian = False
    msg.point_step = POINT_STEP
    msg.row_step = POINT_STEP * n
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(
            name="intensity", offset=12,
            datatype=PointField.UINT8, count=1),
    ]
    msg.data = arr.tobytes()
    # is_dense: True iff every XYZ is finite (no NaN/Inf)
    msg.is_dense = bool(n == 0 or np.isfinite(xyz).all())
    return msg


class AiryRos2Bridge(Node):
    """Poll gated Airy frames and publish /airy/points."""

    def __init__(self,
                 msop_port: int = 6699,
                 min_distance: float = 0.2,
                 max_distance: float = 200.0,
                 verbose: bool = False):
        super().__init__("airy_ros2_bridge")

        RoboSenseAiry = _load_robosense_airy_class()
        self._verbose = verbose
        self._last_seq = 0
        self._pub_count = 0

        # dense_points=True: driver already drops OOR returns; poll_frame
        # still uses remove_nan for safety. Frame FULL/DEGRADED gate stays
        # inside RoboSenseAiry — this node never re-assembles UDP.
        self._lidar = RoboSenseAiry(
            msop_port=msop_port,
            min_distance=min_distance,
            max_distance=max_distance,
            dense_points=True,
            split_angle=0.0,
            verbose=verbose,
            display_max_points=None,  # publish full gated cloud, no viz subsample
        )

        self._pub = self.create_publisher(PointCloud2, "/airy/points", 10)
        # Poll faster than lidar frame rate (~10 Hz); publish only on new seq.
        self._timer = self.create_timer(0.01, self._on_timer)

        self.get_logger().info(
            f"Airy → ROS 2 bridge ready | UDP :{msop_port} | "
            f"topic=/airy/points | frame_id=airy"
        )

    def _on_timer(self) -> None:
        pcd, intensity, seq = self._lidar.poll_frame(
            last_seq=self._last_seq, remove_nan=True)
        if pcd is None or len(pcd) == 0:
            return
        if seq == self._last_seq:
            return

        self._last_seq = seq
        stamp = self.get_clock().now().to_msg()
        msg = _numpy_xyz_intensity_to_cloud2(
            pcd, intensity, stamp=stamp, frame_id="airy")
        self._pub.publish(msg)
        self._pub_count += 1

        if self._verbose or (self._pub_count % 10 == 1):
            quality = getattr(self._lidar, "last_frame_quality", "?")
            self.get_logger().info(
                f"pub #{self._pub_count} seq={seq} points={len(pcd)} "
                f"quality={quality}"
            )

    def destroy_node(self):
        try:
            if hasattr(self, "_lidar") and self._lidar is not None:
                self._lidar.stop()
        except Exception as exc:  # noqa: BLE001 — shutdown must not raise
            self.get_logger().warn(f"lidar.stop() raised: {exc}")
        super().destroy_node()


def main(argv: Optional[list] = None) -> None:
    parser = argparse.ArgumentParser(
        description="RoboSense Airy → ROS 2 PointCloud2 bridge (/airy/points)")
    parser.add_argument("--msop-port", type=int, default=6699,
                        help="MSOP UDP port (default: 6699)")
    parser.add_argument("--min-distance", type=float, default=0.2,
                        help="Min range filter in metres (default: 0.2)")
    parser.add_argument("--max-distance", type=float, default=200.0,
                        help="Max range filter in metres (default: 200.0)")
    parser.add_argument("--verbose", action="store_true",
                        help="Verbose driver + publish logs")
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    node = AiryRos2Bridge(
        msop_port=args.msop_port,
        min_distance=args.min_distance,
        max_distance=args.max_distance,
        verbose=args.verbose,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
