#!/usr/bin/env python3
"""LaserScan (/scan) → PointCloud2 (/scan_cloud) in the scan frame, z=0.

Gives MOLA-LO a wall-height 2D slice as PointCloud2 so 3D ICP does not
see the Airy ceiling dome. QoS: subscribe best-effort, publish reliable.
"""
from __future__ import annotations

import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header


def _cloud(header: Header, xs: list[float], ys: list[float]) -> PointCloud2:
    buf = bytearray()
    for x, y in zip(xs, ys):
        buf += struct.pack("<ffff", x, y, 0.0, 1.0)
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(xs)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * len(xs)
    msg.data = bytes(buf)
    msg.is_dense = True
    return msg


class ScanToCloud(Node):
    def __init__(self) -> None:
        super().__init__("scan_to_cloud")
        pub_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(PointCloud2, "/scan_cloud", pub_qos)
        self.create_subscription(LaserScan, "/scan", self._cb, qos_profile_sensor_data)

    def _cb(self, scan: LaserScan) -> None:
        xs: list[float] = []
        ys: list[float] = []
        a = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                xs.append(r * math.cos(a))
                ys.append(r * math.sin(a))
            a += scan.angle_increment
        if xs:
            self._pub.publish(_cloud(scan.header, xs, ys))


def main() -> None:
    rclpy.init()
    node = ScanToCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
