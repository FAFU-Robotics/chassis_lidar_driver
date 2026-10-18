#!/usr/bin/env python3
"""Republish Airy IMU into ROS REP-145: m/s², Z-up.

Raw /rslidar_imu_data at rest is ~(-1, 0, 0) with |a|≈1 (g, X vertical).
Map: x_out=z_in, y_out=y_in, z_out=-x_in, then acc *= 9.81.
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

G = 9.80665


class AiryImuEnu(Node):
    def __init__(self) -> None:
        super().__init__("airy_imu_enu")
        qos = QoSProfile(
            depth=200,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub = self.create_publisher(Imu, "/imu", qos)
        self.create_subscription(Imu, "/rslidar_imu_data", self._cb, qos)

    def _cb(self, msg: Imu) -> None:
        ax, ay, az = (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )
        wx, wy, wz = (
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        )
        out = Imu()
        out.header = msg.header
        out.header.frame_id = "base_link"
        # R_y(+90°): X_imu (up) → -Z_base? z_out = -x_in → +g on Z
        out.linear_acceleration.x = az * G
        out.linear_acceleration.y = ay * G
        out.linear_acceleration.z = -ax * G
        out.angular_velocity.x = wz
        out.angular_velocity.y = wy
        out.angular_velocity.z = -wx
        out.orientation_covariance[0] = -1.0
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = AiryImuEnu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
