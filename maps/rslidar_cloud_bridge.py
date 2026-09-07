#!/usr/bin/python3
"""ROS 2 Humble: /rslidar_points → Unix stream for the local console agent.

Must run under system Python after sourcing Humble. Do not bind UDP 6699.
Frame layout must match bunker_mini.rslidar_cloud (MAGIC RSC1).

    source /opt/ros/humble/setup.bash
    /usr/bin/python3 maps/rslidar_cloud_bridge.py --socket /tmp/bunker_rslidar_cloud.sock
"""
from __future__ import annotations

import argparse
import math
import socket
import struct
import sys
import time

MAGIC = b"RSC1"
HEADER = struct.Struct("<4sI")
POINT = struct.Struct("<ffff")
MAX_POINTS = 20_000

# sensor_msgs/PointField datatype
_FLOAT32 = 7
_FLOAT64 = 8
_UINT8 = 2
_UINT16 = 4
_INT16 = 3
_UINT32 = 6


def _field_map(msg) -> dict[str, tuple[int, int]]:
    return {f.name: (int(f.offset), int(f.datatype)) for f in msg.fields}


def _read_number(buf: bytes, offset: int, datatype: int, big: bool) -> float | None:
    try:
        if datatype == _FLOAT32:
            fmt = ">f" if big else "<f"
            return struct.unpack_from(fmt, buf, offset)[0]
        if datatype == _FLOAT64:
            fmt = ">d" if big else "<d"
            return struct.unpack_from(fmt, buf, offset)[0]
        if datatype == _UINT8:
            return float(buf[offset])
        if datatype == _UINT16:
            fmt = ">H" if big else "<H"
            return float(struct.unpack_from(fmt, buf, offset)[0])
        if datatype == _INT16:
            fmt = ">h" if big else "<h"
            return float(struct.unpack_from(fmt, buf, offset)[0])
        if datatype == _UINT32:
            fmt = ">I" if big else "<I"
            return float(struct.unpack_from(fmt, buf, offset)[0])
    except (struct.error, IndexError):
        return None
    return None


def cloud_to_tuples(msg, max_points: int) -> list[tuple[float, float, float, float]]:
    fields = _field_map(msg)
    if not all(k in fields for k in ("x", "y", "z")):
        return []
    ox, tx = fields["x"]
    oy, ty = fields["y"]
    oz, tz = fields["z"]
    oi = ti = None
    for name in ("intensity", "intensities"):
        if name in fields:
            oi, ti = fields[name]
            break
    step = int(msg.point_step)
    n = int(msg.width) * int(msg.height)
    if n <= 0 or step <= 0:
        return []
    data = bytes(msg.data)
    stride = 1
    if n > max_points > 0:
        stride = max(1, n // max_points)
    big = bool(msg.is_bigendian)
    out: list[tuple[float, float, float, float]] = []
    for i in range(0, n, stride):
        off = i * step
        if off + step > len(data) and off + 12 > len(data):
            break
        x = _read_number(data, off + ox, tx, big)
        y = _read_number(data, off + oy, ty, big)
        z = _read_number(data, off + oz, tz, big)
        if x is None or y is None or z is None:
            continue
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        intensity = 0.0
        if oi is not None and ti is not None:
            raw = _read_number(data, off + oi, ti, big)
            if raw is not None and math.isfinite(raw):
                intensity = raw if raw <= 255.0 else min(255.0, raw)
        out.append((x, y, z, intensity))
        if len(out) >= MAX_POINTS:
            break
    return out


def encode_frame(points: list[tuple[float, float, float, float]]) -> bytes:
    payload = b"".join(POINT.pack(*p) for p in points[:MAX_POINTS])
    return HEADER.pack(MAGIC, len(payload)) + payload


def connect_socket(path: str, timeout_s: float = 8.0) -> socket.socket:
    deadline = time.monotonic() + timeout_s
    last_err: OSError | None = None
    while time.monotonic() < deadline:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            sock.connect(path)
            sock.settimeout(2.0)
            return sock
        except OSError as exc:
            last_err = exc
            sock.close()
            time.sleep(0.1)
    raise ConnectionError(f"connect {path}: {last_err}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Forward /rslidar_points to the local agent")
    parser.add_argument("--socket", required=True, help="Unix stream socket created by the agent")
    parser.add_argument("--topic", default="/rslidar_points")
    parser.add_argument("--max-points", type=int, default=4000)
    args = parser.parse_args()

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2
    except ImportError as exc:
        print(f"需要 ROS 2 Humble (rclpy): {exc}", file=sys.stderr)
        return 2

    sock = connect_socket(args.socket)

    class Bridge(Node):
        def __init__(self) -> None:
            super().__init__("bunker_rslidar_cloud_bridge")
            self._sock = sock
            self.create_subscription(
                PointCloud2, args.topic, self._on_cloud, qos_profile_sensor_data,
            )

        def _on_cloud(self, msg: PointCloud2) -> None:
            pts = cloud_to_tuples(msg, max(1, int(args.max_points)))
            try:
                self._sock.sendall(encode_frame(pts))
            except OSError as exc:
                self.get_logger().error(f"agent socket closed: {exc}")
                raise SystemExit(1) from exc

    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        try:
            sock.close()
        except OSError:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
