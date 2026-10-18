#!/usr/bin/python3
"""ROS 2 Humble: lookup TF map→base_link, JSON lines to the local agent.

Must run under system Python after sourcing Humble. Do not bind UDP 6699.

    source /opt/ros/humble/setup.bash
    /usr/bin/python3 maps/tf_pose_bridge.py --socket /tmp/bunker_tf_pose.sock
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


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


def _frame_present(blob: str, name: str) -> bool:
    if not blob or not name:
        return False
    token = f"Frame {name} "
    return (
        token in blob
        or f"Frame: {name}" in blob
        or f'"{name}"' in blob
        or f" {name}\n" in blob
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Forward TF map→base_link to the local agent")
    parser.add_argument("--socket", required=True, help="Unix stream socket created by the agent")
    parser.add_argument("--parent", default="map")
    parser.add_argument("--child", default="base_link")
    parser.add_argument("--hz", type=float, default=20.0)
    args = parser.parse_args()

    try:
        import rclpy
        from rclpy.duration import Duration
        from rclpy.node import Node
        from rclpy.time import Time
        from tf2_ros import Buffer, TransformListener
    except ImportError as exc:
        print(f"需要 ROS 2 Humble (rclpy / tf2_ros): {exc}", file=sys.stderr)
        return 2

    sock = connect_socket(args.socket)
    period = 1.0 / max(1.0, float(args.hz))

    class Bridge(Node):
        def __init__(self) -> None:
            super().__init__("bunker_tf_pose_bridge")
            self._sock = sock
            self._buf = Buffer()
            self._listener = TransformListener(self._buf, self)
            self.create_timer(period, self._tick)

        def _tick(self) -> None:
            frames = ""
            try:
                frames = self._buf.all_frames_as_string()
            except Exception:
                frames = ""
            has_map = _frame_present(frames, args.parent)
            has_odom = _frame_present(frames, "odom")
            msg: dict = {
                "ok": False,
                "hasMap": has_map,
                "hasOdom": has_odom,
                "parent": args.parent,
                "child": args.child,
            }
            try:
                tf = self._buf.lookup_transform(
                    args.parent, args.child, Time(), timeout=Duration(seconds=0.0),
                )
                t = tf.transform.translation
                q = tf.transform.rotation
                yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
                msg.update({
                    "ok": True,
                    "hasMap": True,
                    "x": float(t.x),
                    "y": float(t.y),
                    "yawDeg": math.degrees(yaw),
                })
                try:
                    stamp = tf.header.stamp
                    age = time.time() - (int(stamp.sec) + int(stamp.nanosec) * 1e-9)
                    msg["stampAgeS"] = round(age, 3)
                except Exception:
                    age = None
                # lookup_transform(Time()) 会命中 tf2 缓冲里的最后一帧；
                # MOLA 挂了之后仍可能 ok=true，必须按 stamp 年龄作废。
                if age is None or abs(float(age)) > 0.80:
                    msg["ok"] = False
                    msg["hasMap"] = has_map
                    msg["reason"] = "stale_tf"
            except Exception as exc:
                msg["reason"] = str(exc).split("\n", 1)[0][:120]
            line = (json.dumps(msg, ensure_ascii=False) + "\n").encode("utf-8")
            try:
                self._sock.sendall(line)
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
