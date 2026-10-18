#!/usr/bin/python3
"""Listen-only MOLA localmap growth monitor. No CAN, no chassis control."""
from __future__ import annotations

import json
import math
import os
import struct
import time

os.environ.setdefault("ROS_HOME", "/tmp/ros_ppt_localmap_mon")
os.makedirs(os.environ["ROS_HOME"] + "/log", exist_ok=True)

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, String

LOG = "/tmp/ppt_localmap_grow.log"
SUM = "/tmp/ppt_localmap_grow_summary.json"
MOLA_LOG = "/tmp/nav_demo_bringup/mola.log"

qos_tl = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


def yaw_from_q(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny, cosy))


def xyz_bbox(msg):
    n = msg.width * msg.height
    names = {f.name: f for f in msg.fields}
    if "x" not in names or n == 0:
        return dict(n=n, xmin=None, xmax=None, ymin=None, ymax=None, zmin=None, zmax=None, dx=0, dy=0, dz=0)
    offx, offy, offz = names["x"].offset, names["y"].offset, names["z"].offset
    step = msg.point_step
    data = msg.data
    stride = 1 if n <= 25000 else max(1, n // 20000)
    xmin = ymin = zmin = 1e18
    xmax = ymax = zmax = -1e18
    for i in range(0, n, stride):
        x, y, z = struct.unpack_from("<fff", data, i * step + offx)
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        xmin = min(xmin, x)
        xmax = max(xmax, x)
        ymin = min(ymin, y)
        ymax = max(ymax, y)
        zmin = min(zmin, z)
        zmax = max(zmax, z)
    return dict(
        n=n,
        xmin=xmin,
        xmax=xmax,
        ymin=ymin,
        ymax=ymax,
        zmin=zmin,
        zmax=zmax,
        dx=xmax - xmin,
        dy=ymax - ymin,
        dz=zmax - zmin,
    )


class Mon(Node):
    def __init__(self):
        super().__init__("ppt_localmap_grow_mon")
        self.t0 = time.monotonic()
        self.pose_hist = []
        self.qual_hist = []
        self.lm_hist = []
        self.snaps = []
        self.baseline = None
        self.last_pose = None
        self.last_qual = None
        self.last_lm = None
        self.motion_start_t = None
        self.motion_end_t = None
        self.stop_after = None
        self._still_since = None
        self.done = False
        self.max_gap = 0.0
        self.n_qual_low = 0
        self.qual_min = 1e9
        self.qual_max = -1e9
        self.mola_new = []
        try:
            self.mola_fp = open(MOLA_LOG, "r", errors="replace")
            self.mola_fp.seek(0, os.SEEK_END)
        except OSError:
            self.mola_fp = None
        self.fp = open(LOG, "w")
        self.create_subscription(Odometry, "/lidar_odometry/pose", self.on_pose, qos_tl)
        self.create_subscription(Float32, "/lidar_odometry/pose_quality", self.on_qual, qos_tl)
        self.create_subscription(PointCloud2, "/lidar_odometry/localmap_points", self.on_lm, qos_tl)
        self.create_subscription(String, "/mola_diagnostics/lidar_odom/status", self.on_diag, qos_tl)
        self.create_timer(1.0, self.on_sec)
        self.log("MONITOR_START listen-only. No CAN. Waiting baseline then human teleop ~2m.")

    def log(self, s):
        line = f"{time.monotonic() - self.t0:8.3f}  {s}"
        print(line, flush=True)
        self.fp.write(line + "\n")
        self.fp.flush()

    def drain_mola(self):
        if not self.mola_fp:
            return
        while True:
            line = self.mola_fp.readline()
            if not line:
                break
            s = line.rstrip()
            if any(k in s for k in ("ICP", "WARN", "ERROR", "keyframe", "localmap", "failure")):
                self.mola_new.append(s)
                self.log("MOLA_LOG " + s[-240:])

    def on_pose(self, m):
        now = time.monotonic()
        p = m.pose.pose.position
        yaw = yaw_from_q(m.pose.pose.orientation)
        rec = dict(t=now - self.t0, mono=now, x=p.x, y=p.y, z=p.z, yaw_deg=yaw)
        if self.pose_hist:
            gap = now - self.pose_hist[-1]["mono"]
            self.max_gap = max(self.max_gap, gap)
            rec["gap"] = gap
        self.pose_hist.append(rec)
        self.last_pose = rec

    def on_qual(self, m):
        q = float(m.data)
        self.last_qual = q
        self.qual_hist.append((time.monotonic() - self.t0, q))
        self.qual_min = min(self.qual_min, q)
        self.qual_max = max(self.qual_max, q)
        if q < 0.50:
            self.n_qual_low += 1
            self.log("QUALITY_LOW {:.3f}".format(q))

    def on_lm(self, m):
        rec = dict(t=time.monotonic() - self.t0, stamp=m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, frame=m.header.frame_id, **xyz_bbox(m))
        prev = self.last_lm
        self.lm_hist.append(rec)
        self.last_lm = rec
        if prev is None or rec["n"] != prev["n"] or abs(rec["stamp"] - prev["stamp"]) > 1e-6:
            self.log(
                "LOCALMAP n={} stamp={:.3f} bbox=[{:.2f},{:.2f}]x[{:.2f},{:.2f}]x[{:.2f},{:.2f}] size={:.1f}x{:.1f}x{:.1f}m".format(
                    rec["n"], rec["stamp"], rec["xmin"], rec["xmax"], rec["ymin"], rec["ymax"], rec["zmin"], rec["zmax"], rec["dx"], rec["dy"], rec["dz"]
                )
            )
            if prev is not None:
                self.log("KEYFRAME_CANDIDATE n{:+d} stamp{:+.3f}s".format(rec["n"] - prev["n"], rec["stamp"] - prev["stamp"]))

    def on_diag(self, m):
        one = m.data.replace("\n", " | ")[:280]
        self.log("DIAG " + one)

    def maybe_baseline(self):
        if self.baseline or self.last_pose is None or self.last_lm is None or self.last_qual is None:
            return
        p, lm = self.last_pose, self.last_lm
        self.baseline = dict(
            x=p["x"], y=p["y"], z=p["z"], yaw_deg=p["yaw_deg"], quality=self.last_qual,
            lm_n=lm["n"], lm_stamp=lm["stamp"], bbox={k: lm[k] for k in ("xmin", "xmax", "ymin", "ymax", "zmin", "zmax", "dx", "dy", "dz")},
        )
        b = self.baseline
        self.log(
            "BASELINE pose=({:.3f},{:.3f}) yaw={:.1f} q={:.3f} localmap n={} size={:.1f}x{:.1f}x{:.1f}m".format(
                b["x"], b["y"], b["yaw_deg"], b["quality"], b["lm_n"], b["bbox"]["dx"], b["bbox"]["dy"], b["bbox"]["dz"]
            )
        )
        self.log("WAITING_TELEOP 现在可以人工遥控机器人缓慢直行约 2 m，周围保持有墙、桌椅等明显环境结构；遥控完成后立即松开遥控器。")

    def on_sec(self):
        self.drain_mola()
        self.maybe_baseline()
        if self.done:
            return
        p, lm = self.last_pose, self.last_lm
        if p is None or lm is None:
            return
        disp = 0.0
        dyaw = 0.0
        if self.baseline:
            disp = math.hypot(p["x"] - self.baseline["x"], p["y"] - self.baseline["y"])
            dyaw = abs(p["yaw_deg"] - self.baseline["yaw_deg"])
            if dyaw > 180:
                dyaw = 360 - dyaw
        self.snaps.append(
            dict(
                t=p["t"], x=p["x"], y=p["y"], yaw=p["yaw_deg"], q=self.last_qual,
                n=lm["n"], stamp=lm["stamp"], bbox=(lm["dx"], lm["dy"], lm["dz"]), disp=disp,
            )
        )
        self.log(
            "SNAP x={:.3f} y={:.3f} yaw={:.1f} q={} n={} stamp={:.3f} bbox={:.2f}x{:.2f}x{:.2f} disp={:.3f}".format(
                p["x"], p["y"], p["yaw_deg"],
                "{:.3f}".format(self.last_qual) if self.last_qual is not None else "n/a",
                lm["n"], lm["stamp"], lm["dx"], lm["dy"], lm["dz"], disp,
            )
        )
        now = time.monotonic()
        if self.baseline and self.motion_start_t is None and disp > 0.08:
            self.motion_start_t = now
            self.log("MOTION_START disp={:.3f} dyaw={:.1f}".format(disp, dyaw))
        if self.motion_start_t and disp >= 1.40 and self.motion_end_t is None:
            recent = [h for h in self.pose_hist if h["mono"] >= now - 1.0]
            speed = 0.0
            if len(recent) >= 4:
                path = 0.0
                for i in range(1, len(recent)):
                    path += math.hypot(recent[i]["x"] - recent[i - 1]["x"], recent[i]["y"] - recent[i - 1]["y"])
                speed = path / max(1e-3, recent[-1]["mono"] - recent[0]["mono"])
            if speed < 0.03:
                if self._still_since is None:
                    self._still_since = now
                elif now - self._still_since >= 5.0:
                    self.motion_end_t = now
                    self.stop_after = now + 5.0
                    self.log("MOTION_END disp={:.3f}; extra 5s".format(disp))
            else:
                self._still_since = None
        if self.stop_after is not None and now >= self.stop_after:
            self.finish("complete")
            return
        elapsed = now - self.t0
        if self.motion_start_t is None and elapsed > 1200:
            self.finish("timeout_no_motion")
        elif self.motion_start_t is not None and now - self.motion_start_t > 240 and self.motion_end_t is None:
            if disp >= 1.40:
                self.motion_end_t = now
                self.stop_after = now + 5.0
                self.log("MOTION_END_TIMEOUT extra 5s")
            elif now - self.motion_start_t > 300:
                self.finish("timeout_during_motion")

    def finish(self, reason):
        if self.done:
            return
        self.done = True
        p = self.last_pose or {}
        lm = self.last_lm or {}
        b = self.baseline or {}
        disp = 0.0
        if p and b:
            disp = math.hypot(p.get("x", 0) - b.get("x", 0), p.get("y", 0) - b.get("y", 0))

        def jitter(window):
            if len(window) < 5:
                return None
            xs = [h["x"] for h in window]
            ys = [h["y"] for h in window]
            return dict(n=len(window), x_span=max(xs) - min(xs), y_span=max(ys) - min(ys),
                        xy_span=max(math.hypot(x - xs[0], y - ys[0]) for x, y in zip(xs, ys)))

        pre = [h for h in self.pose_hist if self.motion_start_t and h["mono"] < self.motion_start_t]
        if not pre:
            pre = self.pose_hist[:20]
        post = self.pose_hist[-40:]
        ns = [h["n"] for h in self.lm_hist]
        kf = max(0, len([h for h in self.lm_hist[1:] if True]) )  # recount below
        kf = 0
        if self.lm_hist:
            prev = self.lm_hist[0]
            for h in self.lm_hist[1:]:
                if h["n"] != prev["n"] or abs(h["stamp"] - prev["stamp"]) > 1e-6:
                    kf += 1
                prev = h
        grown = False
        if ns:
            grown = (ns[-1] - ns[0] >= 400) or (
                lm.get("dx", 0) - (b.get("bbox") or {}).get("dx", 0) > 0.4
                or lm.get("dy", 0) - (b.get("bbox") or {}).get("dy", 0) > 0.4
            )
        summary = dict(
            reason=reason,
            baseline=b,
            final_pose=p,
            final_quality=self.last_qual,
            final_lm=lm,
            disp_m=disp,
            n_pose=len(self.pose_hist),
            pose_max_gap_s=self.max_gap,
            qual_min=None if self.qual_min > 1e8 else self.qual_min,
            qual_max=None if self.qual_max < -1e8 else self.qual_max,
            n_qual_below_0_50=self.n_qual_low,
            localmap_n_first=ns[0] if ns else None,
            localmap_n_last=ns[-1] if ns else None,
            localmap_n_delta=(ns[-1] - ns[0]) if ns else None,
            keyframe_candidates=kf,
            grown=grown,
            pre_motion_jitter=jitter(pre[-80:]),
            end_jitter=jitter(post),
            icp_fail_lines=[s for s in self.mola_new if "ICP failure" in s],
            snaps=self.snaps,
        )
        with open(SUM, "w") as f:
            json.dump(summary, f, indent=2, default=str)
        self.log(
            "FINISH reason={} disp={:.3f} n {} -> {} delta={} kf={} grown={} qmin={} qlow={}".format(
                reason, disp, summary["localmap_n_first"], summary["localmap_n_last"],
                summary["localmap_n_delta"], kf, grown,
                "{:.3f}".format(summary["qual_min"]) if summary["qual_min"] is not None else "n/a",
                self.n_qual_low,
            )
        )
        self.fp.close()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = Mon()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    if not node.done:
        try:
            node.finish("interrupted")
        except Exception:
            pass


if __name__ == "__main__":
    main()
