#!/usr/bin/env python3
"""Read-only live stack check for PPT materials. No YAML/SDK/MOLA/TF changes."""
from __future__ import annotations

import json
import os
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parents[1]
OUT = HERE / "live_status.json"
WANTED = [
    "/rslidar_points",
    "/lidar_odometry/pose",
    "/lidar_odometry/deskewed_scan_points",
    "/lidar_odometry/localmap_points",
    "/tf",
    "/tf_static",
    "/wheel_odom",
    "/map",
    "/odom",
    "/plan",
    "/local_plan",
    "/cmd_vel",
    "/horizon_localmap",
]
NAV_ABSENT_OK = ["/map", "/odom", "/plan", "/local_plan", "/cmd_vel"]
PROCS = [
    ("rslidar_sdk", "rslidar_sdk"),
    ("mola", "mola-cli"),
    ("wheel_odom", "publish_wheel_odom.py"),
    ("tf_airy", "tf_airy_mount"),
    ("teleop_agent", "run_local.py"),
    ("rviz2", "rviz2"),
]


def bash_ros(cmd: str, timeout: float = 10.0) -> subprocess.CompletedProcess:
    env = os.environ.copy()
    env["PATH"] = "/usr/bin:/usr/sbin:" + env.get("PATH", "")
    env["HOME"] = env.get("HOME", "/home/fafu_robot")
    env["ROS_DOMAIN_ID"] = env.get("ROS_DOMAIN_ID", "0")
    env["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(ROOT / "maps/fastdds_udp.xml")
    for k in ("CONDA_PREFIX", "CONDA_DEFAULT_ENV", "CONDA_PYTHON_EXE", "CONDA_SHLVL"):
        env.pop(k, None)
    wrapped = (
        "set +u; unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PYTHON_EXE CONDA_SHLVL; "
        "source /opt/ros/humble/setup.bash >/dev/null; "
        "source /home/fafu_robot/rslidar_ws/install/setup.bash >/dev/null 2>&1; "
        + cmd
    )
    return subprocess.run(
        ["bash", "-lc", wrapped],
        capture_output=True,
        text=True,
        timeout=timeout,
        env=env,
    )


def proc_alive(needle: str) -> bool:
    r = subprocess.run(["pgrep", "-af", needle], capture_output=True, text=True)
    return any(needle in line and "pgrep" not in line for line in r.stdout.splitlines())


def port_listen(port: int) -> bool:
    r = subprocess.run(["ss", "-ltn"], capture_output=True, text=True)
    return f":{port}" in r.stdout


def main() -> int:
    try:
        lst = bash_ros("timeout 8 ros2 topic list", timeout=12)
        topics = {t.strip() for t in lst.stdout.splitlines() if t.startswith("/")}
    except Exception as exc:
        topics = set()
        lst_err = str(exc)
    else:
        lst_err = (lst.stderr or "").strip()[-400:]

    topic_status = {t: ("present" if t in topics else "absent") for t in WANTED}
    report = {
        "ts_unix": time.time(),
        "processes": {name: proc_alive(pat) for name, pat in PROCS},
        "teleop_tcp_9100": port_listen(9100),
        "topics": topic_status,
        "ready_for_ABC_recording": all(
            topic_status[t] == "present"
            for t in (
                "/rslidar_points",
                "/lidar_odometry/pose",
                "/lidar_odometry/deskewed_scan_points",
                "/tf",
                "/wheel_odom",
            )
        )
        and proc_alive("mola-cli")
        and port_listen(9100),
        "absent_navigation_topics": [t for t in NAV_ABSENT_OK if topic_status.get(t) == "absent"],
        "horizon_localmap_has_no_use_for_ppt": topic_status.get("/horizon_localmap") == "present",
        "ros2_list_error_tail": lst_err,
    }
    OUT.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    print("wrote", OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
