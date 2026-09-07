#!/usr/bin/env python3
"""PPT recording preflight. Read-only. Never sends chassis velocity."""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import time
from pathlib import Path

ROOT = Path("/home/fafu_robot/Desktop/chassis_lidar_drivers")
HERE = Path(__file__).resolve().parents[1]
RVIZ = HERE / "rviz" / "ppt_live.rviz"


def bash_ros(cmd: str, timeout: float = 12.0) -> subprocess.CompletedProcess:
    env = os.environ.copy()
    env["PATH"] = "/usr/bin:/usr/sbin:" + env.get("PATH", "")
    env["HOME"] = "/home/fafu_robot"
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


def ok(msg: str) -> None:
    print(f"[OK]   {msg}", flush=True)


def bad(msg: str) -> None:
    print(f"[FAIL] {msg}", flush=True)


def info(msg: str) -> None:
    print(f"[INFO] {msg}", flush=True)


def topic_once(topic: str) -> bool:
    extra = ""
    if topic in (
        "/rslidar_points",
        "/lidar_odometry/deskewed_scan_points",
        "/lidar_odometry/localmap_points",
    ):
        extra = " --qos-reliability best_effort"
    r = bash_ros(f"timeout 5 ros2 topic echo {topic} --once{extra} >/dev/null 2>&1")
    if r.returncode == 0:
        return True
    if extra:
        r2 = bash_ros(f"timeout 5 ros2 topic echo {topic} --once >/dev/null 2>&1")
        return r2.returncode == 0
    return False


def echo_text(topic: str, timeout: float = 4.0) -> str:
    r = bash_ros(f"timeout {int(timeout)} ros2 topic echo {topic} --once")
    return r.stdout or ""


def yaw_from_echo(txt: str) -> float | None:
    # crude parse of orientation xyzw
    vals = {}
    lines = txt.splitlines()
    in_ori = False
    for i, line in enumerate(lines):
        s = line.strip()
        if s == "orientation:":
            in_ori = True
            continue
        if in_ori and ":" in s and s.split(":")[0] in ("x", "y", "z", "w"):
            k, v = s.split(":", 1)
            try:
                vals[k.strip()] = float(v.strip())
            except ValueError:
                pass
        if in_ori and len(vals) >= 4:
            break
    if not all(k in vals for k in "xyzw"):
        return None
    x, y, z, w = vals["x"], vals["y"], vals["z"], vals["w"]
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def cloud_width(txt: str) -> int | None:
    for line in txt.splitlines():
        if line.strip().startswith("width:"):
            try:
                return int(line.split(":")[1].strip())
            except ValueError:
                return None
    return None


def tf_ok(a: str, b: str) -> bool:
    r = bash_ros(
        f"timeout 6 ros2 run tf2_ros tf2_echo {a} {b} 2>&1 | head -20",
        timeout=10,
    )
    out = (r.stdout or "") + (r.stderr or "")
    return "Translation:" in out and "Invalid frame ID" not in out.split("Translation:")[-1][:80]


def port_9100() -> bool:
    r = subprocess.run(["ss", "-ltn"], capture_output=True, text=True)
    return ":9100" in (r.stdout or "")


def proc_has(needle: str) -> bool:
    r = subprocess.run(["pgrep", "-af", needle], capture_output=True, text=True)
    return any(needle in ln and "pgrep" not in ln for ln in (r.stdout or "").splitlines())


def check_core_topics() -> bool:
    good = True
    for t in (
        "/rslidar_points",
        "/lidar_odometry/deskewed_scan_points",
        "/lidar_odometry/localmap_points",
        "/lidar_odometry/pose",
        "/tf",
        "/tf_static",
        "/wheel_odom",
    ):
        if topic_once(t):
            ok(f"topic has data: {t}")
        else:
            bad(f"no data: {t}")
            good = False
    return good


def check_tf() -> bool:
    good = True
    for a, b in (("map", "base_link"), ("base_link", "rslidar"), ("map", "odom")):
        if tf_ok(a, b):
            ok(f"TF {a} -> {b}")
        else:
            bad(f"TF {a} -> {b}")
            good = False
    return good


def check_pose_stream(wait_s: float = 2.2) -> None:
    t1 = echo_text("/lidar_odometry/pose")
    y1 = yaw_from_echo(t1)
    time.sleep(wait_s)
    t2 = echo_text("/lidar_odometry/pose")
    y2 = yaw_from_echo(t2)
    if y1 is None or y2 is None:
        bad("cannot parse /lidar_odometry/pose yaw")
        return
    dy = abs(((y2 - y1 + 180) % 360) - 180)
    ok(f"pose yaw {y1:.1f}° then {y2:.1f}°  (|Δ|={dy:.2f}° in {wait_s:.1f}s)")
    if dy < 0.05:
        info("yaw almost unchanged — robot is likely standing still (normal before you rotate)")
    else:
        info("yaw is changing — localization is updating while the robot moves")


def check_localmap() -> None:
    w1 = cloud_width(echo_text("/lidar_odometry/localmap_points"))
    time.sleep(2.0)
    w2 = cloud_width(echo_text("/lidar_odometry/localmap_points"))
    if w1 is None:
        bad("LocalMap has no width")
        return
    ok(f"LocalMap width {w1} then {w2}")
    if w1 == w2:
        info("LocalMap size unchanged — expected at rest; it grows on mapping keyframes while you rotate")
    else:
        info("LocalMap size changed — mapping is inserting keyframes")


def stage_a1() -> bool:
    print("\n==== A1 静止环境感知 预检 ====", flush=True)
    info("本脚本不会开车、不会发 stick")
    g = True
    if not proc_has("rslidar_sdk"):
        bad("rslidar_sdk not running")
        g = False
    else:
        ok("rslidar_sdk running")
    if not proc_has("mola-cli"):
        bad("MOLA mola-cli not running")
        g = False
    else:
        ok("MOLA running")
    g = check_core_topics() and g
    g = check_tf() and g
    check_pose_stream(1.6)
    check_localmap()
    info(f"RViz 配置: {RVIZ}")
    info("下一步: bash ppt_materials/scripts/start_rviz_ppt.sh")
    info("画面确认: Fixed Frame=map；能看见墙 + 坐标轴(机器人) + 米色 LocalMap")
    info("录 12–15 秒，车保持静止")
    return g


def stage_a2b1() -> bool:
    print("\n==== A2+B1 旋转扫描+定位建图 预检（不自动旋转） ====")
    info("不要用本脚本开车。你自己用网页/手柄慢转。")
    g = stage_a1()
    info("旋转开始后请再跑: python3 ppt_materials/scripts/preflight.py --stage a2b1")
    info("期望: yaw 持续变化；LocalMap 点数可能增加（关键帧，不是每帧）")
    info("录 20–25 秒: 静止3秒 → 你手动慢转约8–10秒 → 停车2秒")
    return g


def stage_c1c2() -> bool:
    print("\n==== C1+C2 人工遥控 预检（不发送速度） ====")
    g = True
    if port_9100():
        ok("TCP :9100 is listening")
    else:
        bad("TCP :9100 not listening — start run_local.py --daemon")
        g = False
    if proc_has("run_local.py"):
        ok("run_local.py is running")
    else:
        bad("run_local.py not found")
        g = False
    if proc_has("publish_wheel_odom.py"):
        ok("wheel_odom node running")
    else:
        bad("publish_wheel_odom.py not running")
        g = False
    if topic_once("/wheel_odom"):
        ok("/wheel_odom has data")
    else:
        bad("/wheel_odom no data")
        g = False
    info("本脚本故意不发送 stick/速度")
    info("C1: 你手动原地慢转 8–10 秒（网页 WASD 的 A/D 或摇杆）")
    info("C2: 你手动慢速前进 3–4 秒后立刻松键停车")
    info("录制用手机拍车，可与 RViz 分屏")
    return g


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", choices=["a1", "a2b1", "c1c2", "all"], default="all")
    args = ap.parse_args()
    stages = {
        "a1": stage_a1,
        "a2b1": stage_a2b1,
        "c1c2": stage_c1c2,
    }
    results = {}
    if args.stage == "all":
        results["a1"] = stage_a1()
        results["c1c2"] = stage_c1c2()
        info("A2+B1 与 A1 共用同一套 topic/TF；开车请你手动做")
    else:
        results[args.stage] = stages[args.stage]()
    (HERE / "live_status.json").write_text(
        json.dumps({"stage": args.stage, "pass": results}, indent=2) + "\n"
    )
    if all(results.values()):
        print("\n预检通过。可以开始按 RECORDING_STEPS.txt 录制。")
        return 0
    print("\n预检有 FAIL。先看上面的 [FAIL]，不要开始录。")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
