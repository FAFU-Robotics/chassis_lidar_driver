#!/usr/bin/env python3
"""导航精度量化评估 —— 纯里程航迹推算（dead reckoning）闭环仿真。

用途：在不接底盘/雷达的情况下，量化 :class:`bunker_mini.navigator.Navigator`
的「点到点导航精度」：给定目标点，用差速运动学模型闭环回喂里程，
统计**终点位置误差 / 航向误差 / 行程 / 耗时**，并支持注入轮滑/打滑
噪声，观察纯里程定位在月球松软地面（高打滑）下精度如何退化。

这直接对应「建图完成前的导航精度基线」：若纯里程误差不可接受，即可
判定需要外部位姿源（SLAM/视觉 ``pose_source``）做闭环校正。

用法::

    python3 examples/nav_accuracy_eval.py                 # 默认无噪声基线
    python3 examples/nav_accuracy_eval.py --slip 0.05     # 5% 轮滑噪声
    python3 examples/nav_accuracy_eval.py --slip 0.15     # 15% 打滑（松软月壤）

输出：每条任务一行，末尾给「全部任务平均误差」。均无第三方依赖。
"""

from __future__ import annotations

import argparse
import math
import random
import sys
import time
from dataclasses import dataclass, field

from _bootstrap import ensure_project_root

ensure_project_root()

from bunker_mini.navigator import NavigateConfig, Navigator  # noqa: E402


@dataclass
class Mission:
    name: str
    goal: tuple[float, float]
    waypoints: list[tuple[float, float]] = field(default_factory=list)


MISSIONS: list[Mission] = [
    Mission("直线 1.0m", (1.0, 0.0)),
    Mission("斜向 1.4m", (1.0, 1.0)),
    Mission("直角 90°", (0.0, 1.0)),
    Mission("直线 3.0m", (3.0, 0.0)),
    Mission("多航点折线", (2.0, 1.0), waypoints=[(1.0, 0.0), (1.5, 0.5)]),
]


@dataclass
class RunResult:
    mission: str
    goal: tuple[float, float]
    arrived: bool
    pos_err_m: float
    yaw_err_deg: float
    path_m: float
    duration_s: float


class FakeCtrl:
    """记录最近一次速度指令，并输出底盘运动学所需的 (v, w)。"""

    def __init__(self) -> None:
        self.v: float = 0.0
        self.w: float = 0.0
        self.stopped: bool = False

    def set_velocity(self, v: float, w: float) -> None:
        self.v, self.w = v, w

    def stop_motion(self) -> None:
        self.v = self.w = 0.0
        self.stopped = True


def _wrap_deg(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


def run_mission(
    mission: Mission,
    *,
    wheelbase_m: float,
    slip: float,
    dt: float,
    max_time_s: float,
) -> RunResult:
    ctrl = FakeCtrl()
    cfg = NavigateConfig(
        max_linear_m_s=0.3,
        max_angular_rad_s=0.6,
        goal_tolerance_m=0.1,
        update_interval_s=dt,
        stall_timeout_s=max_time_s,
    )
    nav = Navigator(ctrl, guard=None, wheelbase_m=wheelbase_m, config=cfg)
    nav.reset_pose()

    arrived: list[bool] = []
    ok = nav.goto(mission.goal[0], mission.goal[1],
                  waypoints=mission.waypoints or None,
                  on_arrived=lambda: arrived.append(True))
    if not ok:
        nav.stop()
        return RunResult(mission.name, mission.goal, False, -1.0, -1.0, -1.0, -1.0)

    # 差速运动学闭环：把当前指令 (v, w) 积分为轮里程回喂（可注入打滑）
    left_mm = right_mm = 0.0
    path_m = 0.0
    t0 = time.monotonic()
    rng = random.Random(0)
    while not arrived and (time.monotonic() - t0) < max_time_s:
        v, w = ctrl.v, ctrl.w
        d_center = v * dt
        d_yaw = w * dt
        # 差速逆解：d_center=(dl+dr)/2, d_yaw=(dr-dl)/wheelbase
        dl = d_center - d_yaw * wheelbase_m / 2.0
        dr = d_center + d_yaw * wheelbase_m / 2.0
        path_m += abs(d_center)
        # 注入轮滑/打滑噪声（左右轮独立随机比例，模拟松软月面）
        if slip > 0:
            dl *= 1.0 + rng.uniform(-slip, slip)
            dr *= 1.0 + rng.uniform(-slip, slip)
        left_mm += dl * 1000.0
        right_mm += dr * 1000.0
        nav.feed_odometry(int(left_mm), int(right_mm))
        time.sleep(dt * 0.5)

    duration_s = time.monotonic() - t0
    nav.stop()
    p = nav.pose
    pos_err = math.hypot(p.x - mission.goal[0], p.y - mission.goal[1])
    # 航向误差 = 最终航向 vs 「起点→终点连线」方向（goto 为位置型导航，
    # 最终朝向应大体指向目标方向，供机械臂交接/视觉伺服参考）
    expected_yaw = math.degrees(math.atan2(mission.goal[1], mission.goal[0]))
    yaw_err = abs(_wrap_deg(p.yaw_deg - expected_yaw))
    return RunResult(
        mission.name, mission.goal, bool(arrived), pos_err, yaw_err,
        path_m, duration_s,
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="导航精度量化评估（差速运动学闭环）")
    ap.add_argument("--wheelbase", type=float, default=0.5,
                    help="轮距 m（BUNKER MINI 2.0 约 0.5）")
    ap.add_argument("--slip", type=float, default=0.0,
                    help="轮滑/打滑噪声比例 0~1（0=无噪声）")
    ap.add_argument("--dt", type=float, default=0.02, help="仿真步长 s")
    ap.add_argument("--max-time", type=float, default=20.0, help="单任务超时 s")
    args = ap.parse_args()

    print("=" * 78)
    print(f"导航精度量化评估  wheelbase={args.wheelbase}m  slip={args.slip*100:.0f}%")
    print("=" * 78)
    hdr = f"{'任务':<14}{'目标':>10}{'到达':>5}{'位置误差(m)':>12}" \
          f"{'航向误差(°)':>12}{'行程(m)':>9}{'耗时(s)':>9}"
    print(hdr)
    print("-" * 78)

    results: list[RunResult] = []
    for m in MISSIONS:
        r = run_mission(m, wheelbase_m=args.wheelbase, slip=args.slip,
                        dt=args.dt, max_time_s=args.max_time)
        results.append(r)
        goal = f"({m.goal[0]:.1f},{m.goal[1]:.1f})"
        if r.arrived:
            print(f"{r.mission:<14}{goal:>10}{'Y':>5}{r.pos_err_m:>12.3f}"
                  f"{r.yaw_err_deg:>12.2f}{r.path_m:>9.3f}{r.duration_s:>9.2f}")
        else:
            print(f"{r.mission:<14}{goal:>10}{'N':>5}{'—':>12}{'—':>12}"
                  f"{'—':>9}{'—':>9}")
    print("-" * 78)

    arrived_rs = [r for r in results if r.arrived]
    if arrived_rs:
        avg_pos = sum(r.pos_err_m for r in arrived_rs) / len(arrived_rs)
        avg_yaw = sum(r.yaw_err_deg for r in arrived_rs) / len(arrived_rs)
        print(f"平均位置误差: {avg_pos:.3f} m   平均航向误差: {avg_yaw:.2f}° "
              f"(到达 {len(arrived_rs)}/{len(results)})")
    else:
        print("无任务到达（打滑过大或超时）")
    print("=" * 78)
    print("提示：短距离 + 对称打滑下纯里程尚可用，但误差随距离/不对称打滑累积；")
    print("建图完成后接入 pose_source（SLAM/视觉位姿）做闭环校正可消除累积漂移。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
