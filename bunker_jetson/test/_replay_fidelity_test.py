#!/usr/bin/env python3
"""Playback fidelity tests for the trajectory replay fixes.

覆盖本轮修复的三个核心点：
  1. 慢速但仍在推进的回放不会被固定预算截断（动态航点预算 + 全局里程停滞检测）；
  2. 真正卡死（里程/轮速都无推进）时回放标记不完整，`arrived` 不再误报——
     这是 fb 往返「残缺路径仍触发回程、回不到起点」的根因；
  3. 反向回放（fb 回程 / 倒放）按里程精确覆盖全程，回到起点的里程基准一致。

用法：
    python3 -u _replay_fidelity_test.py
"""

import sys
import threading
import time

from bunker_mini.navigator import Pose2D
from bunker_mini.protocol import OdometerFeedback
from bunker_mini.tracker import (
    Track,
    TrackPlayer,
    Waypoint,
)


def _odo(l, r):
    return OdometerFeedback(left_wheel_mm=l, right_wheel_mm=r)


class _Ctrl:
    """Controller fake that feeds odometer/motion on demand."""

    def __init__(self) -> None:
        self._odo = _odo(0, 0)
        self._motion = type("M", (), {
            "linear_velocity_m_s": 0.0, "angular_velocity_rad_s": 0.0,
        })()
        self.cmds: list[tuple[float, float]] = []
        self._odo_lock = threading.Lock()
        self._feed_thread: threading.Thread | None = None
        self._feed_stop = threading.Event()

    @property
    def latest_odometer(self):
        with self._odo_lock:
            return self._odo

    @property
    def latest_motion(self):
        return self._motion

    def set_odometer(self, l: int, r: int) -> None:
        with self._odo_lock:
            self._odo = _odo(l, r)

    def set_motion(self, v: float, w: float) -> None:
        self._motion = type("M", (), {
            "linear_velocity_m_s": v, "angular_velocity_rad_s": w,
        })()

    def set_velocity(self, v: float, w: float) -> None:
        self.cmds.append((v, w))

    def stop_motion(self) -> None:
        self.set_velocity(0.0, 0.0)

    def feed_odometer(self, step_mm: int = 10, interval_s: float = 0.05):
        """Continuously advance the odometer (simulates a moving chassis)."""
        def _run():
            cur = self._odo.left_wheel_mm
            while not self._feed_stop.wait(interval_s):
                cur += step_mm
                self.set_odometer(cur, cur)
        self._feed_stop.clear()
        self._feed_thread = threading.Thread(target=_run, daemon=True)
        self._feed_thread.start()

    def stop_feed(self) -> None:
        self._feed_stop.set()


def _straight_track(dist_mm: int = 600, v: float = 0.3) -> Track:
    """A straight track whose odometer readings grow by 100mm/waypoint."""
    n = max(2, dist_mm // 100 + 1)
    wps = [
        Waypoint(t=round(i / (n - 1), 3), left_mm=i * 100, right_mm=i * 100,
                 v=v, w=0.0)
        for i in range(n)
    ]
    return Track(name="fidelity", created_at="t", total_duration_s=1.0,
                 waypoints=wps)


_PASS = 0
_FAIL = 0


def check(label: str, cond: bool, detail: str = "") -> None:
    global _PASS, _FAIL
    if cond:
        _PASS += 1
        print(f"  PASS  {label}" + (f"  ({detail})" if detail else ""))
    else:
        _FAIL += 1
        print(f"  FAIL  {label}" + (f"  ({detail})" if detail else ""))


def test_slow_but_moving_not_truncated() -> None:
    """慢速推进不被截断：500mm 全程必须完整覆盖，complete=True。"""
    print("\n[1] 慢速但持续推进的回放不被截断")
    ctrl = _Ctrl()
    player = TrackPlayer(ctrl)
    track = _straight_track(dist_mm=500, v=0.3)
    done: list[bool] = []
    err: list[BaseException] = []

    def play():
        try:
            done.append(player.play(track))
        except BaseException as e:  # noqa: BLE001
            err.append(e)

    t = threading.Thread(target=play, daemon=True)
    t.start()
    # 慢速推进：每 50ms 走 5mm（远慢于录制速度，旧固定预算 2s/航点会截断）
    ctrl.feed_odometer(step_mm=5, interval_s=0.05)
    t.join(timeout=15.0)
    ctrl.stop_feed()
    check("回放返回", not err and len(done) == 1, f"err={err}")
    if not err and done:
        check("完整覆盖 complete=True", done[0] is True,
              f"total cmds={len(ctrl.cmds)}")
    # 最后一跳必须把里程推进到 ≥500mm
    last_odo = ctrl.latest_odometer
    check("里程覆盖 ≥500mm", last_odo.left_wheel_mm >= 500,
          f"left={last_odo.left_wheel_mm}mm")


def test_genuine_stall_reports_incomplete() -> None:
    """真卡死：里程不再推进 → 回放标记不完整（complete=False）。"""
    print("\n[2] 真正卡死时回放标记不完整")
    ctrl = _Ctrl()
    player = TrackPlayer(ctrl)
    track = _straight_track(dist_mm=400, v=0.3)
    done: list[bool] = []
    err: list[BaseException] = []

    def play():
        try:
            done.append(player.play(track))
        except BaseException as e:  # noqa: BLE001
            err.append(e)

    t = threading.Thread(target=play, daemon=True)
    t.start()
    # 先推进一点（模拟走了 100mm 后被墙挡住），然后完全停住
    ctrl.feed_odometer(step_mm=10, interval_s=0.02)
    time.sleep(0.6)
    ctrl.stop_feed()
    # 停住后不再有任何推进
    t.join(timeout=15.0)
    check("回放返回", not err and len(done) == 1, f"err={err}")
    if not err and done:
        check("卡死回放 complete=False", done[0] is False)
    last_odo = ctrl.latest_odometer
    check("里程停在 ~100mm（未伪报完成）", last_odo.left_wheel_mm < 400,
          f"left={last_odo.left_wheel_mm}mm")


def test_reverse_covers_full_distance() -> None:
    """反向回放：delta 为负，回程覆盖完整里程（fb 回程基准一致）。"""
    print("\n[3] 反向回放按里程精确覆盖全程")
    ctrl = _Ctrl()
    player = TrackPlayer(ctrl)
    track = _straight_track(dist_mm=300, v=0.3)
    rev = track.reversed()
    # 起点里程为 1000mm（模拟正向回放结束时里程计位置）
    ctrl.set_odometer(1000, 1000)
    done: list[bool] = []
    err: list[BaseException] = []

    def play():
        try:
            done.append(player.play(rev))
        except BaseException as e:  # noqa: BLE001
            err.append(e)

    t = threading.Thread(target=play, daemon=True)
    t.start()
    # 反向推进：里程递减 10mm/50ms
    def _run():
        while not ctrl._feed_stop.wait(0.05):
            with ctrl._odo_lock:
                cur = ctrl._odo.left_wheel_mm - 10
            ctrl.set_odometer(cur, cur)
    ctrl._feed_stop.clear()
    ctrl._feed_thread = threading.Thread(target=_run, daemon=True)
    ctrl._feed_thread.start()
    t.join(timeout=15.0)
    ctrl.stop_feed()
    check("回放返回", not err and len(done) == 1, f"err={err}")
    if not err and done:
        check("反向完整覆盖 complete=True", done[0] is True)
    last_odo = ctrl.latest_odometer
    # 起点 1000mm - 300mm = 700mm
    check("里程回到 700mm（正向起点基准）", last_odo.left_wheel_mm <= 700,
          f"left={last_odo.left_wheel_mm}mm")
    check("未过度回退（≥680mm）", last_odo.left_wheel_mm >= 680,
          f"left={last_odo.left_wheel_mm}mm")


def test_fixed_budget_no_longer_truncates_slow_replay() -> None:
    """回归：旧版固定 2s/航点预算会把慢速回放截断，现在不会。"""
    print("\n[4] 动态预算——超预算但仍在推进则继续等待")
    ctrl = _Ctrl()
    player = TrackPlayer(ctrl)
    track = _straight_track(dist_mm=200, v=0.2)
    done: list[bool] = []
    err: list[BaseException] = []

    def play():
        try:
            done.append(player.play(track))
        except BaseException as e:  # noqa: BLE001
            err.append(e)

    t = threading.Thread(target=play, daemon=True)
    t.start()
    # 极慢推进：每 100ms 走 3mm —— 单航点 100mm 需要 ~3.3s，远超旧 2s 上限
    ctrl.feed_odometer(step_mm=3, interval_s=0.1)
    t.join(timeout=20.0)
    ctrl.stop_feed()
    check("回放返回", not err and len(done) == 1, f"err={err}")
    if not err and done:
        check("慢速回放仍完整 complete=True", done[0] is True,
              f"total cmds={len(ctrl.cmds)}")
    last_odo = ctrl.latest_odometer
    check("里程覆盖 ≥200mm", last_odo.left_wheel_mm >= 200,
          f"left={last_odo.left_wheel_mm}mm")


def test_odometer_arrives_midway_rebases() -> None:
    """里程中途才到达：重基准只补走剩余里程，不重复计算时间回放段。

    里程计读数是绝对累计值，中途出现时只是「当前物理位置的里程读数」，
    并不代表时间回放段已走了多少。关键性质：重基准后只按「剩余录制距离」
    驱动，而不是把整条轨迹从重基准点再走一遍（否则里程会多出
    base + 全程，即重复计算）。
    """
    print("\n[5] 里程中途到达——重基准只走剩余距离")
    ctrl = _Ctrl()
    player = TrackPlayer(ctrl)
    track = _straight_track(dist_mm=400, v=0.3)
    # 模拟启动时 0x311 缺失：初始 latest_odometer 返回 None，直到 ~1.4s 后
    # 才出现。基准读数取 100mm（绝对里程的任意偏移，非物理位移）。
    state = {"odo_available": False, "cur_mm": 100}

    class _LateCtrl:
        @property
        def latest_odometer(self):
            if not state["odo_available"]:
                return None
            return _odo(state["cur_mm"], state["cur_mm"])

        @property
        def latest_motion(self):
            return ctrl.latest_motion

        def set_velocity(self, v, w):
            ctrl.set_velocity(v, w)

        def stop_motion(self):
            ctrl.stop_motion()

    late_ctrl = _LateCtrl()
    p2 = TrackPlayer(late_ctrl)
    done: list[bool] = []
    err: list[BaseException] = []

    def play():
        try:
            done.append(p2.play(track))
        except BaseException as e:  # noqa: BLE001
            err.append(e)

    t = threading.Thread(target=play, daemon=True)
    t.start()
    # 前 1.4s 无里程（初始 0.5s 等待 + 2~3 个航点的时间回放阶段）
    time.sleep(1.4)
    state["odo_available"] = True
    # 之后里程从 100mm 起持续推进（每 20ms 走 8mm）
    def _run():
        while not ctrl._feed_stop.wait(0.02):
            state["cur_mm"] += 8
    ctrl._feed_stop.clear()
    ctrl._feed_thread = threading.Thread(target=_run, daemon=True)
    ctrl._feed_thread.start()
    t.join(timeout=15.0)
    ctrl.stop_feed()
    check("回放返回", not err and len(done) == 1, f"err={err}")
    if not err and done:
        check("中途切里程后仍完整 complete=True", done[0] is True)
    last_odo = state["cur_mm"]
    # 若未正确重基准（旧 bug）：会以录制起点 first 为参考走完整 400mm，
    # 里程 = 100 + 400 = 500mm；正确重基准后只补走剩余距离，远小于 500。
    check("未重复计算全程（里程 < 460mm）", last_odo < 460,
          f"left={last_odo}mm（旧 bug 会到 ~500mm）")
    # 重基准后应至少补走了一部分剩余里程（> 基准 100mm 再加一些）
    check("剩余部分已补走（里程 > 180mm）", last_odo > 180,
          f"left={last_odo}mm")


def test_interrupted_replay_always_reports() -> None:
    """卡死/残缺回放也必须触发 on_complete(complete=False)。

    这是「r 模式按 B 回程中途卡住 → 云端收不到 arrived → 回程状态一直空等、
    终端卡死」的根因修复。回放无论完整与否都要上报结果。
    """
    print("\n[6] 中断的回放也始终上报 on_complete(False)")
    ctrl = _Ctrl()
    player = TrackPlayer(ctrl)
    track = _straight_track(dist_mm=600, v=0.3)
    report: list[bool | None] = [None]
    err: list[BaseException] = []

    def play():
        try:
            player.play(
                track,
                on_complete=lambda complete: report.__setitem__(0, bool(complete)),
            )
        except BaseException as e:  # noqa: BLE001
            err.append(e)

    t = threading.Thread(target=play, daemon=True)
    t.start()
    # 车完全不动：里程停在 0、无轮速。每个航点 1.5s 停滞后被放弃并继续，
    # 最终走完全部航点返回；on_complete 必须被调用（complete=False）。
    t.join(timeout=90.0)
    ctrl.stop_feed()
    check("回放返回（不卡死）", not err and not t.is_alive(), f"err={err}")
    check("on_complete 已被调用（即使残缺）", report[0] is not None)
    check("残缺回放上报 complete=False", report[0] is False)


def test_frozen_odometer_uses_wheel_speed() -> None:
    """里程帧在但值冻结不更新时，用 0x221 轮速积分继续推进。

    这是「回程只返回一小段就中止」的另一个根因：里程帧一直来但值不动，
    旧逻辑把正在移动的小车误判成停滞，每个航点 1.5s 后放弃、回放提前中止。
    """
    print("\n[7] 里程冻结——轮速积分兜底推进")
    ctrl = _Ctrl()
    ctrl.set_odometer(0, 0)  # 里程永远冻结在 0
    player = TrackPlayer(ctrl)
    track = _straight_track(dist_mm=400, v=0.3)

    def _run():
        while not ctrl._feed_stop.wait(0.02):
            ctrl.set_motion(0.3, 0.0)  # 小车确实在动，但里程值不动
    ctrl._feed_stop.clear()
    ctrl._feed_thread = threading.Thread(target=_run, daemon=True)
    ctrl._feed_thread.start()

    done: list[bool] = []
    err: list[BaseException] = []

    def play():
        try:
            done.append(player.play(track))
        except BaseException as e:  # noqa: BLE001
            err.append(e)

    t = threading.Thread(target=play, daemon=True)
    t.start()
    t.join(timeout=30.0)
    ctrl.stop_feed()
    ctrl.set_motion(0.0, 0.0)
    check("回放返回", not err and len(done) == 1, f"err={err}")
    if not err and done:
        check("里程冻结但轮速在动 → 完整覆盖 complete=True", done[0] is True,
              f"total cmds={len(ctrl.cmds)}")


def test_playback_odo_flag() -> None:
    """里程驱动 vs 时间回放：last_playback_had_odo 标志必须如实反映。

    车侧 agent 用这个标志决定 arrived 文案：
    * 有里程 → "Reverse replay completed — back at start"（真回到起点）
    * 无里程（时间回放）→ "position approximate, may not be exactly back
      at start"（只说明回放跑完了，小车不一定真的回到起点）
    """
    print("\n[8] 里程驱动/时间回放的 last_playback_had_odo 标志")

    # 1) 有里程数据 + 底盘真实反馈 → 里程驱动，标志应为 True
    ctrl = _Ctrl()
    ctrl.set_odometer(0, 0)
    ctrl.feed_odometer(step_mm=10, interval_s=0.03)
    player = TrackPlayer(ctrl)
    track = _straight_track(dist_mm=400, v=0.3)
    done: list[bool] = []
    t = threading.Thread(
        target=lambda: done.append(player.play(track)), daemon=True,
    )
    t.start()
    t.join(timeout=30.0)
    ctrl.stop_feed()
    check("里程驱动回放正常返回", len(done) == 1, f"done={done}")
    if done:
        check("有里程 → complete=True", done[0] is True, f"done={done}")
        check("有里程 → last_playback_had_odo=True",
              player.last_playback_had_odo is True)

    # 2) 轨迹无里程数据（left_mm/right_mm 全 0）→ 时间回放，标志应为 False
    ctrl2 = _Ctrl()
    ctrl2.set_odometer(0, 0)  # 车侧里程始终为 0（0x311 缺失/全 0 的场景）
    player2 = TrackPlayer(ctrl2)
    track2 = _straight_track(dist_mm=400, v=0.3)
    track2.waypoints = [
        Waypoint(t=w.t, left_mm=0, right_mm=0, v=w.v, w=w.w)
        for w in track2.waypoints
    ]
    done2: list[bool] = []
    t2 = threading.Thread(
        target=lambda: done2.append(player2.play(track2)), daemon=True,
    )
    t2.start()
    t2.join(timeout=30.0)
    check("无里程 → 时间回放正常返回", len(done2) == 1, f"done2={done2}")
    if done2:
        check("无里程 → 时间回放 complete=True（跑完即算）", done2[0] is True)
        check("无里程 → last_playback_had_odo=False",
              player2.last_playback_had_odo is False)


def main() -> None:
    test_slow_but_moving_not_truncated()
    test_genuine_stall_reports_incomplete()
    test_reverse_covers_full_distance()
    test_fixed_budget_no_longer_truncates_slow_replay()
    test_odometer_arrives_midway_rebases()
    test_interrupted_replay_always_reports()
    test_frozen_odometer_uses_wheel_speed()
    test_playback_odo_flag()
    print(f"\n===== {_PASS} passed, {_FAIL} failed =====")
    return 1 if _FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
