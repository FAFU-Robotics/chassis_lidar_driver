"""Failsafe for network latency / disconnect / power loss on the main mission.

断网不是布尔开关，而是延迟的极限：RTT 变大、报文变稀、最后 TCP 断开。
遥控（kb / ``move``）的速度指令走这条有延迟的通道；主线任务
（find_object / goto / 回放）的控制环在 Jetson 本地，云端只下发一次性目标。

因此保护必须按「通道角色」分层，而不是「一掉线就停车」或「一掉线就回家」：

    1. 过期指令闸（延迟的直接危害）
       高延迟下「松开键之后才到的 move」会让车继续跑；更糟的是迟到的
       ``move`` 会取消正在跑的 find_object。闸门按指令类型设 TTL，
       ``estop`` / ``cancel`` 永不因过期拒绝。
    2. 链路分级（healthy / degraded / lost）
       用最近一次收包年龄 + ping RTT 判定。degraded：丢掉遥控、任务继续。
       lost：等同 WebSocket 断开（停车仅当没有本地自主环）。
    3. 电力（仍在运行 vs 已经重启）
       进程还活着、里程还有效 → 欠压时中止对接并本地返航。
       进程已经死过（断电重启）→ 里程原点丢失，禁止自动继续开，
       只恢复检查点给云端看，等操作员 ``c`` / 新 ``fo``。
    4. 物理层（软件做不到的）
       延迟 2 秒的软件急停救不了正在撞墙的车；底盘物理急停、遥控器、
       Jetson 死机后底盘收不到 0x111 的失联保护，才是硬实时。
"""

from __future__ import annotations

import json
import logging
import os
import tempfile
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Optional

logger = logging.getLogger(__name__)

CHECKPOINT_NAME: str = "failsafe_checkpoint.json"

# 任务链仍在「开着」的状态：掉电重启后不得自动续开。
ACTIVE_MISSION_STATUSES: frozenset[str] = frozenset({
    "recon", "found", "navigating", "approaching", "ready",
    "holding", "grasped", "returning",
})

# 安全指令：过期也执行（宁可不该停也停，也不该停没停）。
NEVER_STALE_ACTIONS: frozenset[str] = frozenset({
    "estop", "cancel", "query", "autonav_map",
})

# 遥控速度：TTL 必须短于人的反应，否则延迟通道会把「已松开」变成继续开。
MOVE_ACTIONS: frozenset[str] = frozenset({"move"})

# 一次性启动类：迟到数秒的 fo/goto 会在错误地点开工。
START_ACTIONS: frozenset[str] = frozenset({
    "find_object", "goto", "go_home", "track_follow", "track_record",
    "task_submit", "odom_reset", "lidar_on", "lidar_off",
    "map_upload", "pose_align", "map_return",
    "autonav_goto", "autonav_select_map", "autonav_cancel",
})


class LinkClass(str, Enum):
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    LOST = "lost"


class PowerClass(str, Enum):
    OK = "ok"
    WARN = "warn"
    CRITICAL = "critical"


@dataclass
class FailsafePolicy:
    """Tunable thresholds. Defaults are conservative for cave / RF interference."""

    # --- command TTL (seconds); envelope ts is Unix ms ---
    # kb 长按刷新必须明显快于此值（mock_cloud.KB_REFRESH_S），
    # 否则保活包一排队就会被拒，表现为遥控「信号丢失」。
    move_ttl_s: float = 0.80
    start_ttl_s: float = 5.0
    # 车侧时钟比云端慢时 age 为负；比云端快时 age 偏大。允许 ± 这么多。
    clock_skew_s: float = 2.0
    # 缺 ts 的旧云端：healthy 下放过（兼容），degraded 下拒绝 move/start。

    # --- link quality ---
    rtt_degraded_s: float = 0.50
    silence_degraded_s: float = 3.0
    silence_lost_s: float = 12.0

    # --- power (SOC 优先；无 BMS 才看电压) ---
    soc_warn_percent: float = 20.0
    soc_critical_percent: float = 12.0
    voltage_warn_v: float = 22.5
    voltage_critical_v: float = 21.5

    checkpoint_period_s: float = 2.0

    @classmethod
    def from_env(cls) -> FailsafePolicy:
        """Override defaults from BUNKER_FAILSAFE_* environment variables."""

        def _f(name: str, default: float) -> float:
            raw = os.environ.get(name)
            if raw is None or raw == "":
                return default
            try:
                return float(raw)
            except ValueError:
                return default

        return cls(
            move_ttl_s=_f("BUNKER_FAILSAFE_MOVE_TTL", 0.80),
            start_ttl_s=_f("BUNKER_FAILSAFE_START_TTL", 5.0),
            clock_skew_s=_f("BUNKER_FAILSAFE_CLOCK_SKEW", 2.0),
            rtt_degraded_s=_f("BUNKER_FAILSAFE_RTT_DEGRADED", 0.50),
            silence_degraded_s=_f("BUNKER_FAILSAFE_SILENCE_DEGRADED", 3.0),
            silence_lost_s=_f("BUNKER_FAILSAFE_SILENCE_LOST", 12.0),
            soc_warn_percent=_f("BUNKER_FAILSAFE_SOC_WARN", 20.0),
            soc_critical_percent=_f("BUNKER_FAILSAFE_SOC_CRIT", 12.0),
            voltage_warn_v=_f("BUNKER_FAILSAFE_V_WARN", 22.5),
            voltage_critical_v=_f("BUNKER_FAILSAFE_V_CRIT", 21.5),
            checkpoint_period_s=_f("BUNKER_FAILSAFE_CKPT_S", 2.0),
        )


@dataclass
class CommandVerdict:
    accept: bool
    reason: str = ""
    age_s: Optional[float] = None


def command_age_s(ts_ms: Optional[int], now: Optional[float] = None,
                  clock_skew_s: float = 2.0) -> Optional[float]:
    """Age of an envelope timestamp. None = cloud omitted ts (legacy).

    Negative age (cloud clock ahead) is clamped to 0 after allowing
    ``clock_skew_s``. Huge negative values are treated as missing.
    """
    if not ts_ms:
        return None
    try:
        ts = int(ts_ms)
    except (TypeError, ValueError):
        return None
    if ts <= 0:
        return None
    now = time.time() if now is None else now
    age = now - (ts / 1000.0)
    if age < -max(clock_skew_s * 4.0, 8.0):
        return None
    if age < 0.0:
        return 0.0
    return age


def ttl_for_action(action: str, policy: FailsafePolicy) -> Optional[float]:
    """TTL in seconds, or None = never reject for staleness."""
    if action in NEVER_STALE_ACTIONS:
        return None
    if action in MOVE_ACTIONS:
        return policy.move_ttl_s
    if action in START_ACTIONS:
        return policy.start_ttl_s
    return policy.start_ttl_s


def judge_command(
    action: str,
    ts_ms: Optional[int],
    link: LinkClass,
    policy: FailsafePolicy,
    now: Optional[float] = None,
) -> CommandVerdict:
    """Accept or reject one inbound cloud command.

    规则（按优先级）::

      * estop / cancel / query → 永远接受（安全通道）。
      * link=lost → 拒绝 move/start（半开连接上的迟到包）；安全通道仍接受。
      * 有 ts 且 age > TTL → 拒绝（这是延迟的直接定义）。
      * 无 ts 且 link=degraded → 拒绝 move/start（无法证明新鲜）。
      * 其余接受。
    """
    age = command_age_s(ts_ms, now=now, clock_skew_s=policy.clock_skew_s)
    if action in NEVER_STALE_ACTIONS:
        return CommandVerdict(True, "safety_channel", age)

    if link is LinkClass.LOST:
        return CommandVerdict(False, "link_lost", age)

    ttl = ttl_for_action(action, policy)
    if ttl is None:
        return CommandVerdict(True, "no_ttl", age)

    if age is None:
        if link is LinkClass.DEGRADED:
            return CommandVerdict(False, "degraded_no_ts", None)
        return CommandVerdict(True, "legacy_no_ts", None)

    if age > ttl:
        return CommandVerdict(
            False, f"stale age={age:.2f}s ttl={ttl:.2f}s", age)
    return CommandVerdict(True, "fresh", age)


def classify_link(
    *,
    rx_age_s: Optional[float],
    rtt_s: Optional[float],
    policy: FailsafePolicy,
) -> LinkClass:
    """Map silence + RTT onto a three-level link class.

    ``rx_age_s is None`` means we have never received a cloud frame on
    this session — treat as lost only after silence_lost (connecting).
    """
    if rx_age_s is not None and rx_age_s >= policy.silence_lost_s:
        return LinkClass.LOST
    degraded = False
    if rx_age_s is not None and rx_age_s >= policy.silence_degraded_s:
        degraded = True
    if rtt_s is not None and rtt_s >= policy.rtt_degraded_s:
        degraded = True
    return LinkClass.DEGRADED if degraded else LinkClass.HEALTHY


def classify_power(
    *,
    soc_percent: Optional[float] = None,
    voltage_v: Optional[float] = None,
    undervoltage_fault: bool = False,
    undervoltage_warning: bool = False,
    policy: Optional[FailsafePolicy] = None,
) -> PowerClass:
    """Power class from BMS SOC (preferred), chassis flags, then voltage.

    ``soc_percent is None`` / ``voltage_v`` near 0 = 传感器还没报到，不当
    作空电（上电瞬间否则会误触发返航）。
    """
    p = policy or FailsafePolicy()
    if undervoltage_fault:
        return PowerClass.CRITICAL
    if soc_percent is not None and soc_percent > 0.0:
        if soc_percent <= p.soc_critical_percent:
            return PowerClass.CRITICAL
        if soc_percent <= p.soc_warn_percent or undervoltage_warning:
            return PowerClass.WARN
        return PowerClass.OK
    if undervoltage_warning:
        return PowerClass.WARN
    if voltage_v is not None and voltage_v > 5.0:
        if voltage_v <= p.voltage_critical_v:
            return PowerClass.CRITICAL
        if voltage_v <= p.voltage_warn_v:
            return PowerClass.WARN
    return PowerClass.OK


def should_return_on_power(mission_status: Optional[str],
                           power: PowerClass) -> bool:
    """True when the still-running process should abort and return home.

    只在 CRITICAL 且任务仍在「离开起点」的阶段触发。已经在 returning /
    终态则不再插一脚。没有任务时由调用方选择停车而不是返航
    （没有探路轨迹，乱 go_home 会用地图原点当家）。
    """
    if power is not PowerClass.CRITICAL:
        return False
    if not mission_status:
        return False
    return mission_status in ACTIVE_MISSION_STATUSES and mission_status not in (
        "returning",
    )


@dataclass
class FailsafeMonitor:
    """Mutable runtime state for link / power / checkpoints. Thread-safe."""

    policy: FailsafePolicy = field(default_factory=FailsafePolicy)
    checkpoint_path: Optional[Path] = None

    def __post_init__(self) -> None:
        self._lock = threading.Lock()
        self._last_rx_mono: Optional[float] = None
        self._last_rtt_s: Optional[float] = None
        self._last_checkpoint_mono: float = 0.0
        self._last_reject: str = ""
        self._boot_checkpoint: Optional[dict[str, Any]] = None
        self._power_return_started: bool = False
        self._last_power: PowerClass = PowerClass.OK
        self._last_link: LinkClass = LinkClass.LOST

    # -- observations ---------------------------------------------------

    def note_rx(self) -> None:
        with self._lock:
            self._last_rx_mono = time.monotonic()

    def note_pong(self, rtt_s: float) -> None:
        if rtt_s < 0.0 or rtt_s > 30.0:
            return
        with self._lock:
            self._last_rx_mono = time.monotonic()
            self._last_rtt_s = rtt_s

    def reset_session(self) -> None:
        """New WebSocket session: RTT/silence start fresh; power flags stay."""
        with self._lock:
            self._last_rx_mono = time.monotonic()
            self._last_rtt_s = None
            self._last_link = LinkClass.HEALTHY

    def mark_lost(self) -> None:
        with self._lock:
            self._last_link = LinkClass.LOST

    # -- queries --------------------------------------------------------

    def snapshot(self) -> dict[str, Any]:
        link, rx_age, rtt = self.link_state()
        with self._lock:
            power = self._last_power
            reject = self._last_reject
            boot = self._boot_checkpoint
        return {
            "link": link.value,
            "rxAgeS": None if rx_age is None else round(rx_age, 2),
            "rttMs": None if rtt is None else int(round(rtt * 1000.0)),
            "power": power.value,
            "lastReject": reject or None,
            "bootInterrupt": bool(
                boot and (boot.get("mission") or {}).get("status")
                in ACTIVE_MISSION_STATUSES
            ),
        }

    def link_state(self) -> tuple[LinkClass, Optional[float], Optional[float]]:
        with self._lock:
            last_rx = self._last_rx_mono
            rtt = self._last_rtt_s
        rx_age = None if last_rx is None else (time.monotonic() - last_rx)
        link = classify_link(
            rx_age_s=rx_age, rtt_s=rtt, policy=self.policy)
        with self._lock:
            self._last_link = link
        return link, rx_age, rtt

    def judge(self, action: str, ts_ms: Optional[int]) -> CommandVerdict:
        link, _, _ = self.link_state()
        verdict = judge_command(action, ts_ms, link, self.policy)
        if not verdict.accept:
            with self._lock:
                self._last_reject = f"{action}: {verdict.reason}"
            logger.warning("Failsafe rejected cmd %s (%s)", action, verdict.reason)
        return verdict

    def observe_power(
        self,
        *,
        soc_percent: Optional[float] = None,
        voltage_v: Optional[float] = None,
        undervoltage_fault: bool = False,
        undervoltage_warning: bool = False,
    ) -> PowerClass:
        power = classify_power(
            soc_percent=soc_percent,
            voltage_v=voltage_v,
            undervoltage_fault=undervoltage_fault,
            undervoltage_warning=undervoltage_warning,
            policy=self.policy,
        )
        with self._lock:
            self._last_power = power
        return power

    @property
    def power_return_started(self) -> bool:
        with self._lock:
            return self._power_return_started

    def begin_power_return(self) -> bool:
        """First caller wins; subsequent ticks no-op."""
        with self._lock:
            if self._power_return_started:
                return False
            self._power_return_started = True
            return True

    def clear_power_return(self) -> None:
        with self._lock:
            self._power_return_started = False

    # -- checkpoint -----------------------------------------------------

    def load_boot_checkpoint(self) -> Optional[dict[str, Any]]:
        data = load_checkpoint(self.checkpoint_path)
        self._boot_checkpoint = data
        return data

    @property
    def boot_checkpoint(self) -> Optional[dict[str, Any]]:
        return self._boot_checkpoint

    def maybe_save(self, payload: dict[str, Any], *, force: bool = False,
                   reason: str = "periodic") -> None:
        path = self.checkpoint_path
        if path is None:
            return
        now = time.monotonic()
        with self._lock:
            if not force and (now - self._last_checkpoint_mono) < self.policy.checkpoint_period_s:
                return
            self._last_checkpoint_mono = now
        payload = dict(payload)
        payload.setdefault("reason", reason)
        payload.setdefault("saved_at", time.strftime("%Y-%m-%dT%H:%M:%S"))
        try:
            atomic_write_json(path, payload)
        except Exception:
            logger.debug("failsafe checkpoint write failed", exc_info=True)

    def clear_checkpoint(self) -> None:
        path = self.checkpoint_path
        if path is None:
            return
        try:
            if path.is_file():
                path.unlink()
        except OSError:
            logger.debug("failsafe checkpoint unlink failed", exc_info=True)


def atomic_write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=".failsafe_", suffix=".json",
                               dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            json.dump(data, fh, ensure_ascii=False, indent=2)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, path)
    except Exception:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def load_checkpoint(path: Optional[Path]) -> Optional[dict[str, Any]]:
    if path is None or not path.is_file():
        return None
    try:
        with path.open("r", encoding="utf-8") as fh:
            data = json.load(fh)
    except (OSError, json.JSONDecodeError):
        logger.warning("failsafe checkpoint unreadable: %s", path)
        return None
    return data if isinstance(data, dict) else None


def checkpoint_path_for(track_dir: str) -> Path:
    return Path(track_dir) / CHECKPOINT_NAME
