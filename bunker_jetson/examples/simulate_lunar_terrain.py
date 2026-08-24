#!/usr/bin/env python3
"""Lunar-cave terrain simulator for the RoboSense Airy LiDAR pipeline.

在没有真雷达 / 底盘的情况下，验证「月球溶洞」式复杂路况下的避障与
通过性逻辑。生成合成 MSOP 包，两种用法：

  1. --selftest  直接把合成点云喂给解析器 + 地形剖面，打印各方向
                 通过性判定（无需网络、无需雷达）。
  2. --udp       把合成 MSOP 包发到 UDP 端口，配合 view_lidar.py /
                 run_agent.py 做端到端验证。

地形要素（车体系：雷达光心为原点，x 右、y 前、z 上）：
  * 碎石 gravel      — 密集低矮凸起（1~5 cm，可碾过，应只限速不急停）
  * 大石 rocks       — 高耸石块（0.2~0.5 m，不可通行，应绕行）
  * 台阶 step        — y=1.2 m 处 0.12 m 突变（不可通行，应绕行）
  * 坑 pit           — 半径 0.3 m 凹陷（不可通行）
  * 坡 slope         — 全局坡度（度）（可通行，应降速）
  * 目标 target      — 高反光材料圆柱（模拟贴了反光标记的目标物体，
    用于验证 LiDAR 反射强度目标检测通路）

雷达安装 tilt：Airy 车顶平装时视场向上（0~90°），只能看见高于雷达的
立体障碍（岩壁/钟乳石/高堆石）；要感知地面（碎石/台阶/坑）必须前倾安装
（手册推荐）。--tilt 模拟前倾角，默认 25°。

Usage::

    python examples/simulate_lunar_terrain.py --selftest --seed 7
    python examples/simulate_lunar_terrain.py --selftest --no-gravel --rocks 3
    python examples/simulate_lunar_terrain.py --selftest --no-target
    python examples/simulate_lunar_terrain.py --udp 127.0.0.1:6699 --tilt 25
"""

from __future__ import annotations

import argparse
import math
import os
import random
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bunker_mini.lidar import (
    CHANNEL_VERTICAL_DEG,
    MSOP_PACKET_SIZE,
    LidarPoint,
    parse_msop_packet,
)
from bunker_mini.terrain import DEFAULT_STEP_LIMIT_M, TerrainProfile
from bunker_mini.vision import ReflectivityDetector

# 一包 8 个 block 对应 4 个方位（每个方位 2 个 block：通道 1-48 / 49-96）
_AZ_PER_PACKET = 4

# 地面/岩石的反射强度（玄武岩低反射）
_REFL_GROUND = 110
# 目标高反光材料的反射强度（工程级反光贴纸通常在 200+）
_REFL_TARGET = 230


class LunarTerrain:
    """Synthetic lunar-cave terrain surface (z = f(x, y))."""

    def __init__(
        self,
        *,
        seed: int = 42,
        mount_h: float = 0.35,
        gravel: int = 60,
        rocks: int = 2,
        pit: bool = True,
        slope_deg: float = 0.0,
        step_h: float = 0.12,
        step_y: float = 1.2,
        target: tuple[float, float, float, float, int] | None = (0.5, 1.6, 0.15, 0.5, _REFL_TARGET),
        overhangs: list[tuple[float, float, float, float, float]] | None = None,
        cliff_y: float | None = None,
        ceiling_z: float | None = None,
        ground_refl: int = _REFL_GROUND,
    ) -> None:
        self._rng = random.Random(seed)
        self._mount_h = mount_h
        self._slope = math.tan(math.radians(slope_deg))
        self._step_h = step_h
        self._step_y = step_y

        # 碎石：高斯凸起 (cx, cy, sigma, h)，h 1~5 cm
        self._gravel = []
        for _ in range(gravel):
            self._gravel.append((
                self._rng.uniform(-1.5, 1.5),
                self._rng.uniform(0.1, 2.4),
                self._rng.uniform(0.03, 0.06),
                self._rng.uniform(0.01, 0.05),
            ))

        # 大石：高耸石块 (cx, cy, sigma, h)，h 0.2~0.5 m
        self._rocks = []
        for i in range(rocks):
            self._rocks.append((
                self._rng.choice([-0.6, 0.0, 0.6]),
                self._rng.uniform(0.6, 1.8),
                self._rng.uniform(0.08, 0.15),
                self._rng.uniform(0.2, 0.5),
            ))

        # 坑：凹陷 (cx, cy, radius, depth)
        self._pit = (0.35, 1.5, 0.3, 0.1) if pit else None

        # 目标：高反光圆柱 (tx, ty, radius, height, reflectivity)
        self._target = target
        # 钟乳石/低梁：(cx, cy, radius, z_lo, z_hi)，雷达系 z
        self._overhangs = list(overhangs or [])
        # 悬崖：y 大于该值后不再有地面回波（溶洞口）
        self._cliff_y = cliff_y
        # 溶洞顶板高度（雷达系 z）；None=无顶
        self._ceiling_z = ceiling_z
        self._ground_refl = ground_refl

    def ground_z(self, x: float, y: float) -> float | None:
        if self._cliff_y is not None and y >= self._cliff_y:
            return None
        z = -self._mount_h
        z += self._slope * y
        if y > self._step_y:
            z += self._step_h
        for cx, cy, sigma, h in self._gravel:
            d2 = (x - cx) ** 2 + (y - cy) ** 2
            r2 = (3 * sigma) ** 2
            if d2 < r2:
                z += h * math.exp(-d2 / (2 * sigma * sigma))
        for cx, cy, sigma, h in self._rocks:
            d2 = (x - cx) ** 2 + (y - cy) ** 2
            r = 3 * sigma
            if d2 < r * r:
                # 圆锥形陡边岩块：中心 h，边缘 0（比平滑高斯更接近真实石块，
                # 边缘坡度 h/r 通常 > 1 → 被判定为不可通行）
                z += h * max(0.0, 1.0 - math.sqrt(d2) / r)
        if self._pit is not None:
            cx, cy, r, depth = self._pit
            d2 = (x - cx) ** 2 + (y - cy) ** 2
            if d2 < r * r:
                z -= depth * (1.0 - math.sqrt(d2) / r)
        return z

    def ray_hit(self, az_deg: float, elev_deg: float, max_r: float = 3.0,
                step: float = 0.05) -> tuple[float | None, int]:
        """First horizontal distance where the ray meets a surface, plus reflectivity.

        返回 (水平距离, 反射强度)。先检测高反光目标圆柱（优先命中），
        再检测地面/地形。距离 0.1~3 m，步进 5 cm。
        """
        tan_e = math.tan(math.radians(elev_deg))
        s = math.sin(math.radians(az_deg))
        c = math.cos(math.radians(az_deg))

        # 高反光目标圆柱：射线与垂直圆柱求交（xy 平面内直线到圆）
        if self._target is not None:
            tx, ty, r, h, refl = self._target
            b = -2.0 * (tx * s + ty * c)
            cc = tx * tx + ty * ty - r * r
            disc = b * b - 4.0 * cc
            if disc >= 0.0:
                sq = math.sqrt(disc)
                cands = [(-b - sq) / 2.0, (-b + sq) / 2.0]
                hits = [t for t in cands if t > 0.1]
                if hits:
                    hd = min(hits)
                    z = hd * tan_e
                    gz = self.ground_z(tx, ty)
                    if gz is not None and gz <= z <= gz + h:
                        return hd, refl

        # 钟乳石 / 低梁：垂直圆柱，z 在 [z_lo, z_hi]
        for ox, oy, r, z_lo, z_hi in self._overhangs:
            b = -2.0 * (ox * s + oy * c)
            cc = ox * ox + oy * oy - r * r
            disc = b * b - 4.0 * cc
            if disc < 0.0:
                continue
            sq = math.sqrt(disc)
            hits = [t for t in ((-b - sq) / 2.0, (-b + sq) / 2.0) if t > 0.1]
            if not hits:
                continue
            hd = min(hits)
            z = hd * tan_e
            if z_lo <= z <= z_hi:
                return hd, self._ground_refl

        # 地面 / 地形
        hd = 0.1  # 雷达盲区 0.1 m
        while hd <= max_r:
            x, y = hd * s, hd * c
            z_ray = hd * tan_e
            if self._ceiling_z is not None and z_ray >= self._ceiling_z:
                return hd, self._ground_refl
            gz = self.ground_z(x, y)
            if gz is not None and z_ray <= gz:
                return hd, self._ground_refl
            hd += step
        return None, self._ground_refl


def build_msop_packet(terrain: LunarTerrain, tilt_deg: float,
                      az_start: int,
                      data_map: dict[int, tuple[list[int], list[int]]]) -> bytes:
    """Encode one MSOP packet (4 azimuth columns × 96 channels).

    ``data_map[az] = (distance_raw_list, reflectivity_list)``，每份 96 项，
    distance 0 = 无回波。Fills the 8 data blocks, block pairs share an azimuth.
    """
    data = bytearray(MSOP_PACKET_SIZE)
    struct.pack_into(">I", data, 0, 0x55AA055A)
    struct.pack_into(">I", data, 12, az_start + 1)
    data[31] = 0x31  # lidar_type: airy
    data[32] = 0x02  # lidar_model: 96-line
    for pair in range(_AZ_PER_PACKET):
        az = (az_start + pair) % 360
        az_raw = int(az * 100.0)
        entry = data_map.get(az)
        d_raw_list, r_list = entry if entry else (None, None)
        for sub in range(2):
            block = pair * 2 + sub
            off = 42 + block * 148
            struct.pack_into(">H", data, off, 0xFFEE)
            struct.pack_into(">H", data, off + 2, az_raw)
            base = off + 4
            ch_offset = sub * 48
            for ch in range(48):
                d_raw = d_raw_list[ch_offset + ch] if d_raw_list else 0
                refl = r_list[ch_offset + ch] if r_list else 0
                struct.pack_into(">HB", data, base + ch * 3, d_raw, refl)
    return bytes(data)


def synthesize_frame(terrain: LunarTerrain, tilt_deg: float,
                     az_step: int = 3, max_r: float = 3.0,
                     res: float = 0.05) -> tuple[list[bytes], dict[int, tuple[list[int], list[int]]]]:
    """Generate one 360° frame as a list of MSOP packets + data map."""
    packets: list[bytes] = []
    data_map: dict[int, tuple[list[int], list[int]]] = {}
    for az in range(0, 360, az_step):
        dlist: list[int] = []
        rlist: list[int] = []
        for ch in range(96):
            elev = CHANNEL_VERTICAL_DEG[ch] - tilt_deg
            hd, refl = terrain.ray_hit(az, elev, max_r, res)
            dlist.append(int(hd / 0.005) if hd is not None else 0)
            rlist.append(refl if hd is not None else 0)
        data_map[az] = (dlist, rlist)
    for az_start in range(0, 360, az_step):
        packets.append(build_msop_packet(terrain, tilt_deg, az_start, data_map))
    return packets, data_map


def to_horizontal_frame(frame, tilt_deg: float):
    """Convert parsed (radar-frame) points to the horizontal frame.

    真实部署中 Airy 前倾安装时，点云 z 是雷达坐标系（光轴倾角 tilt 未补偿），
    地形高度判定前需用 IMU 姿态把点旋回水平系。这里用已知 tilt 重建点，
    等价于「已做 IMU 校正」的点云，供自检验证地形逻辑。
    """
    t = math.radians(tilt_deg)
    out = []
    for p in frame.points:
        elev = p.vertical_deg - tilt_deg
        xy = p.distance_m * math.cos(math.radians(elev))
        az_r = math.radians(p.azimuth_deg)
        out.append(LidarPoint(
            azimuth_deg=p.azimuth_deg,
            vertical_deg=elev,
            distance_m=p.distance_m,
            reflectivity=p.reflectivity,
            channel=p.channel,
            x=xy * math.sin(az_r),
            y=xy * math.cos(az_r),
            z=p.distance_m * math.sin(math.radians(elev)),
        ))
    return out


def run_selftest(terrain: LunarTerrain, tilt_deg: float) -> int:
    """Feed a synthetic frame into the parser + terrain profile, print verdicts."""
    print("=== 月球溶洞地形自检 ===")
    print(f"tilt={tilt_deg}°  gravel={len(terrain._gravel)}  "
          f"rocks={len(terrain._rocks)}  pit={terrain._pit is not None}  "
          f"step={terrain._step_h:.2f}m  slope={terrain._slope:.2f}  "
          f"target={terrain._target is not None}")
    packets, data_map = synthesize_frame(terrain, tilt_deg, az_step=3)
    profile = TerrainProfile()
    all_points: list[LidarPoint] = []
    total_points = 0
    for pkt in packets:
        frame = parse_msop_packet(pkt)
        profile.add_frame(to_horizontal_frame(frame, tilt_deg))
        all_points.extend(frame.points)
        total_points += len(frame.points)
    print(f"合成帧: {len(packets)} 包, {total_points} 点")
    print("-" * 72)
    print(f"{'方向':>6} {'判定':>8} {'不可通行@m':>10} {'最高凸起m':>9} "
          f"{'坡度':>6} {'黑洞':>4}")
    for angle in range(0, 360, 30):
        r = profile.sector(angle, DEFAULT_STEP_LIMIT_M)
        verdict = ("不可通行" if r.blocked else
                   "坡面" if r.is_slope else
                   "黑洞/未知" if r.unclear else "可通行")
        print(f"{angle:5d}° {verdict:>8} "
              f"{r.obstacle_distance_m if r.obstacle_distance_m is not None else '—':>10}"
              f"{r.max_height_m:>9.3f} {r.slope_grade:>6.2f} {'Y' if r.unclear else '':>4}")
    print("-" * 72)

    # 反射强度目标检测演示（阶段 3 视觉通路）
    detector = ReflectivityDetector()
    ests = detector.detect(all_points)
    print("反射强度目标检测 (ReflectivityDetector):")
    if ests:
        for e in ests:
            print(f"  命中 → {e.summary()}")
    else:
        print("  （未检测到目标——目标可能被遮挡或超出探测范围）")
    if terrain._target is not None:
        tx, ty, r, h, _ = terrain._target
        true_bearing = math.degrees(math.atan2(tx, ty)) % 360.0
        true_dist = math.hypot(tx, ty)
        print(f"  真值: 方位 {true_bearing:.1f}° 距离 {true_dist:.2f} m "
              f"(目标中心 {tx}, {ty})")
    print("-" * 72)
    print("提示: 台阶/坑/大石方向应显示『不可通行』，碎石方向为『可通行』但近距离点密集。")
    print("注意: --udp 发送的是雷达系原始点云；真实前倾安装需 IMU 校正后才能"
          "得到本自检的水平系地形判定（本自检已模拟 IMU 校正）。")
    return 0


def run_udp(terrain: LunarTerrain, tilt_deg: float, target: str, frames: int) -> int:
    host, _, port_s = target.partition(":")
    port = int(port_s)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"模拟雷达发送到 UDP {host}:{port}（tilt={tilt_deg}°, Ctrl+C 停止）...")
    try:
        sent = 0
        az = 0
        while frames == 0 or sent < frames:
            data_map: dict[int, tuple[list[int], list[int]]] = {}
            for k in range(az, az + _AZ_PER_PACKET):
                azi = k % 360
                dlist: list[int] = []
                rlist: list[int] = []
                for ch in range(96):
                    elev = CHANNEL_VERTICAL_DEG[ch] - tilt_deg
                    hd, refl = terrain.ray_hit(azi, elev)
                    dlist.append(int(hd / 0.005) if hd is not None else 0)
                    rlist.append(refl if hd is not None else 0)
                data_map[azi] = (dlist, rlist)
            pkt = build_msop_packet(terrain, tilt_deg, az, data_map)
            sock.sendto(pkt, (host, port))
            sent += 1
            az = (az + _AZ_PER_PACKET) % 360
            time.sleep(0.002)  # ~ 一帧(90包) ≈ 0.18 s ≈ 5 Hz 全向刷新
    except KeyboardInterrupt:
        print("\n已停止发送")
    finally:
        sock.close()
    return 0


def make_scene(name: str, seed: int = 42) -> LunarTerrain:
    """Named lunar-cave presets for OA coverage (also used by tests)."""
    name = (name or "default").strip().lower()
    if name in ("default", "mixed"):
        return LunarTerrain(seed=seed)
    if name == "gravel":
        return LunarTerrain(seed=seed, gravel=80, rocks=0, pit=False,
                            step_h=0.0, target=None)
    if name == "rock":
        return LunarTerrain(seed=seed, gravel=0, rocks=2, pit=False,
                            step_h=0.0, target=None)
    if name == "step":
        return LunarTerrain(seed=seed, gravel=0, rocks=0, pit=False,
                            step_h=0.14, step_y=1.0, target=None)
    if name == "pit":
        return LunarTerrain(seed=seed, gravel=0, rocks=0, pit=True,
                            step_h=0.0, target=None)
    if name == "slope":
        return LunarTerrain(seed=seed, gravel=0, rocks=0, pit=False,
                            step_h=0.0, slope_deg=12.0, target=None)
    if name == "overhang":
        return LunarTerrain(
            seed=seed, gravel=0, rocks=0, pit=False, step_h=0.0, target=None,
            overhangs=[(0.0, 0.70, 0.12, 0.12, 0.40)],
        )
    if name == "high_ceiling":
        return LunarTerrain(
            seed=seed, gravel=0, rocks=0, pit=False, step_h=0.0, target=None,
            ceiling_z=0.90,
        )
    if name == "cliff":
        return LunarTerrain(
            seed=seed, gravel=0, rocks=0, pit=False, step_h=0.0, target=None,
            cliff_y=0.85,
        )
    if name == "low_refl":
        return LunarTerrain(
            seed=seed, gravel=20, rocks=1, pit=False, step_h=0.0, target=None,
            ground_refl=25,
        )
    raise ValueError(f"unknown lunar scene: {name}")


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Lunar-cave terrain LiDAR simulator")
    p.add_argument("--selftest", action="store_true",
                   help="自检：合成一帧直接验证地形判定（无需网络）")
    p.add_argument("--udp", metavar="HOST:PORT", default=None,
                   help="把合成 MSOP 包发到 UDP 端口（如 127.0.0.1:6699）")
    p.add_argument("--frames", type=int, default=0,
                   help="--udp 发送帧数（0=持续发送）")
    p.add_argument("--seed", type=int, default=42, help="随机种子")
    p.add_argument("--tilt", type=float, default=25.0,
                   help="雷达前倾角（度，模拟安装俯仰，默认 25）")
    p.add_argument("--gravel", type=int, default=60, help="碎石数量")
    p.add_argument("--no-gravel", dest="gravel", action="store_const", const=0,
                   help="关闭碎石")
    p.add_argument("--rocks", type=int, default=2, help="大石数量")
    p.add_argument("--no-pit", dest="pit", action="store_false", default=True,
                   help="关闭坑")
    p.add_argument("--slope", type=float, default=0.0, help="全局坡度（度）")
    p.add_argument("--step-h", type=float, default=0.12, help="台阶高度（米，0=无台阶）")
    p.add_argument("--no-target", dest="target", action="store_false", default=True,
                   help="关闭高反光目标圆柱")
    p.add_argument(
        "--scene",
        default=None,
        help="预设溶洞场景：default/gravel/rock/step/pit/slope/"
             "overhang/high_ceiling/cliff/low_refl（覆盖后仍可用其它开关微调）",
    )
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    if args.scene:
        terrain = make_scene(args.scene, seed=args.seed)
    else:
        terrain = LunarTerrain(
            seed=args.seed,
            gravel=args.gravel,
            rocks=args.rocks,
            pit=args.pit,
            slope_deg=args.slope,
            step_h=args.step_h,
            target=(0.5, 1.6, 0.15, 0.5, _REFL_TARGET) if args.target else None,
        )
    if args.selftest:
        return run_selftest(terrain, args.tilt)
    if args.udp:
        return run_udp(terrain, args.tilt, args.udp, args.frames)
    print("请指定 --selftest 或 --udp（详见模块注释）", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
