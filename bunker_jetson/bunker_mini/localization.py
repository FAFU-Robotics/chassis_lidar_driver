"""Localization & map-frame alignment — 建图适配框架 (map-ready scaffold).

本模块是「组员建图完成后」接入 SLAM 位姿与全局地图坐标系的适配层。
当前阶段（建图未完成）默认：
  * 无外部位姿源（``pose_source=None``）→ 纯里程航迹推算；
  * ``MapAlignment`` 恒等变换 → 里程系 == 地图系；
  * 地图优先返回未启用 → 返回仍沿探路轨迹 / 直线。

建图完成后，只需三件事即可无缝接入，**无需改动导航/任务编排**：

  1. 提供一个 :class:`PoseSource` 实现：``get_pose()`` 返回 SLAM 位姿
     ``(x, y, yaw_deg)``（地图系），无观测时返回 ``None``。组员的 SLAM
     （回环 / scan-matching）比里程准得多，把它作为**绝对位姿修正源**
     周期喂给 ``Navigator.apply_external_pose``，即可根治长时间航迹推算
     的漂移——这是任务链「找目标 → 导航 → 返回」精度的最关键一环。

  2. 做一次标定，把 SLAM 地图原点在里程系中的位姿填入 :class:`MapAlignment`
     （通常：把车停在地图已知点位，读两坐标系下的位姿求差），保证
     A* 规划出的航点与导航/里程系对齐。

  3. 把 ``agent.prefer_map_return`` 置 ``True``，返回阶段即可用全局地图
     A* 规划更优返回路径，而不是原路轨迹回放。

坐标系约定与 ``navigator.OdometryPose`` 一致：里程系 yaw=0 时车头沿世界
+x，逆时针为正；地图系同向（仅原点/朝向偏移）。本模块保持零第三方依赖。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Protocol

_DEG_PER_RAD: float = 180.0 / math.pi
_RAD_PER_DEG: float = math.pi / 180.0


def _wrap_deg(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


@dataclass
class MapAlignment:
    """里程系 ↔ 地图系的刚性坐标变换（默认恒等）。

    ``origin_*`` 表示**地图原点在里程系中的位姿**。未标定时三者均为 0，
    等价于两坐标系重合。标定后：
      * ``odom_to_map``：里程系 (x, y, yaw_deg) → 地图系；
      * ``map_to_odom``：地图系 → 里程系（喂给 navigator 前用它换算）。
    """

    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_yaw_deg: float = 0.0

    def odom_to_map(self, x: float, y: float,
                    yaw_deg: float) -> tuple[float, float, float]:
        theta = self.origin_yaw_deg * _RAD_PER_DEG
        xo, yo = x - self.origin_x, y - self.origin_y
        mx = xo * math.cos(theta) + yo * math.sin(theta)
        my = -xo * math.sin(theta) + yo * math.cos(theta)
        return mx, my, _wrap_deg(yaw_deg - self.origin_yaw_deg)

    def map_to_odom(self, x: float, y: float,
                    yaw_deg: float) -> tuple[float, float, float]:
        theta = self.origin_yaw_deg * _RAD_PER_DEG
        xo = x * math.cos(theta) - y * math.sin(theta)
        yo = x * math.sin(theta) + y * math.cos(theta)
        return xo + self.origin_x, yo + self.origin_y, \
            _wrap_deg(yaw_deg + self.origin_yaw_deg)


class PoseSource(Protocol):
    """外部绝对位姿源（SLAM / 视觉定位 / UWB 等）的约定接口。

    组员建图模块实现 ``get_pose``，返回**地图系** ``(x, y, yaw_deg)``；
    无当前观测（建图未收敛 / 丢跟踪）时返回 ``None`` 表示「本次不修正」。
    agent 的定位 tick 会用 :class:`MapAlignment.map_to_odom` 换算后喂给
    ``Navigator.apply_external_pose``，内部仍统一里程系。
    """

    def get_pose(self) -> Optional[tuple[float, float, float]]:
        """Return (x, y, yaw_deg) in the map frame, or None if unavailable."""
        ...
