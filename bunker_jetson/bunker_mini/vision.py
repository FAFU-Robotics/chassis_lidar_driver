"""Vision-based target detection & localisation (stage 3).

任务链中「视觉判定目标 → 确定目标坐标」这一环。目标检测分两类通路：

  1. **LiDAR 反射强度通路（本模块已实现）**——目标上贴高反光材料
     （如工程级反光贴纸/镀膜），点云 ``reflectivity`` 显著高于周围
     玄武岩（低反射）。按强度阈值过滤 → 方位聚类 → 输出目标的
     车体系方位角 / 距离 / 高度。主动光源，不受月球无光照影响，
     精度到厘米级，零新增硬件。

  2. **相机 / 深度学习模型通路（预留接口）**——组员训练的识别模型
     （YOLO 等）可包装成 :class:`TargetDetector` 接入，无需改动
     find_object / approach 编排。模型负责「这是什么物体」（语义），
     本模块的反射强度通路负责「物体在哪」（几何），可并行融合。

坐标系约定：
  * 检测结果在**车体系**：``bearing_deg`` 0°=车头、左转为正；
    ``distance_m`` 为水平距离；``height_m`` 相对雷达水平面。
  * 换算到导航用的**里程系**坐标（goto 目标点）见 :func:`target_to_odom`。

真实环境注意：
  * 月球玄武岩/尘埃反射率极低，反光标记的信噪比很高——强度阈值
    可以设得比较自信；但在扬尘或强斜射下标记强度会下降，阈值留余量。
  * 目标会被地形（台阶/坡）部分遮挡，故聚类取「最近命中点」的距离，
    高度取「簇内最高点」。
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:  # 避免与 lidar.py 循环依赖（lidar 不 import vision）
    from .lidar import LidarPoint

# 默认反射强度阈值：Airy 反射强度 1~255；高反光贴纸通常 >200，
# 玄武岩/尘埃一般 <120。默认 180 留出余量。
DEFAULT_REFLECT_THRESHOLD: int = 180
DEFAULT_TARGET_MAX_RANGE_M: float = 3.0


@dataclass
class TargetEstimate:
    """One detected target in the vehicle frame."""

    name: str = "target"
    bearing_deg: float = 0.0     # 车体系方位（0°=车头，左转为正）
    distance_m: float = 0.0      # 水平距离（米，簇内最近命中点=目标表面）
    radius_estimate_m: float = 0.0  # 由簇方位角跨度估计的目标半径（0=未知）
    height_m: float = 0.0        # 相对雷达水平面的高度（米）
    confidence: float = 1.0      # 0~1，与命中点数相关
    source: str = "lidar_reflectivity"
    reflectivity: int = 0        # 簇内最高反射强度
    point_count: int = 0         # 簇内命中点数

    @property
    def center_distance_m(self) -> float:
        """目标**中心**的水平距离 = 表面距离 + 半径。

        机械臂抓取的目标点是目标中心；approach 应逼近中心而非表面
        （否则大半径目标会停在够不着的位置）。
        """
        return self.distance_m + self.radius_estimate_m

    def summary(self) -> str:
        return (f"{self.name}@{self.source} 方位{self.bearing_deg:.1f}° "
                f"距离{self.distance_m:.2f}m 中心{self.center_distance_m:.2f}m "
                f"半径{self.radius_estimate_m:.2f}m 高{self.height_m:.2f}m "
                f"点数{self.point_count} 强度{self.reflectivity}")


class TargetDetector(ABC):
    """Detection back-end interface.

    任何检测源（LiDAR 反射强度 / ArUco / 组员深度学习模型）实现
    :meth:`detect`，返回车体系的目标估计列表即可接入 find_object 与
    approach 链路。:class:`ReflectivityDetector` 是本模块的内置实现。
    """

    source: str = "unknown"

    @abstractmethod
    def detect(self, points: list[LidarPoint], **ctx) -> list[TargetEstimate]:
        """Detect targets from one point cloud frame (or other context)."""


class ReflectivityDetector(TargetDetector):
    """Detect targets by clustering high-reflectivity LiDAR points.

    目标贴高反光材料后，反射强度显著高于低反射的月球表面。流程：
    强度过滤 → 按方位/距离聚类（同一目标占连续方位区间）→ 每簇输出
    一个 :class:`TargetEstimate`（方位取簇中心、距离取最近命中、高度
    取簇内最高）。
    """

    source = "lidar_reflectivity"

    def __init__(
        self,
        *,
        threshold: int = DEFAULT_REFLECT_THRESHOLD,
        min_points: int = 3,
        max_range_m: float = DEFAULT_TARGET_MAX_RANGE_M,
        max_az_gap_deg: float = 6.0,   # 同簇最大方位间隔（相邻命中）
        max_dist_gap_m: float = 0.35,  # 同簇最大水平距离跳变
        mount_yaw_deg: float = 0.0,    # 雷达 0° 相对车头偏置（与 AiryLidar 一致）
        max_az_width_deg: float = 30.0,    # 簇方位跨度超此值 → 视为大片亮区（岩壁高亮），非目标
        max_dist_spread_m: float = 0.6,    # 簇内距离差超此值 → 视为散落亮点，非紧致目标
    ) -> None:
        self._threshold = threshold
        self._min_points = max(1, int(min_points))
        self._max_range_m = max_range_m
        self._max_az_gap_deg = max_az_gap_deg
        self._max_dist_gap_m = max_dist_gap_m
        self._mount_yaw_deg = mount_yaw_deg % 360.0
        self._max_az_width_deg = max_az_width_deg
        self._max_dist_spread_m = max_dist_spread_m

    def detect(self, points: list[LidarPoint], **ctx) -> list[TargetEstimate]:
        hits: list[tuple[float, float, float, int]] = []  # (az, hd, z, refl)
        for p in points:
            refl = p.reflectivity
            if refl < self._threshold:
                continue
            hd = p.distance_m * math.cos(math.radians(p.vertical_deg))
            if hd < 0.1 or hd > self._max_range_m:
                continue
            az = (p.azimuth_deg + self._mount_yaw_deg) % 360.0
            hits.append((az, hd, p.z, refl))

        if not hits:
            return []
        hits.sort(key=lambda h: (h[0], h[1]))
        # 旋转排序起点：把「最大方位间隙」放在首尾边界。跨 0° 的目标
        # （如 354°~6°）在普通排序下会被 6°→354° 的大间隙拦腰截断成
        # 两簇；旋转后线性聚类即可整簇合并。
        if len(hits) > 1:
            best_i = 0
            best_gap = -1.0
            for i in range(len(hits) - 1):
                gap = hits[i + 1][0] - hits[i][0]
                if gap > best_gap:
                    best_gap, best_i = gap, i
            wrap_gap = (hits[0][0] + 360.0) - hits[-1][0]
            if wrap_gap <= best_gap:
                hits = hits[best_i + 1:] + hits[:best_i + 1]

        # 方位聚类：相邻命中方位间隔 ≤ gap 且距离变化 ≤ dist_gap → 同簇
        clusters: list[list[tuple[float, float, float, int]]] = []
        cur: list[tuple[float, float, float, int]] = [hits[0]]
        for prev, h in zip(hits, hits[1:]):
            az_delta = abs(((prev[0] - h[0] + 180.0) % 360.0) - 180.0)
            if az_delta <= self._max_az_gap_deg and abs(prev[1] - h[1]) <= self._max_dist_gap_m:
                cur.append(h)
            else:
                clusters.append(cur)
                cur = [h]
        clusters.append(cur)

        out: list[TargetEstimate] = []
        for cl in clusters:
            if len(cl) < self._min_points:
                continue
            azs = [h[0] for h in cl]
            hds = [h[1] for h in cl]
            dist = min(hds)
            # 尺寸合理性过滤：紧致目标（反光标记/目标物体）才可信；
            # 整片岩壁高亮（方位跨度极大）或散落亮点（距离差极大）不是目标。
            az_width = _circular_span_deg(azs)
            if az_width > self._max_az_width_deg:
                continue
            if max(hds) - min(hds) > self._max_dist_spread_m:
                continue
            # 方位中心：跨 0° 边界时用圆形平均
            az_mean = _circular_mean(azs)
            height = max(h[2] for h in cl)
            refl = max(h[3] for h in cl)
            # 半径估计：圆柱目标「半径 ≈ 距离 × tan(方位半跨度)」
            radius = dist * math.tan(math.radians(az_width / 2.0))
            radius = max(0.0, min(0.5, radius))
            # 置信度：命中点数越多越可信（封顶）
            conf = min(1.0, len(cl) / 12.0)
            out.append(TargetEstimate(
                distance_m=dist,
                bearing_deg=az_mean,
                radius_estimate_m=round(radius, 3),
                height_m=height,
                confidence=round(conf, 2),
                source=self.source,
                reflectivity=refl,
                point_count=len(cl),
            ))
        return out


def target_to_odom(x: float, y: float, yaw_rad: float,
                   est: TargetEstimate) -> tuple[float, float]:
    """Convert a vehicle-frame target estimate into odometry-frame (x, y).

    ``(x, y, yaw_rad)`` 是当前里程计位姿（yaw 弧度，与 Pose2D 一致）。
    即任务链「确定目标点的坐标位置」的换算：检测到目标后，
    ``nav.goto(*target_to_odom(pose.x, pose.y, pose.yaw, est))``。

    换算使用目标**中心**距离（表面距离 + 半径估计），确保导航终点是
    机械臂要抓的目标中心，而非目标表面最近点。
    """
    theta = yaw_rad + math.radians(est.bearing_deg)
    d = est.center_distance_m if hasattr(est, "center_distance_m") \
        else est.distance_m
    gx = x + d * math.cos(theta)
    gy = y + d * math.sin(theta)
    return gx, gy


def _circular_mean(angles_deg: list[float]) -> float:
    """Mean of angles with wrap-around (e.g. 358° and 2° → ~0°)."""
    s = sum(math.sin(math.radians(a)) for a in angles_deg)
    c = sum(math.cos(math.radians(a)) for a in angles_deg)
    return (math.degrees(math.atan2(s, c)) % 360.0)


def _circular_span_deg(angles_deg: list[float]) -> float:
    """Circular span of a sorted angle list (0~360): smallest arc containing all.

    跨 0° 边界（如 358° 与 2°）时正确返回 ~4° 而非 356°。
    """
    if len(angles_deg) < 2:
        return 0.0
    a = sorted(x % 360.0 for x in angles_deg)
    max_gap = 0.0
    for p, q in zip(a, a[1:]):
        max_gap = max(max_gap, q - p)
    max_gap = max(max_gap, (a[0] + 360.0) - a[-1])
    return 360.0 - max_gap
