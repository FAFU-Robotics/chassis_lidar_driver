# 2026-09-07 Deskew / ICP / 绕障跟线

工控机：`fafu_robot`（Jetson，主机名 `fafurobot`）  
底盘：Bunker Mini 2.0 测试车  
雷达：RoboSense Airy 96 线  
工程：`/home/fafu_robot/Desktop/chassis_lidar_drivers`  
ROS 2 Humble；导航节点必须 `/usr/bin/python3`（3.10），conda 3.13 会弄坏 rclpy。

前序：交接审计、PPT 素材、`--live-avoid` 因 pose 中断未到点。  
本日：**跟老师对齐主问题 → 只读核实 deskew/ICP → 改掉原地 ALIGN，验证留明天。**

---

## 一、跟老师对齐的结论

老师关心 deskew 和 ICP。交接时也确认：**没有独立 Global Registration / FPFH-RANSAC**。  
当前 runtime 失败点不是缺 FPFH，而是 **绕障 ALIGN 原地转时点到点 ICP 跟丢 pose**。

---

## 二、Deskew：通路通，效果没在转动时验证

只读实测（感知已起，车静止，未发 CAN）：

| Topic | 结果 |
|---|---|
| `/rslidar_points` | 86400 点；字段 `x y z intensity ring timestamp`；一帧 `timestamp` 跨度 **0.100 s** |
| `/lidar_odometry/deskewed_scan_points` | 约 3174 点；有 MOLA 字段 **`t`**（float32，跨度约 0.045 s） |
| `/lidar_odometry/pose` | 约 8 Hz；静止 `pose_quality ≈ 0.99`；`twist` 全 0 |

含义：

- YAML 里 “Airy clouds have no per-point timestamps” **与今天数据不符**。
- MRPT 2.15 会把 ROS 的 `timestamp` 转成内部 `t`。时间戳链路是通的。
- 车停着、twist=0 时 deskew 补偿量为 0，**看不出转动时有没有把点挪对**。
- `MOLA_OPTIMIZE_TWIST=false`：角速度来自运动模型；ICP 一挂，deskew 也没可用 twist。

**不能说 deskew 已经可用；只能说“不是没接上时间戳”。**

---

## 三、ICP：这是现在卡住的地方

定位：`maps/pipelines/lidar3d-icp-airy.yaml`  
`mp2p_icp::ICP` + `Matcher_Points_DistanceThreshold`（点到点，不是 GICP，不是 FPFH）。

当前扫描去贴 **LocalMap**（体素局部地图），不是两帧原始点云互配。

实车：`--live-avoid` 走到绕行第一段后，ALIGN 原地转对准下一航点 →  
`LIVE_AVOID_ABORT reason=MOLA pose dead held=2.570s`，`arrived=False`。  
雷达还在出点，是 `/lidar_odometry/pose` 真停更。`POSE_STALE_S=0.50` 判断正确。

导航模式原先关键帧 **1.5 m / 90°**，2 m 绕障几乎不加图，ICP target 很稀。  
静止 quality 高时自适应窗口会收死，突然一转更容易配不上。

---

## 四、ALIGN 是什么、为什么要命

`run_live_avoid.py` 旧逻辑：

- **ALIGN**：`v=0`，`|w|≤0.20`，原地转到对准下一个 A* 航点  
- **DRIVE**：偏航 <5° 才 `v=0.05`；偏航 >8° 再回 ALIGN  

绕障路径一拐弯就必须 ALIGN。ICP 最怕履带原地搓。  
ALIGN 不是 MOLA 模块，只是这套跟线写法。

---

## 五、官方 / 同类做法（未整包照搬）

MOLA 教程：[先建图再定位](https://docs.mola-slam.org/latest/tutorial-mola-lo-map-and-localize.html)

仓库已有脚本：

- 建图：`maps/start_mola_mapping.sh`（0.25 m / 20°）  
- 读图：`maps/start_mola_localization.sh`  

**现在用不了：** `maps/qualified.json` 为空；`lab` / `lab2` 几何不合格，在 `archive/banned_maps/`。

全网同类：纯 ICP 在旋转时容易挂，常见对策是边走边转、轮式/IMU 给初值、先有一张合格图再定位。不是先上每帧 FPFH。

---

## 六、今天已改的代码（未实车）

未改 `--live-go`，未改 ICP matcher 类，未发 CAN。

### 1. `nav_demo/run_live_avoid.py`

取消 `v=0` ALIGN，改为 **ARC 边走边转**：

- `v ∈ [0.025, 0.05]`（始终 >0）  
- `|w| ≤ 0.08`（低于原来 ALIGN 的 0.20）  
- 偏航大：`ARC` / `ARC_SLOW`；对准：`DRIVE`

### 2. `nav_demo/scripts/start_mola_nav.sh`

| 项 | 改前 | 改后 |
|---|---|---|
| 关键帧平移/转角 | 1.5 m / 90° | **0.50 m / 30°** |
| `MOLA_SIGMA_MAX_MOTION` | 0.30 m | **2.00 m**（失败后能把搜索窗拉开） |

要生效必须重启 MOLA：`bash nav_demo/scripts/restart_mola_deskew.sh`  
**今天没有重启、没有 dry-run、没有 live。**

---

## 七、明天验证（已约好）

1. `/usr/bin/python3 nav_demo/run_live_avoid.py --dry-only`  
   应看到 ARC/DRIVE，不能再出现 `v=0` 的 ALIGN，结尾 `NO_SPIN=1`。  
2. `bash nav_demo/scripts/restart_mola_deskew.sh`  
   确认 pose 仍在更新。  
3. **口头允许后再** `/usr/bin/python3 nav_demo/run_mvp.py --live-avoid`  
   看拐弯 pose 是否还断、能不能到点。

`--live-go` 先不动。合格图 + 只定位仍是后续系统形态，不是明天第一刀。

---

## 八、给老师的一句话

Deskew 时间戳已经接到 MOLA；转动补偿还没验证。  
当前卡点是 **绕障原地转导致点到点 ICP 丢 pose**。  
已改成边走边转，并略加密导航 LocalMap、放宽 ICP 恢复窗；实车验证留明天。
