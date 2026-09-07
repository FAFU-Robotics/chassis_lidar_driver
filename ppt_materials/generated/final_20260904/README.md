# 2026-09-04 大厅实验：目录、能力、差距、下一步

场地：实验室大厅  
平台：BUNKER MINI 2.0 + RoboSense Airy 96 线  
本目录只保留**当天定稿素材 + 官方建图日志**。草稿在 `_archive/`。

更细的问题清单见同目录 `问题与下阶段目标.md`。

---

## 1. 整理后的目录

```
final_20260904/
  01_final_rviz_3d_mapping.png      PPT 第1页 三维点云（RViz 实拍）
  02_mola_localmap_ppt_final.png    PPT 第2页 LocalMap（RViz 实拍）
  03_mola_pose_ppt_final.png        PPT 第2页 位姿轨迹（teleop_map4 真实 pose）
  04_real_obstacle_grid.png         PPT 第3页 真实 obstacle_grid
  05_real_astar_path.png            PPT 第3页 A* 绕行图（官方规划器，目标为演示绕障所选）
  05_astar_planner_raw.png          对照：默认向前 1.8 m 的直线规划（不要和 05 混用数字）
  P3_slide_photo.png                PPT 第3页右侧大厅实拍
  p2_measured_stats.txt             第2页右侧「已完成实测」数字
  p3_astar_meta.json                第3页 A* 元数据
  p3_obstacle_grid.npy              当天栅格原值（未改 occupied）
  teleop_map4/                      第2页位姿图对应的完整建图日志
  _archive/drafts/                  被定稿替换的旧截图
  _archive/sessions/                teleop_map / map2 / map3（pose 冻过，不作 PPT 主图）
  _archive/map_runs/                空的建图输出目录
```

**PPT 三页只用顶层 `01`–`05` + `P3_slide_photo`。**

| 页 | 用这张 | 不要用 |
|---|---|---|
| 01 | `01_final_rviz_3d_mapping.png` | `_archive` 里更早的 lidar 截图 |
| 02 上 | `02_mola_localmap_ppt_final.png` | teleop3 的 LocalMap |
| 02 下 | `03_mola_pose_ppt_final.png` | 不要说成闭合 U |
| 02 右 | `p2_measured_stats.txt` | 不要用 1287→1685 旧数 |
| 03 左 | `04_real_obstacle_grid.png` | |
| 03 中 | `05_real_astar_path.png` | 不要和 `05_astar_planner_raw` 的 1.80 m 混标 |
| 03 右 | `P3_slide_photo.png` | |

当前 `05_real_astar_path.png` 元数据：`found=True`，**3.35 m**，**5 waypoints**，hits=0，最近障碍 0.511 m。若 PPT 里仍写 3.91 m / wps=6，改成与这张图一致，或换回当时那张图，**不要混用**。

---

## 2. 现在有什么、能实现什么

### 硬件与驱动

- 履带底盘 CAN、网页/遥控 teleop、`:9100` stick 通道已有。
- Airy 96 线 UDP 有线可达，`/rslidar_points` 约 10–11 Hz。
- 静态 TF：`base_link → rslidar` 已固定，不要改正式 YAML。

**能做：** 人开车、雷达出点云、RViz 看三维环境。

### 定位与局部建图（MOLA）

- 正式建图脚本 `maps/start_mola_mapping.sh`（关键帧约 0.25 m / 20°）。
- 导航里程计脚本更稀（约 1.5 m / 90°），适合跟车，不适合当大厅地图主图。
- teleop_map4：**pose 未冻**，路径 8.746 m，LocalMap 1244→7847，quality 平均 0.886，ICP 0。

**能做：** 慢速遥控建局部地图、实时位姿、PPT 第 2 页素材。  
**不能保证：** 急转后 pose 不冻；激光轨迹走出闭合 U。

### 感知栅格

- `nav_demo/cloud_to_grid.py`：点云 + MOLA pose → `/nav_demo/obstacle_grid`（120×120，0.10 m）。
- 值：100 occupied / 50 inflated / 0 free / −1 unknown。

**能做：** 实时占用图，给 A* 用。  
**观感：** unknown 多、植物/箱子是团块，不是照片级。

### 规划（A*，只规划不运动）

- `nav_demo/plan_once.py`：默认前方 1.8 m，官方 `GlobalPlanner`，inflation 0.20。
- 默认结果：前方空则 **直线 1.80 m、2 路点**（见 `05_astar_planner_raw.png`）。
- 演示绕障：目标选在「直线会撞 occupied」的可通行点，一次 `plan()` 得到绕行（见 `05_real_astar_path.png`）。
- `run_mvp.py`：A* + Navigator + Guard，只打印 `DRY_RUN`，不发 stick。

**能做：** 真实栅格上算出不穿障碍的路径，并发布 `/nav_demo/plan`。  
**不能说：** 车已经跟着跑完；默认 1.8 m 就是绕行图。

### 实车跟踪 / 避障（代码有，当天未作为 PPT 主实验）

- `run_live_go.py` / `run_live_avoid.py` 已存在。
- 约束（按约定不改）：`v≤0.06`，横向偏离 **0.25 m** 就 ESTOP，pose 超过 **0.50 s** 当过期。
- 历史上 live-avoid 容易因横向阈值 / pose 过期停住。

**能做：** 口头允许后，低速跟 A* 路点。  
**尚未交付：** 一段可答辩的「实车绕开植物/箱体」录像。

---

## 3. 距离目标还差什么

目标（国赛可演示）：**真实感知 → 真实定位 → 真实规划 → 实车绕障并录像**。

| 环节 | 状态 | 缺口 |
|---|---|---|
| 点云感知 | 已有，PPT 01 已用 | 无 |
| MOLA 定位 / LocalMap | 已有，PPT 02 已用 | 急转易冻；轨迹是 L 不是闭合 U |
| obstacle_grid | 已有，PPT 03 左已用 | 走廊窄、unknown 多 |
| A* dry-run | 已有，PPT 03 中已用 | 默认直线不好看；绕行图换过目标 |
| 路径跟踪实车 | 代码有，未作为主实验 | **缺跟线视频** |
| 动态避障实车 | 代码有，易 ESTOP | **缺成功绕障视频** |
| 闭合 U 跑道 | 人开过，激光没走完 | 不要在 PPT 上画成 U |

PPT 本身可以定稿。国奖成色差在 **第 3 页「明日实车验证」还没发生**。

---

## 4. 下一步怎么规划

不要再改正式 YAML / ICP / TF / A* 常数。不要为好看补假地图。

### 第 0 步：PPT 定稿（今晚可做完）

1. 三页只用本目录顶层 01–05。
2. 第 2 页右侧用 `p2_measured_stats.txt`。
3. 第 3 页底栏与 `05_real_astar_path.png` 一致：`3.35 m`、hits=0、最近障碍 0.511 m。
4. 答辩不说闭合 U、不说已经实车跑完、不说默认 1.8 m 就是绕行。

### 第 1 步：实车前预检（开车前 10 分钟）

```text
雷达 Hz > 8
pose Hz 稳定、age < 0.5 s、quality 不要掉到冻结
obstacle_grid 在发
plan_once 先 dry-run，found=True 且 hits=0
遥控急停可用
```

人开车：**慢、大弯分段停、禁止原地猛拧。**

### 第 2 步：只规划，确认绕障目标

- 车停在植物/箱体侧前方，正前方或侧前方有真实障碍。
- `/usr/bin/python3 nav_demo/plan_once.py`（或约定的 forward）。
- 若直线 1.8 m 仍是空的，把车或障碍摆到「直线会撞、绕行可通」，再规划。
- **不发 CAN。** 确认绕行后再进入第 3 步。

### 第 3 步：口头允许后 live-avoid

- 只启动现有 `run_live_avoid.py`，不改 0.06 / 0.25 / 0.50。
- 同时录：实车画面 + RViz（grid + 绿线 + 车）。
- 横向超 0.25 m 或 pose 过期会停：记下原因，不要当场改参数硬冲。

### 第 4 步：收实验包

成功或失败都留下：

- 起止 pose、found、path length、waypoints、hits、最近障碍
- pose 是否冻、是否 ESTOP
- 两路录像

放到新目录，例如 `ppt_materials/generated/live_avoid_YYYYMMDD/`，不要覆盖本目录定稿图。

### 验收（做完才算下一阶段完成）

- [ ] pose 全程未冻
- [ ] `found=True`，occupied/inflated hits=0
- [ ] 实车走出肉眼可见的绕行
- [ ] 实车 + RViz 同步录像

---

## 5. 当天关键数字（备查）

**teleop_map4（第 2 页）：**  
LiDAR ≈11 Hz（截图时 11.25）· Pose ≈5 Hz · quality avg ≈0.89 · LocalMap 1244→7847 · ICP 0 · TF 正常 · 路径 8.746 m · 未冻

**A* 演示图（第 3 页 05）：**  
found=True · 3.35 m · 5 wps · hits=0 · 最近 occupied 0.511 m · 未执行 · 未发 CAN

**A* 默认对照（05_raw）：**  
found=True · 1.80 m · 2 wps · 直线
