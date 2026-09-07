# 建图 + 自主导航（不再做 PPT）

Mapping and autonomy entry: `bash nav_demo/scripts/autonomy.sh`. Motion requires `--i-allow-motion`.

入口只有一个：

```bash
cd /home/fafu_robot/Desktop/chassis_lidar_drivers
bash nav_demo/scripts/autonomy.sh
```

不改正式 MOLA YAML / ICP / TF。默认**不发 CAN**。实车跟线必须显式 `--i-allow-motion`。

---

## 现在能做什么

| 能力 | 命令 | 说明 |
|---|---|---|
| 建图 | `autonomy.sh mapping` | 雷达 + 轮式 odom + MOLA 稠密关键帧。你用网页/遥控慢开。 |
| 存图 | `autonomy.sh save` | `/map_save` 写到 `maps/runs/hall_时间/`，不覆盖 lab2。 |
| 感知导航栈 | `autonomy.sh sense` | 雷达 + 稠密 MOLA 里程计 + `obstacle_grid`。 |
| 看状态 | `autonomy.sh status` | 雷达 / pose / grid 是否在发。 |
| A* 只规划 | `autonomy.sh plan --forward 1.8` | 官方规划器，不发车。 |
| 指定目标 | `autonomy.sh plan --gx 3.25 --gy -2.85` | 地图坐标目标。 |
| 实车绕障 | `autonomy.sh live-avoid --i-allow-motion` | 现有 `run_live_avoid.py`，会发 CAN。 |

读图定位（对照已存 `.mm`）**还不能当主路径**：`maps/qualified.json` 里合格图列表是空的。lab/lab2 几何不合格，已挪到 `maps/archive/banned_maps/`，禁止加载。近期导航用 **在线里程计 + 局部栅格 + A***。

点云「漂」：急转后 pose 冻、RViz 看错坐标系、原始云和 LocalMap 叠在一起。建图必须慢开。详见 `maps/README.md`。

---

## 推荐流程

### A. 建一张今天的图

1. 遥控急停可用，场地清空行人。
2. `bash nav_demo/scripts/autonomy.sh mapping`
3. **慢开**，弯角停一下，禁止原地猛拧（急转 pose 会冻）。
4. `bash nav_demo/scripts/autonomy.sh save`
5. 图在 `maps/runs/hall_*/map.mm` 与 `map.simplemap`。验收合格后再写入 `qualified.json`，才能 `start_mola_localization.sh`。

### B. 自主导航（局部）

1. `bash nav_demo/scripts/autonomy.sh stop-mola`（若还在建图模式）
2. `bash nav_demo/scripts/autonomy.sh sense`
3. `bash nav_demo/scripts/autonomy.sh status` — pose、grid 要有 Hz
4. `bash nav_demo/scripts/autonomy.sh plan --forward 1.8`  
   前方空则是直线，这是对的。要绕障就把车摆到障碍侧前方，或用 `--gx --gy`。
5. 确认 `found=True` 且 hits=0 后，**你口头允许**再：  
   `bash nav_demo/scripts/autonomy.sh live-avoid --i-allow-motion`

实车约束（不改）：`v≤0.06`，横向 0.25 m ESTOP，pose 超过 0.50 s 当过期。

---

## 和 PPT 阶段的区别

- 导航 MOLA 关键帧用 **0.25 m / 20°**（环境变量），不再用 PPT 试车时的 1.5 m / 90°。
- 建图输出进 `maps/runs/`，避免覆盖不合格的 lab2。
- 没有合格全局图之前，不做「载入旧地图再定位」。

---

## 日志

`/tmp/nav_demo_bringup/`（mola.log、cloud_to_grid.log、rslidar_sdk.log）
