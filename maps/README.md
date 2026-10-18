# maps/ 目录说明

建图与定位相关文件。不要改 `pipelines/lidar3d-icp-airy.yaml`、不要改 `mola_lo.rviz` 里的 ICP/TF 数值。

## 目录

```
maps/
  start_mola_mapping.sh       稠密关键帧建图（0.25 m / 20°）
  start_mola_localization.sh  只加载 qualified.json 里的合格图
  map_save.sh                 /map_save
  publish_wheel_odom.py       轮式 odom → /wheel_odom
  qualified.json              合格图名单（当前 maps=[]）
  pipelines/                  官方 MOLA 管线（勿改）
  mola_lo.rviz                官方 RViz（勿覆盖）
  runs/                       新图输出（hall_时间/）
  legacy_2d/                  旧 slam_toolbox 2D，不与 MOLA 同时开
  archive/banned_maps/        不合格 lab / lab2（会漂、几何不对）
  archive/ppt_figures/        旧 PPT 示意图
```

入口请用仓库根目录 `bash nav_demo/scripts/autonomy.sh`，不要直接加载 lab/lab2。

## 点云为什么会「漂」

不是雷达支架在晃，常见是这几类叠在一起：

1. **急转后 MOLA 位姿冻住**  
   ICP 质量掉到阈值附近，`/lidar_odometry/pose` Hz→0，最后一帧 pose 卡住。LocalMap 不再长，看起来像整幅点云跟着「滑走」或钉死。  
   **对策：** 慢开，弯角停车，禁止原地猛拧。`MOLA_MINIMUM_ICP_QUALITY=0.50` 不要私自改低。

2. **RViz 坐标系看错**  
   原始 `/rslidar_points` 在 `rslidar` 系，96 线扫描环本身就斜。Target Frame 用 `base_link` + Orbit 会更歪。  
   **对策：** Fixed/Target Frame 用 `map`，看 LocalMap 用 XYOrbit。

3. **原始云和 LocalMap 叠在一起**  
   两套点、两种密度、一种还带 decay，会像重影/漂移。  
   **对策：** 建图主看 `/lidar_odometry/localmap_points`。

4. **没有合格全局图**  
   每次重启 MOLA，map 原点重新钉在开机位姿。旧图 lab/lab2 几何不合格，**禁止当定位图**。  
   **对策：** `autonomy.sh mapping` → 慢开 → `save` → 人工验收后再写入 `qualified.json`。

5. **显示层闪烁（已修过）**  
   旧 `viz.py` 随机抽点会造成每帧跳变，已改成固定 stride。UDP 半帧仍可能闪一下，和 MOLA 漂移不是同一件事。

6. **轮式 vs 激光**  
   打滑时轮子走得比激光多。PPT 里 U 形是人开的，激光轨迹经常是 L。以 `/lidar_odometry/pose` 为准。

建图时若 pose Hz 掉到接近 0，先停车等质量回来，不要继续转。
