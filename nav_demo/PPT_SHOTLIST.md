PPT 素材说明（必须来自真实运行，禁止合成假数据）
================================================

RViz 只用：ppt_materials/rviz/ppt_live.rviz
启动：bash ppt_materials/scripts/start_rviz_ppt.sh
不要用 maps/mola_lo.rviz。

感知栈需已在跑：rslidar_sdk + TF + wheel_odom + MOLA + nav_demo/cloud_to_grid.py
（bash nav_demo/scripts/bringup_sense.sh）


A. 机器人实物照片
-----------------
手机拍，横构图：
  1. 整车 3/4 侧前方（Bunker Mini 2.0 + 顶部 Airy）
  2. 车头正对，能看到履带与雷达
光线：实验室正常灯，不要强补光把雷达罩打成一片白。
人不要入镜（或只留一只手做尺度）。


B. LiDAR 特写
-------------
  1. Airy 顶部特写，能看出安装高度
  2. 雷达网口/线束（可选一张）
不要拆壳。


C. PPT 专用 RViz
----------------
打开 ppt_live.rviz 后立刻确认：
  - Fixed Frame = map
  - 背景纯黑
  - RawLidar 勾选（浅灰点，非彩虹）
  - ObstacleGrid 勾选（/nav_demo/obstacle_grid）
  - AStarPlan 勾选（运行 plan_once.py 或 run_mvp.py 后才有黄线）
  - LidarPose 坐标轴可见
  - TF 只显示 map / base_link / rslidar
视角：机器人在画面中下部，前方 3–6 m 环境占满中景。滚轮不要缩成一个点。


D. 点云截图（真实 /rslidar_points）
----------------------------------
静止 3 秒后截 RViz 全窗口（含左侧 Displays，证明 topic 是真的）。
文件建议：ppt_materials/A_perception/D_cloud_still.png
评委应看到墙/家具的真实点，而不是空盒子。


E. Grid 截图（真实 /nav_demo/obstacle_grid）
------------------------------------------
同一视角，ObstacleGrid 打开，RawLidar 可略降透明度。
确认 OccupancyGrid 在机器人周围有占用格（不是整片未知）。
终端对照 /tmp/nav_demo_bringup/cloud_to_grid.log：
  occupied_cells 应稳定 > 0
文件建议：ppt_materials/A_perception/E_obstacle_grid.png


F. 定位 / 局部地图
------------------
打开 LocalMap + LidarPose，Fixed Frame=map。
静止一张 +（可选）你遥控慢转 8–10 秒的录屏。
不要声称 ICP 的 ~14–16 cm XY 已解决。
文件建议：ppt_materials/B_localization/F_localmap_pose.png


G. A* 路径截图（真实规划，车不动）
--------------------------------
另开终端（不控车）：

  /usr/bin/python3 nav_demo/plan_once.py --forward 1.8

RViz 应出现黄色 /nav_demo/plan。截图同时要能看到 Grid + 路径 + 坐标轴。
若路径是两点直线：说明前方 1.8 m 视线内无膨胀障碍，这是真实结果，不要改成弯的假路径。
文件建议：ppt_materials/C_motion/G_astar_path.png


H. 最终自主避障视频（实车，必须你允许后才拍）
------------------------------------------
现在还不能拍「车自己走」的成片。dry-run 已通，等你允许实车后再录。

建议流程（允许后）：
  1. 清空前方 3 m 走廊，一侧放一把真实椅子（不要贴车）
  2. 人站在急停位置（网页 :9101 或遥控器）
  3. RViz 用 ppt_live.rviz 录屏 + 手机拍车
  4. 启动 dry-run 确认 v/w 打印正常后，再切到 stick 使能
  5. 目标约 1.5–2.0 m，v ≤ 0.12 m/s
  6. 成片 15–25 秒：起步 → 靠近椅子减速/绕开或停车 → 停止
  7. 失败立刻松键 / 急停；回退：只跑感知 + RViz，不再发 stick

未允许前，可用「RViz 里 Grid + 黄线 + 终端 DRY_RUN v=0.12」作为过渡页，
标题必须写「接口联调 / 未发车」，不能写成已经自主行驶。
