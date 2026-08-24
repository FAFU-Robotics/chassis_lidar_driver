"""
RoboSense Airy LiDAR driver for the WRS system.
"""
from wrs.drivers.devices.robosense_airy.airy_driver import (
    RoboSenseAiry,
    RoboSenseAiryPcap,
)
from wrs.drivers.devices.robosense_airy.obstacle import (
    DangerZone,
    MultiRegionZones,
    SectorZones,
    check_obstacle,
    danger_zone_mask,
    make_danger_zone_wireframe,
    make_default_sector_zones,
    make_robot_body_wireframe,
    process_obstacle_frame,
    send_chassis_cmd,
    rotate_x_neg90,
    to_forward_frame,
    transform_point_cloud,
    configure_lidar_mount,
    add_obstacle_cli_args,
    danger_zone_from_args,
    print_obstacle_banner,
    DEFAULT_GROUND_Z_MIN,
    CRUISE_VX,
    CRUISE_VY,
)
from wrs.drivers.devices.robosense_airy.pcap_reader import (
    DEFAULT_PCAP,
    load_msop_packets,
)
from wrs.drivers.devices.robosense_airy.viz import (
    IntensityPointCloudRenderer,
    apply_rsview_scene,
    auto_intensity_range,
    filter_near_origin,
    intensity_colormap,
    make_ground_grid,
    prepare_display_frame,
    subsample,
)
# 方式三：扇区密度反应式避障（纯 numpy，不依赖 wrs / panda3d）。
# 注意：不在此重导出 CRUISE_VX / CRUISE_VY，避免与上方 WRS obstacle 的同名
# 常量（DangerZone 体系，值不同）冲突；二者按需经
# ``robosense_airy.reactive_avoid.CRUISE_VX`` 访问。
from .reactive_avoid import (
    GAP_CRAWL_LINEAR,
    OBSTACLE_N_SECTORS,
    OBSTACLE_RANGE_M,
    DECISION_CMDS,
    ObstacleAvoidance,
    points_to_sector_counts,
    sector_range,
)

__all__ = [
    "RoboSenseAiry",
    "RoboSenseAiryPcap",
    "DEFAULT_PCAP",
    "load_msop_packets",
    "DangerZone",
    "MultiRegionZones",
    "SectorZones",
    "DEFAULT_GROUND_Z_MIN",
    "CRUISE_VX",
    "CRUISE_VY",
    "check_obstacle",
    "danger_zone_mask",
    "make_danger_zone_wireframe",
    "make_default_sector_zones",
    "make_robot_body_wireframe",
    "process_obstacle_frame",
    "rotate_x_neg90",
    "to_forward_frame",
    "transform_point_cloud",
    "configure_lidar_mount",
    "send_chassis_cmd",
    "add_obstacle_cli_args",
    "danger_zone_from_args",
    "print_obstacle_banner",
    "IntensityPointCloudRenderer",
    "apply_rsview_scene",
    "auto_intensity_range",
    "filter_near_origin",
    "intensity_colormap",
    "make_ground_grid",
    "prepare_display_frame",
    "subsample",
    "GAP_CRAWL_LINEAR",
    "OBSTACLE_N_SECTORS",
    "OBSTACLE_RANGE_M",
    "DECISION_CMDS",
    "ObstacleAvoidance",
    "points_to_sector_counts",
    "sector_range",
]
