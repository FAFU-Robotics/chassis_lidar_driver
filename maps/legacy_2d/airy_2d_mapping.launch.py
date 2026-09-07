"""Airy wall-band LaserScan + static TFs + slam_toolbox occupancy mapping."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_base_to_rslidar",
            # Live cloud: +Z optical (forward), +Y up (floor at y≈-0.36 m).
            # p_base = (z_s, x_s, y_s) + (0, 0, 0.365)
            arguments=["--x", "0", "--y", "0", "--z", "0.365",
                       "--roll", "1.570796", "--pitch", "0", "--yaw", "1.570796",
                       "--frame-id", "base_link", "--child-frame-id", "rslidar"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_base_to_footprint",
            arguments=["--x", "0", "--y", "0", "--z", "0",
                       "--roll", "0", "--pitch", "0", "--yaw", "0",
                       "--frame-id", "base_link", "--child-frame-id", "base_footprint"],
        ),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            remappings=[("cloud_in", "/rslidar_points"), ("scan", "/scan")],
            parameters=[{
                "target_frame": "base_link",
                "transform_tolerance": 0.3,
                "min_height": 0.25,
                "max_height": 1.60,
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.0087,
                "scan_time": 0.1,
                "range_min": 0.80,
                "range_max": 12.0,
                "use_inf": True,
                "use_sim_time": use_sim_time,
            }],
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                os.path.join(os.path.dirname(os.path.abspath(__file__)), "slam_toolbox_airy.yaml"),
                {"use_sim_time": use_sim_time},
            ],
        ),
    ])
