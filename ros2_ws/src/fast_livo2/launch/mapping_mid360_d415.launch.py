#!/usr/bin/python3
# -- coding: utf-8 --

# Launch FAST-LIVO2 with Livox Mid-360 LiDAR + Intel RealSense D415 camera
# on Unitree A2-W robot.
#
# Prerequisites:
#   source /opt/ros/$ROS_DISTRO/setup.bash
#   source ~/fast_livo2_ws/install/setup.bash
#
# Usage:
#   ros2 launch fast_livo2 mapping_mid360_d415.launch.py
#   ros2 launch fast_livo2 mapping_mid360_d415.launch.py use_rviz:=true
#
# Sensors:
#   LiDAR : Livox Mid-360 (topic /livox/lidar)
#   Camera: Intel RealSense D415 640x480@15fps (topic /camera/camera/color/image_raw)
#   IMU   : Livox Mid-360 integrated IMU (topic /livox/imu)
#
# Calibration:
#   Camera-LiDAR extrinsics calibrated with FAST-Calib (2026-04-20)
#   Rcl/Pcl written into mid360-a2w.yaml

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    config_file_dir = os.path.join(
        get_package_share_directory("fast_livo2"), "config"
    )
    rviz_config_file = os.path.join(
        get_package_share_directory("fast_livo2"), "rviz_cfg", "fast_livo2.rviz"
    )

    mid360_config = os.path.join(config_file_dir, "mid360-a2w.yaml")
    camera_config = os.path.join(config_file_dir, "camera_d415.yaml")

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="False",
        description="Launch Rviz2 for visualization (requires display)",
    )

    use_republish_arg = DeclareLaunchArgument(
        "use_republish",
        default_value="False",
        description="Republish compressed image as raw (not needed for D415 raw topic)",
    )

    mid360_config_arg = DeclareLaunchArgument(
        "mid360_params_file",
        default_value=mid360_config,
        description="Full path to mid360-a2w.yaml (LiDAR + IMU + extrinsics)",
    )

    camera_config_arg = DeclareLaunchArgument(
        "camera_params_file",
        default_value=camera_config,
        description="Full path to camera_d415.yaml (D415 640x480 intrinsics)",
    )

    use_respawn_arg = DeclareLaunchArgument(
        "use_respawn",
        default_value="True",
        description="Respawn node on crash",
    )

    mid360_params_file = LaunchConfiguration("mid360_params_file")
    camera_params_file = LaunchConfiguration("camera_params_file")
    use_respawn = LaunchConfiguration("use_respawn")

    return LaunchDescription([
        use_rviz_arg,
        use_republish_arg,
        mid360_config_arg,
        camera_config_arg,
        use_respawn_arg,

        # Global parameter server: loads camera intrinsics so fast_livo2 can read them
        Node(
            package="demo_nodes_cpp",
            executable="parameter_blackboard",
            name="parameter_blackboard",
            parameters=[camera_params_file],
            output="screen",
        ),

        # FAST-LIVO2 mapping node
        # camera_params_file loaded here so getRemoteParam fallback step 2
        # (nh->get_parameter("cam_model")) succeeds without relying on
        # parameter_blackboard timing.
        Node(
            package="fast_livo2",
            executable="fastlivo_mapping",
            name="laserMapping",
            parameters=[mid360_params_file, camera_params_file],
            output="screen",
            respawn=use_respawn,
        ),

        # Optional: RViz2 (headless robot — disabled by default)
        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen",
        ),
    ])
