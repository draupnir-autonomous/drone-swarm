#!/usr/bin/python3
# -- coding: utf-8 --**

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    
    # Find path
    config_file_dir = os.path.join(get_package_share_directory("fast_livo2"), "config")
    rviz_config_file = os.path.join(get_package_share_directory("fast_livo2"), "rviz_cfg", "M300.rviz")

    #Load parameters
    avia_config_cmd = os.path.join(config_file_dir, "MARS_LVIG.yaml")
    camera_config_cmd = os.path.join(config_file_dir, "camera_MARS_LVIG.yaml")

    # Param use_rviz
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="False",
        description="Whether to launch Rviz2",
    )

    use_republish_arg = DeclareLaunchArgument(
        "use_republish",
        default_value="False",
        description="Whether to launch the image republish node",
    )

    avia_config_arg = DeclareLaunchArgument(
        'avia_params_file',
        default_value=avia_config_cmd,
        description='Full path to the ROS2 parameters file to use for fast_livo2 nodes',
    )

    camera_config_arg = DeclareLaunchArgument(
        'camera_params_file',
        default_value=camera_config_cmd,
        description='Full path to the ROS2 parameters file to use for vikit_ros nodes',
    )

    use_respawn_arg = DeclareLaunchArgument(
        'use_respawn', 
        default_value='True',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    avia_params_file = LaunchConfiguration('avia_params_file')
    camera_params_file = LaunchConfiguration('camera_params_file')
    use_respawn = LaunchConfiguration('use_respawn')

    return LaunchDescription([
        use_rviz_arg,
        use_republish_arg,
        avia_config_arg,
        camera_config_arg,
        use_respawn_arg,

        # use parameter_blackboard as global parameters server and load camera params
        Node(
            package='demo_nodes_cpp',
            executable='parameter_blackboard',
            name='parameter_blackboard',
            # namespace='laserMapping',
            parameters=[
                camera_params_file,
            ],
            output='screen'
        ),

        # republish compressed image to raw image
        # https://robotics.stackexchange.com/questions/110939/how-do-i-remap-compressed-video-to-raw-video-in-ros2
        # ros2 run image_transport republish compressed raw --ros-args --remap in:=/left_camera/image --remap out:=/left_camera/image
        Node(
            condition=IfCondition(LaunchConfiguration("use_republish")),
            package="image_transport",
            executable="republish",
            name="republish",
            arguments=[ # Array of strings/parametric arguments that will end up in process's argv
                'compressed', 
                'raw',
            ],
            remappings=[
                ("in",  "/left_camera/image"),
                ("out", "/left_camera/image")
            ],
            output="screen",
            respawn=use_respawn,
        ),
        
        Node(
            package="fast_livo2",
            executable="fastlivo_mapping",
            name="laserMapping",
            parameters=[
                avia_params_file,
                camera_params_file,
            ],
            prefix=[
            ],
            output="screen"
        ),

        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen"
        ),
    ])