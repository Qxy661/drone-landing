"""
Precision Landing - Real Hardware
RTSP camera + MAVROS
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory("drone_landing"), "config")
    params = os.path.join(config_dir, "landing_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("video_source",
            default_value="rtsp://192.168.1.10:554/stream"),
        DeclareLaunchArgument("fcu_url",
            default_value="serial:///dev/ttyTHS1:57600"),

        Node(package="mavros", executable="mavros_node",
             name="mavros", output="screen",
             parameters=[{
                 "fcu_url": LaunchConfiguration("fcu_url"),
                 "tgt_system": 1, "tgt_component": 1,
             }]),

        Node(package="drone_landing", executable="landing_detector",
             name="landing_detector", output="screen",
             parameters=[params,
                 {"video_source": LaunchConfiguration("video_source")}]),

        Node(package="drone_landing", executable="landing_controller",
             name="landing_controller", output="screen",
             parameters=[params, {"test_mode": False}]),

        Node(package="drone_landing", executable="mission_planner",
             name="mission_planner", output="screen",
             parameters=[params, {"test_mode": False}]),
    ])
