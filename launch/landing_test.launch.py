"""
Precision Landing - Test Mode
Uses webcam, no FCU connection
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory("drone_landing"), "config")
    params = os.path.join(config_dir, "landing_params.yaml")

    return LaunchDescription([
        Node(package="drone_landing", executable="landing_detector",
             name="landing_detector", output="screen",
             parameters=[params]),

        Node(package="drone_landing", executable="landing_controller",
             name="landing_controller", output="screen",
             parameters=[params, {"test_mode": True}]),

        Node(package="drone_landing", executable="mission_planner",
             name="mission_planner", output="screen",
             parameters=[params, {"test_mode": True}]),
    ])
