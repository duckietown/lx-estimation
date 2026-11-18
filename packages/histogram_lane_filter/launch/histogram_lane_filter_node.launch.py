import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    veh = LaunchConfiguration("veh")
    config = os.path.join(
        get_package_share_directory("histogram_lane_filter"),
        "config",
        "histogram_lane_filter_node",
        "default.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument("veh", default_value=EnvironmentVariable("VEHICLE_NAME"),
                              description="robot name (namespace of every topic)"),
        Node(
            package="histogram_lane_filter",
            executable="histogram_lane_filter_node",
            name="histogram_lane_filter_node",
            namespace=veh,
            parameters=[config],
            output="screen",
            emulate_tty=True,
        ),
    ])
