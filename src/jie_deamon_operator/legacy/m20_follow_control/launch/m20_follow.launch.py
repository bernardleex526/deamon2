import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("m20_follow_control")
    parameters = os.path.join(package_share, "config", "m20_follow.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "input_mode",
            default_value="ros",
            description="Point-cloud transport: ros or socket",
        ),
        DeclareLaunchArgument(
            "follow_enabled",
            default_value="false",
            description="Allow the tracker to publish nonzero cmd_vel commands",
        ),
        DeclareLaunchArgument(
            "motion_output_enabled",
            default_value="false",
            description="Allow the UDP bridge to send M20 body-control commands",
        ),
        Node(
            package="m20_follow_control",
            executable="m20_follow_node",
            name="m20_follow_node",
            output="screen",
            parameters=[
                parameters,
                {
                    "enabled": LaunchConfiguration("follow_enabled"),
                    "input_mode": LaunchConfiguration("input_mode"),
                },
            ],
        ),
        Node(
            package="m20_follow_control",
            executable="m20_udp_bridge",
            name="m20_udp_bridge",
            output="screen",
            parameters=[
                parameters,
                {"motion_output_enabled": LaunchConfiguration("motion_output_enabled")},
            ],
        ),
    ])
