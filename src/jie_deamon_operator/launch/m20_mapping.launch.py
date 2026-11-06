"""Passive M20 point-cloud projection for use alongside NOS drmap mapping."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('jie_deamon')
    config = LaunchConfiguration('config')
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=os.path.join(share, 'config', 'm20.yaml')),
        DeclareLaunchArgument('cloud_topic', default_value='/LIDAR/POINTS'),
        DeclareLaunchArgument('scan_topic', default_value='/m20/mapping_scan'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='m20_cloud_to_scan',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
            remappings=[
                ('cloud_in', LaunchConfiguration('cloud_topic')),
                ('scan', LaunchConfiguration('scan_topic')),
            ],
        ),
    ])
