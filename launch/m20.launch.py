import os
import yaml
from m20_adapter.profile import validate_follow_profile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('jie_deamon')
    config = LaunchConfiguration('config')
    boolean = lambda name: ParameterValue(LaunchConfiguration(name), value_type=bool)
    def validate(context):
        live = LaunchConfiguration('dry_run').perform(context).lower() in ('false', '0')
        web = LaunchConfiguration('enable_web').perform(context).lower() in ('true', '1')
        if live and web:
            raise RuntimeError('M20 live profile requires enable_web:=false; use ROS operator interfaces')
        with open(LaunchConfiguration('config').perform(context)) as stream:
            validate_follow_profile(yaml.safe_load(stream) or {}, live=live)
        return []
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'm20.yaml')),
        DeclareLaunchArgument('cloud_topic', default_value='/LIDAR/POINTS'),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('commissioned', default_value='false'),
        DeclareLaunchArgument('enable_web', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=validate),
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
             name='m20_cloud_to_scan', output='screen',
             parameters=[config, {'use_sim_time': boolean('use_sim_time')}],
             remappings=[('cloud_in', LaunchConfiguration('cloud_topic')), ('scan', '/m20/scan')]),
        Node(package='jie_deamon', executable='robot_nexus', name='robot_nexus', output='screen',
             parameters=[config, {'enable_web': boolean('enable_web'),
                                  'enable_android': False, 'enable_actions': False,
                                  'web_root': os.path.join(share, 'web'),
                                  'use_sim_time': boolean('use_sim_time')}],
             remappings=[('/scan', '/m20/scan'), ('/cmd_vel', '/m20/cmd_vel_raw'),
                         ('/d1_cmd', '/m20/unsupported_d1_actions')]),
        Node(package='jie_deamon', executable='m20_bridge', name='m20_bridge', output='screen',
             parameters=[config, {'dry_run': boolean('dry_run'),
                                  'commissioned': boolean('commissioned'),
                                  'use_sim_time': boolean('use_sim_time')}]),
    ])
