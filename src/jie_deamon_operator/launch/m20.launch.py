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
    boolean = lambda name: ParameterValue(LaunchConfiguration(name), value_type=bool)
    string = lambda name: ParameterValue(LaunchConfiguration(name), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'm20.yaml')),
        DeclareLaunchArgument('cloud_topic', default_value='/LIDAR/POINTS'),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('commissioned', default_value='false'),
        DeclareLaunchArgument('enable_web', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # Operator gateway (authenticated phone/Android page). enable_web:=false
        # keeps the ROS-only path and the legacy web/Android servers off.
        DeclareLaunchArgument('operator_token_file',
                              default_value=os.environ.get('M20_OPERATOR_TOKEN_FILE', '')),
        DeclareLaunchArgument('operator_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('operator_port', default_value='8080'),
        DeclareLaunchArgument('android_port', default_value='8889'),
        # Gate policy.
        DeclareLaunchArgument('control_profile', default_value='auto',
                              description='auto | si (Cmd=25, navigation mode) | '
                                          'normalized (Cmd=21, regular mode)'),
        DeclareLaunchArgument('axis_enable_y', default_value='true'),
        DeclareLaunchArgument('axis_enable_yaw', default_value='true'),
        DeclareLaunchArgument('allow_reverse', default_value='false',
                              description='allow algorithm output to drive backwards'),
        DeclareLaunchArgument('gait_min_policy', default_value='zero',
                              description='zero (fail-closed) | snap (raise a non-zero '
                                          'request to the gait minimum)'),
        DeclareLaunchArgument('require_preflight', default_value='true',
                              description='live start requires a fresh all-pass '
                                          '/tmp/m20_preflight.json on this host'),
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
             name='m20_cloud_to_scan', output='screen',
             parameters=[config, {'use_sim_time': boolean('use_sim_time')}],
             remappings=[('cloud_in', LaunchConfiguration('cloud_topic')), ('scan', '/m20/scan')]),
        Node(package='jie_deamon', executable='robot_nexus', name='robot_nexus', output='screen',
             parameters=[config, {'enable_web': False,
                                  'operator_control': boolean('enable_web'),
                                  'enable_android': False, 'enable_actions': False,
                                  'web_root': os.path.join(share, 'web'),
                                  'use_sim_time': boolean('use_sim_time')}],
             remappings=[('/scan', '/m20/scan'), ('/cmd_vel', '/m20/cmd_vel_raw'),
                         ('/d1_cmd', '/m20/unsupported_d1_actions')]),
        Node(package='jie_deamon', executable='m20_bridge', name='m20_bridge', output='screen',
             parameters=[config, {'dry_run': boolean('dry_run'),
                                  'commissioned': boolean('commissioned'),
                                  'control_profile': string('control_profile'),
                                  'axis_enable_y': boolean('axis_enable_y'),
                                  'axis_enable_yaw': boolean('axis_enable_yaw'),
                                  'allow_reverse': boolean('allow_reverse'),
                                  'gait_min_policy': string('gait_min_policy'),
                                  'require_preflight': boolean('require_preflight'),
                                  'enable_operator': boolean('enable_web'),
                                  'operator_token_file': LaunchConfiguration('operator_token_file'),
                                  'operator_page': os.path.join(share, 'web', 'm20.html'),
                                  'operator_host': LaunchConfiguration('operator_host'),
                                  'operator_port': ParameterValue(LaunchConfiguration('operator_port'),
                                                                  value_type=int),
                                  'android_port': ParameterValue(LaunchConfiguration('android_port'),
                                                                 value_type=int),
                                  'use_sim_time': boolean('use_sim_time')}]),
    ])
