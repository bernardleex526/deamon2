#!/usr/bin/env python3
"""
NEW-06: M20 仅算法/NavCmd 预览 launch（standalone）

- 只启动现成 pointcloud_to_laserscan + robot_nexus + nav_cmd_preview
- 不含 D1 bringup / 雷达驱动
- 本阶段为"不可驱动预览"：robot_nexus 的 m20_preview=true，
  不创建 /cmd_vel / /d1_cmd 发布者，机器不会运动
- FASTRTPS_DEFAULT_PROFILES_FILE 指向主代理复核过的只读诊断XML
  （/home/user/new_chase/config/diagnostic_udp_only.xml，本launch不修改它）

话题契约（默认，可用参数覆盖）：
- 点云输入: /new_chase/points（SDK drdds_cloud_export 输出；cloud_topic 仍可覆盖为
  /LIDAR/POINTS 等测试主题——原生 /LIDAR/POINTS 直连 ROS 尚未打通，NEW-11 起
  默认走 export→ROS 已验证链路）
- LaserScan: /new_chase/scan
- 预览raw Twist: /new_chase/cmd_vel_raw
- NavCmd 预览: /new_chase/nav_cmd_preview （标准 drdds/NavCmd 消息，仅供观测）
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# 同一进程环境局部DDS配置（只读文件，主代理已复核，禁止修改）
FASTRTPS_DEFAULT_PROFILES_FILE = os.environ.get(
    'FASTRTPS_DEFAULT_PROFILES_FILE',
    '/home/user/new_chase/config/diagnostic_udp_only.xml',
)


def generate_launch_description():
    cloud_topic = LaunchConfiguration('cloud_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    raw_topic = LaunchConfiguration('raw_topic')
    nav_preview_topic = LaunchConfiguration('nav_preview_topic')
    scan_frame_id = LaunchConfiguration('scan_frame_id')
    expected_scan_frame = LaunchConfiguration('expected_scan_frame')
    use_cloud_converter = LaunchConfiguration('use_cloud_converter')
    enable_web = LaunchConfiguration('enable_web')

    common_env = {'FASTRTPS_DEFAULT_PROFILES_FILE': FASTRTPS_DEFAULT_PROFILES_FILE}

    # NEW-06 P1: web_root必须用包share目录（run脚本cwd=/home/user/new_chase，
    # robot_nexus的autoDetectWebRoot候选./web与../share/jie_deamon/web均不可达）
    web_root = os.path.join(get_package_share_directory('jie_deamon'), 'web')

    # 现成点云转扫描工具（Foxy 自带）
    cloud_converter = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='new_chase_cloud_converter',
        output='screen',
        remappings=[
            ('cloud_in', cloud_topic),
            ('scan', scan_topic),
        ],
        parameters=[{
            # NEW-06 P1: target_frame默认空=沿用点云frame，避免重复TF链；
            # 初始滤波参数为静态估计（未标定，待实机校准）
            'target_frame': scan_frame_id,
            'min_height': -0.05,
            'max_height': 0.50,
            'angle_min': -3.141592653589793,
            'angle_max': 3.141592653589793,
            'angle_increment': 0.00872665,   # ~720点/圈
            'range_min': 0.10,
            'range_max': 20.0,
        }],
        additional_env=common_env,
        condition=IfCondition(use_cloud_converter),
    )

    robot_nexus_node = Node(
        package='jie_deamon',
        executable='robot_nexus',
        name='robot_nexus',
        output='screen',
        parameters=[{
            # M20 入口：仅算法/NavCmd预览，禁Android/动作/直控，机器不会运动
            'm20_preview': True,
            'active': True,
            'enable_web': enable_web,
            'enable_opencv': False,
            'scan_topic': scan_topic,
            'raw_topic': raw_topic,
            # NEW-06 P1: 输入物理frame校验独立参数（默认lidar_link=实机最新cloud frame），
            # 与converter target_frame(scan_frame_id)不混用；测试可经scan_frame_id或本参数校验
            'expected_scan_frame': expected_scan_frame,
            # M20 几何参数（静态估计未标定，不作为实机安全证明）
            'rotation_angle': 0.0,
            'follow_distance': 1.2,
            'rectangle_width': 0.7,
            'robot_frame_front': 0.46,
            'robot_frame_back': 0.46,
            'robot_frame_left': 0.31,
            'robot_frame_right': 0.31,
        }],
        additional_env=common_env,
    )

    nav_preview_node = Node(
        package='jie_deamon',
        executable='nav_cmd_preview.py',
        name='nav_cmd_preview',
        output='screen',
        parameters=[{
            'raw_topic': raw_topic,
            'preview_topic': nav_preview_topic,
        }],
        additional_env=common_env,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'cloud_topic', default_value='/new_chase/points',
            description='点云输入话题（NEW-11默认=SDK drdds_cloud_export输出；'
                        'override保留，可指向任意测试主题如/LIDAR/POINTS——'
                        '原生/LIDAR/POINTS直连ROS尚未打通)'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/new_chase/scan',
            description='LaserScan输出话题'),
        DeclareLaunchArgument(
            'raw_topic', default_value='/new_chase/cmd_vel_raw',
            description='预览raw Twist话题（robot_nexus发布→nav_cmd_preview订阅，双向一致）'),
        DeclareLaunchArgument(
            'nav_preview_topic', default_value='/new_chase/nav_cmd_preview',
            description='NavCmd预览话题（仅观测；重映射到真实控制话题将被节点exit 2拒绝）'),
        DeclareLaunchArgument(
            'scan_frame_id', default_value='',
            description='converter target_frame（空=沿用点云frame，避免重复TF链）'),
        DeclareLaunchArgument(
            'expected_scan_frame', default_value='lidar_link',
            description='robot_nexus输入物理frame校验（实机最新cloud frame=lidar_link）；空=不校验'),
        DeclareLaunchArgument(
            'use_cloud_converter', default_value='true',
            description='false=跳过pointcloud_to_laserscan，供合成LaserScan测试'),
        DeclareLaunchArgument(
            'enable_web', default_value='true',
            description='是否启用Web可视化'),
        robot_nexus_node,
        cloud_converter,
        nav_preview_node,
    ])
