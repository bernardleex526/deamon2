import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'd1_bringup'
    
    # 获取 yaml 文件路径
    config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'd1_params.yaml'
    )

    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='d1_core',
            name='d1_core_node',
            # output='screen',
            # 加载参数文件
            parameters=[config_file]
        )
    ])