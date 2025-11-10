#!/usr/bin/env python3

"""
Hardware Driver Launch File with Config File Support
使用配置文件启动硬件驱动节点（支持Kp/Kd配置）
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 获取配置文件路径
    config_file = os.path.join(
        get_package_share_directory('hardware_driver'),
        'config',
        'hardware_driver_params.yaml'
    )

    # 创建节点
    hardware_driver_node = Node(
        package='hardware_driver',
        executable='hardware_driver',
        name='hardware_driver_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('joint_command', 'joint_command'),
            ('joint_states', 'joint_states'),
        ]
    )

    return LaunchDescription([
        hardware_driver_node,
    ])
