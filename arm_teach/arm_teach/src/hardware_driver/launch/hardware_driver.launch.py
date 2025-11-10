#!/usr/bin/env python3

"""
Hardware Driver Launch File
启动硬件驱动节点，控制 Raytron 电机和夹爪
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明启动参数
    device_path_arg = DeclareLaunchArgument(
        'device_path',
        default_value='/dev/ttyACM0',
        description='USB device path for DM-CANFD adapter'
    )

    nom_baud_arg = DeclareLaunchArgument(
        'nom_baud',
        default_value='1000000',
        description='Nominal baudrate (1Mbps)'
    )

    dat_baud_arg = DeclareLaunchArgument(
        'dat_baud',
        default_value='5000000',
        description='Data baudrate for CANFD (5Mbps)'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='500.0',
        description='Joint state publish rate in Hz'
    )

    # 创建节点
    hardware_driver_node = Node(
        package='hardware_driver',
        executable='hardware_driver',
        name='hardware_driver_node',
        output='screen',
        parameters=[
            {'device_path': LaunchConfiguration('device_path')},
            {'nom_baud': LaunchConfiguration('nom_baud')},
            {'dat_baud': LaunchConfiguration('dat_baud')},
            {'publish_rate': LaunchConfiguration('publish_rate')},
        ],
        remappings=[
            ('joint_command', 'joint_command'),
            ('joint_states', 'joint_states'),
        ]
    )

    return LaunchDescription([
        device_path_arg,
        nom_baud_arg,
        dat_baud_arg,
        publish_rate_arg,
        hardware_driver_node,
    ])
