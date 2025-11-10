#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明启动参数
    device_path_arg = DeclareLaunchArgument(
        'device_path',
        default_value='',
        description='Specific device path to scan (e.g., /dev/ttyACM0). Leave empty to scan all devices.'
    )
    
    scan_usb_arg = DeclareLaunchArgument(
        'scan_usb',
        default_value='true',
        description='Whether to scan USB devices'
    )
    
    scan_serial_arg = DeclareLaunchArgument(
        'scan_serial',
        default_value='true',
        description='Whether to scan serial devices'
    )
    
    publish_info_arg = DeclareLaunchArgument(
        'publish_info',
        default_value='false',
        description='Whether to publish device info to ROS2 topic'
    )

    # 设备扫描器节点
    device_scanner_node = Node(
        package='dm_tools',
        executable='device_scanner_node',
        name='device_scanner',
        output='screen',
        parameters=[{
            'device_path': LaunchConfiguration('device_path'),
            'scan_usb': LaunchConfiguration('scan_usb'),
            'scan_serial': LaunchConfiguration('scan_serial'),
            'publish_info': LaunchConfiguration('publish_info')
        }]
    )

    return LaunchDescription([
        device_path_arg,
        scan_usb_arg,
        scan_serial_arg,
        publish_info_arg,
        device_scanner_node
    ])