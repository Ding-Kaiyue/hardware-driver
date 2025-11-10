#!/usr/bin/env python3

import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def detect_device_and_launch(context):
    """检测设备序列号并启动右臂机械臂控制器"""

    # 获取launch参数
    device_path = LaunchConfiguration('device_path').perform(context)
    tty_device = LaunchConfiguration('tty_device').perform(context)
    control_frequency = LaunchConfiguration('control_frequency').perform(context)
    
    print(f"\n=== Right Arm Smart Device Detection ===")
    
    try:
        # 先尝试使用dm_tools检测设备
        if device_path:
            print(f"Scanning specific device: {device_path}")
            cmd = ['timeout', '5', 'ros2', 'run', 'dm_tools', 'device_scanner_node', 
                   '--ros-args', '-p', f'device_path:={device_path}', '-p', 'publish_info:=false']
        else:
            print(f"Scanning all USB2CANFD devices...")
            cmd = ['timeout', '5', 'ros2', 'run', 'dm_tools', 'device_scanner_node', 
                   '--ros-args', '-p', 'publish_info:=false']
        
        # 执行检测命令
        result = subprocess.run(cmd, capture_output=True, text=True)
        serial_number = None
        detected_device_path = None
        
        # 解析输出中的序列号和设备路径
        if result.returncode == 0:
            for line in result.stderr.split('\n'):  # ROS2日志通常在stderr
                if 'SN:' in line:
                    serial_number = line.split('SN:')[-1].strip()
                if 'Device path:' in line:
                    detected_device_path = line.split('Device path:')[-1].strip()
                    break
        
        # 如果没有检测到，使用默认值
        if not serial_number:
            serial_number = "A4F890D5ACE224020802CF2D55081601"  # 默认序列号
            print(f"⚠️  Using default serial number: {serial_number}")
        else:
            print(f"✅ Detected device serial number: {serial_number}")
            
        # 输出设备映射信息
        if detected_device_path:
            print(f"🔗 Device mapping: {detected_device_path} <-> Right Arm (SN: {serial_number})")
        elif device_path:
            print(f"🔗 Device mapping: {device_path} <-> Right Arm (SN: {serial_number})")
        
        print(f"🚀 Starting right arm controller...")
        
        # 创建右臂机械臂控制器节点
        arm_controller_node = Node(
            package='dm_motor',
            executable='arm_controller_node',
            name='arm_controller_R',
            parameters=[{
                'device_serial_number': serial_number,
                'tty_device': tty_device,
                'control_frequency': float(control_frequency),
                'position_tolerance': 0.01,
                'velocity_limit': 5.0,
                'kp_gains': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 无阻力模式 - 无位置控制力
                'kd_gains': [0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005]   # 无阻力模式 - 无阻尼力
            }],
            output='screen',
            emulate_tty=True,
        )
        
        return [
            LogInfo(msg=f"Using serial number: {serial_number}"),
            arm_controller_node
        ]
        
    except Exception as e:
        print(f"❌ Detection error: {e}")
        print(f"Using default serial number...")
        
        # 失败时使用默认序列号
        arm_controller_node = Node(
            package='dm_motor',
            executable='arm_controller_node',
            name='arm_controller_R',
            parameters=[{
                'device_serial_number': "A4F890D5ACE224020802CF2D55081601",
                'tty_device': tty_device,
                'control_frequency': float(control_frequency),
            }],
            output='screen',
            emulate_tty=True,
        )
        
        return [arm_controller_node]


def generate_launch_description():
    # Launch arguments
    device_path_arg = DeclareLaunchArgument(
        'device_path',
        default_value='',
        description='Specific device path to scan (e.g., /dev/ttyACM0). Leave empty to scan all USB devices.'
    )

    tty_device_arg = DeclareLaunchArgument(
        'tty_device',
        default_value='',
        description='TTY device to map to USB2CANFD device (e.g., /dev/ttyACM0). Leave empty to use device_serial_number.'
    )
    
    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='1000.0',
        description='Control loop frequency in Hz'
    )
    
    # 智能检测和启动
    smart_detection_action = OpaqueFunction(function=detect_device_and_launch)
    
    return LaunchDescription([
        device_path_arg,
        tty_device_arg,
        control_frequency_arg,
        smart_detection_action,
    ])