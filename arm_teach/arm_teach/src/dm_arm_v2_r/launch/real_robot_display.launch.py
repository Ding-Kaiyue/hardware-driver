#!/usr/bin/env python3

import os
import sys
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import conditions


def get_right_arm_device_path():
    """
    自动获取右臂设备路径
    """
    try:
        # 查找设备识别脚本
        dm_tools_share = FindPackageShare(package='dm_tools').find('dm_tools')
        script_path = os.path.join(dm_tools_share, '../../src/dm_tools/scripts/device_identifier.py')
        
        # 如果脚本不存在，尝试其他可能的路径
        if not os.path.exists(script_path):
            # 尝试相对路径
            current_dir = os.path.dirname(os.path.abspath(__file__))
            script_path = os.path.join(current_dir, '../../../dm_tools/scripts/device_identifier.py')
        
        if os.path.exists(script_path):
            # 运行设备识别脚本
            result = subprocess.run([
                sys.executable, script_path, 
                '--action', 'get-path', 
                '--device-type', 'right_arm'
            ], capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                device_path = result.stdout.strip()
                print(f"[INFO] 自动识别到右臂设备: {device_path}")
                return device_path
            else:
                print(f"[WARN] 右臂设备自动识别失败: {result.stderr}")
        else:
            print(f"[WARN] 未找到设备识别脚本: {script_path}")
            
    except Exception as e:
        print(f"[WARN] 设备自动识别异常: {e}")
    
    print("[INFO] 使用默认设备路径，请手动指定 device_path 参数")
    return ''


def generate_launch_description():
    
    # 获取包路径
    pkg_share = FindPackageShare(package='dm_arm_v2_r').find('dm_arm_v2_r')
    
    # URDF文件路径
    urdf_file = os.path.join(pkg_share, 'urdf', 'dm_arm_v2_r.urdf')
    
    # 读取URDF内容
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Launch参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    # 自动获取右臂设备路径
    auto_device_path = get_right_arm_device_path()
    
    device_path_arg = DeclareLaunchArgument(
        'device_path',
        default_value=auto_device_path,
        description='Specific device path to scan (e.g., /dev/ttyACM0). Will auto-detect right arm device if empty.'
    )

    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='1000.0',
        description='Control loop frequency in Hz'
    )

    # Include DM Motor Controller Launch
    dm_motor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('dm_motor'),
                'launch',
                'smart_arm_controller.launch.py'
            ])
        ]),
        launch_arguments={
            'device_path': LaunchConfiguration('device_path'),
            'control_frequency': LaunchConfiguration('control_frequency'),
        }.items()
    )

    # Robot State Publisher节点 - 将joint_states转换为TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_R',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 50.0  # 发布频率50Hz
        }],
        remappings=[
            ('/joint_states', '/joint_states_R')
        ],
        output='screen'
    )

    # RViz节点用于可视化
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'real_robot_display.rviz')
    
    # 如果没有rviz配置文件，使用默认配置
    if not os.path.exists(rviz_config_file):
        rviz_config_file = ''
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_R',
        arguments=['-d', rviz_config_file] if rviz_config_file else [],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=conditions.IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )

    return LaunchDescription([
        # Launch参数
        use_sim_time_arg,
        use_rviz_arg,
        device_path_arg,
        control_frequency_arg,

        # 信息日志
        LogInfo(msg="=== Right Arm Real Robot Display ==="),
        LogInfo(msg=f"Auto-detected device path: {auto_device_path}"),
        LogInfo(msg="Starting DM Motor Controller..."),

        # 启动电机控制器
        dm_motor_launch,

        LogInfo(msg=f"Loading URDF: {urdf_file}"),
        LogInfo(msg="Subscribing to /joint_states from real robot"),
        LogInfo(msg="Publishing robot model to TF tree"),

        # 节点
        robot_state_publisher_node,
        rviz_node,

        LogInfo(msg="✅ Real robot display ready!"),
    ])
