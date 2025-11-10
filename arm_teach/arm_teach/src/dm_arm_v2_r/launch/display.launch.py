from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('dm_arm_v2_r'),
            'urdf',
            'dm_arm_v2_r.urdf'
        ])
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_R',
        parameters=[{
            'robot_description': ParameterValue(
                robot_description_content, value_type=str
            )
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=''),
        DeclareLaunchArgument('gui', default_value='true'),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_R',
            parameters=[{'use_gui': LaunchConfiguration('gui')}]
        ),

        robot_state_publisher_node,

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_R',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('dm_arm_v2_r'),
                'urdf.rviz'
            ])],
            output='screen'
        )
    ])

