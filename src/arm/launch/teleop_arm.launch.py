from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    joy_params = LaunchConfiguration('joy_params')

    default_joy_params = PathJoinSubstitution([
        FindPackageShare('arm'), 'config', 'teleop_twist_joy.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('joy_params', default_value=default_joy_params),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[joy_params],
            output='screen'
        ),
        Node(
            package='arm',
            executable='twist_to_arm.py',   # this will now exist in lib/arm
            name='twist_to_arm',
            output='screen',
            parameters=[
                {'input_twist': '/cmd_vel'},
                {'rate': 60.0}
            ]
        ),
    ])
