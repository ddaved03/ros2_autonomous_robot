from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonomous_robot',
            executable='fake_scan_publisher',
            name='fake_scan_publisher',
            output='screen'
        ),
        Node(
            package='autonomous_robot',
            executable='controller_node',
            name='controller_node',
            output='screen',
            parameters=[
                {'stop_distance': 0.5},
                {'forward_speed': 0.2},
                {'turn_speed': 0.8},
            ]
        ),
        Node(
            package='autonomous_robot',
            executable='cmd_vel_monitor',
            name='cmd_vel_monitor',
            output='screen'
        ),
    ])

