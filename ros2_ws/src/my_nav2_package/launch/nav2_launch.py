from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='nav2_bringup',
            output='screen',
            parameters=[
                '/path/to/nav2_params.yaml'
            ]
        ),
        Node(
            package='my_nav2_package',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
        ),
    ])
