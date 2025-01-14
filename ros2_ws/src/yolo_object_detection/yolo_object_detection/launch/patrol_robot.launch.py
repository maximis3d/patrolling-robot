from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='yolo_object_detection', executable='state_manager_node'),
        Node(package='yolo_object_detection', executable='baseline_manager_node'),
        Node(package='yolo_object_detection', executable='patrolling_yolo_node'),
    ])