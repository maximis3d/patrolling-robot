from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="yolo_object_detection",
            executable="yolo_object_detection_node",
            name="yolo_object_detection_node",
            output="screen"
        )
    ])