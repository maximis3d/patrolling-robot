from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Baseline Manager Node
        Node(
            package="yolo_object_detection",
            executable="baseline_manager_node",
            name="baseline_manager_node",
            output="screen"
        ),
        
        # Patrolling Node (YOLO Object Detection)
        Node(
            package="yolo_object_detection",
            executable="patrolling_yolo_node",
            name="patrolling_yolo_node",
            output="screen"
        ),
        
        # Patrolling Robot Node (Navigation)
        Node(
            package="yolo_object_detection",
            executable="patrolling_robot_node",
            name="patrolling_robot_node",
            output="screen"
        ),
        
        # Launch Nav2 Stack
        Node(
            package="nav2_bringup",
            executable="bringup_launch.py",
            output="screen"
        )
    ])
