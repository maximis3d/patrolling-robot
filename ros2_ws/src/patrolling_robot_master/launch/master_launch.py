# patrolling_robot_master/launch/master_launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from os import path

def generate_launch_description():
    yolo_launch_file = path.join(
        get_package_share_directory("yolo_object_detection"),
        "launch",
        "yolo_launch.py"
    )
    turtlebot_launch_file = path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "launch",
        "turtlebot3_world.launch.py"
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(yolo_launch_file)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot_launch_file)
        )
    ])
