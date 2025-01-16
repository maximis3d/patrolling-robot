# Patrolling Robot

## Install:

1. Create venv
2. Source venv
3. Pip install requirements
4. `colcon build --symlink-install` to install application
4. `source insstall/setup.bash` to source applicaiton


## Running:


### Creating baseline:

1. Launch simulation world: `ros2 launch patrol_robot_simulation patrol_robot.launch.py`
2. Launch nav2bring_up for SLAM Map: `ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=src/patrolling_robot_simulation/maps/map.yaml`
3. Run waypoint node: `ros2 run yolo_detection_node waypoint_follower_node`
4. Run Baseline: `ros2 run yolo_detection_node baseline_manager_node`


### Patrolling Node:
1. Launch simulation world: `ros2 launch patrol_robot_simulation patrol_robot.launch.py`
2. Launch nav2bring_up for SLAM Map: `ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=src/patrolling_robot_simulation/maps/map.yaml`
3. Run waypoint node: `ros2 run yolo_detection_node waypoint_follower_node`
4. Run Baseline: `ros2 run yolo_detection_node patrolling_yolo_node`

