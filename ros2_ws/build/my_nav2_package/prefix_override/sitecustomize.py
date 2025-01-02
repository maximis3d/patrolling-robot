import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/max/patrolling-robot/ros2_ws/install/my_nav2_package'
