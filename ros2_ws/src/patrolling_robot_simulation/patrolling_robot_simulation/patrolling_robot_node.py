import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles to a quaternion.

    Parameters:
    roll (float): Rotation around the x-axis.
    pitch (float): Rotation around the y-axis.
    yaw (float): Rotation around the z-axis.

    Returns:
    list: Quaternion [x, y, z, w].
    """
    r = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
    q = r.as_quat()
    return q  # Returns [x, y, z, w] order

class WaypointFollower(Node):
    """
    ROS2 Node for following a sequence of waypoints using Nav2.
    """
    def __init__(self):
        super().__init__('waypoint_follower')
        self.nav = BasicNavigator()

        # Declare the waypoints parameter and set default if not provided
        self.declare_parameter('waypoints', [])
        waypoint_list_param = self.get_parameter('waypoints').get_parameter_value().string_value

        # Convert the string parameter into a list of floats
        try:
            waypoint_list = [float(x) for x in waypoint_list_param.strip('[]').split(',')]
        except ValueError:
            waypoint_list = []

        # If waypoints were passed as a parameter, convert them to PoseStamped objects
        if waypoint_list:
            waypoints = []
            for i in range(0, len(waypoint_list), 3):
                x, y, yaw = waypoint_list[i], waypoint_list[i+1], waypoint_list[i+2]
                waypoints.append(self.create_pose(x, y, yaw))
        else:
            waypoints = [
                self.create_pose(0.0, 4.0, 1.57),
                self.create_pose(4.4, 0.33, -1.57),
                self.create_pose(5.15, -0.31, 3.14)
            ]

        self.waypoints = waypoints

        # Set the initial pose of the robot
        initial_pose = self.create_pose(-2.0, -0.5, 0.0)
        self.nav.setInitialPose(initial_pose)

        # Wait for Nav2 to become active before proceeding
        self.nav.waitUntilNav2Active()

        self.follow_waypoints()

    def create_pose(self, x, y, yaw):
        """
        Create a PoseStamped message for a given position and orientation.

        Parameters:
        x (float): X-coordinate.
        y (float): Y-coordinate.
        yaw (float): Rotation around the z-axis.

        Returns:
        PoseStamped: The created pose message.
        """
        q_x, q_y, q_z, q_w = euler_to_quaternion(0, 0, yaw)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w
        return pose

    def follow_waypoints(self):
        """
        Command the robot to follow the defined waypoints and log the index.
        """
        for index, waypoint in enumerate(self.waypoints):
            self.get_logger().info(f"Navigating to waypoint {index + 1}: ({waypoint.pose.position.x}, {waypoint.pose.position.y})")
            self.nav.followWaypoints([waypoint])  # Send one waypoint at a time

            while not self.nav.isTaskComplete():
                pass

        self.get_logger().info("Completed all waypoints!")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
