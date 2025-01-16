import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles to a quaternion."""
    r = R.from_euler("xyz", [roll, pitch, yaw], degrees=False)
    return r.as_quat()

class WaypointFollower(Node):
    """
    ROS2 Node to independently follow waypoints using Nav2.
    """
    def __init__(self):
        super().__init__("waypoint_follower")
        self.nav = BasicNavigator()

        # Predefined waypoints to be followed independently
        self.waypoints = [
            self.create_pose(0.0, -3.5, 0),
            self.create_pose(5.0, -3.0, 0),
            self.create_pose(6.0, -3.0, 0),
            self.create_pose(5.15, -0.31, 3.14),
            self.create_pose(-1.0, 1.0, 0.0),
            self.create_pose(-2.0, -0.5, 0.0),
            self.create_pose(-5.0, 0.0, 0.0),
            self.create_pose(-8.0, 0.0, 0.0),
            self.create_pose(-5.0, 0.0, 0.0),
            self.create_pose(-8.0, -2.5, 0.0),
            self.create_pose(-2.0, -0.5, 0.0)
        ]

        # Set initial pose for the robot
        self.nav.setInitialPose(self.create_pose(-2.0, -0.5, 0.0))

        # Wait for Nav2 to become active before proceeding
        self.get_logger().info("Waiting for Nav2 to become active...")
        self.nav.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active. Starting waypoint following...")

        # Start following the waypoints independently
        self.follow_waypoints()

    def create_pose(self, x, y, yaw):
        """Helper to create a PoseStamped message."""
        q_x, q_y, q_z, q_w = euler_to_quaternion(0, 0, yaw)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w
        return pose

    def follow_waypoints(self):
        """Command the robot to follow waypoints one by one."""
        for index, waypoint in enumerate(self.waypoints):
            self.get_logger().info(f"Navigating to waypoint {index + 1}: {waypoint.pose.position.x}, {waypoint.pose.position.y}")
            self.nav.followWaypoints([waypoint])

            # Wait until the robot reaches the current waypoint
            while not self.nav.isTaskComplete():
                pass

        self.get_logger().info("Completed all waypoints successfully!")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
