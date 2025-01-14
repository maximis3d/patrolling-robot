# waypoint_follower_node.py
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from scipy.spatial.transform import Rotation as R

def euler_to_quaternion(roll, pitch, yaw):
    r = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
    return r.as_quat()

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.nav = BasicNavigator()
        self.current_mode = "baseline"
        
        self.mode_subscription = self.create_subscription(
            String, '/robot_mode', self.mode_callback, 10)

        # Predefined waypoints
        self.waypoints = [
            self.create_pose(0.0, -3.5, 0),
            self.create_pose(5.0, -3.0, 0),
            self.create_pose(6.0, -3.0, 0)
        ]

        self.nav.setInitialPose(self.create_pose(-2.0, -0.5, 0.0))
        self.nav.waitUntilNav2Active()

    def mode_callback(self, msg):
        """Switch mode between baseline and patrol."""
        self.current_mode = msg.data
        if self.current_mode == "baseline":
            self.get_logger().info("Following waypoints.")
            self.follow_waypoints()
        else:
            self.get_logger().info("Patrol mode active, stopping waypoint following.")

    def create_pose(self, x, y, yaw):
        q_x, q_y, q_z, q_w = euler_to_quaternion(0, 0, yaw)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w
        return pose

    def follow_waypoints(self):
        if self.current_mode == "baseline":
            for index, waypoint in enumerate(self.waypoints):
                self.get_logger().info(f"Navigating to waypoint {index + 1}")
                self.nav.followWaypoints([waypoint])
                while not self.nav.isTaskComplete():
                    pass
            self.get_logger().info("Completed all waypoints.")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
