import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time

class PatrollingRobotNode(Node):
    def __init__(self):
        super().__init__("patrolling_robot_node")

        # Initialize the navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.get_logger().info("Patrolling robot initialized.")

        # Define waypoints (x, y, yaw) as PoseStamped messages
        self.waypoints = [
            self.create_waypoint(2.0, 2.0, 0.0),
            self.create_waypoint(-2.0, 2.0, 0.0),
            self.create_waypoint(2.0, -2.0, 0.0),
            self.create_waypoint(-2.0, -2.0, 0.0)
        ]

        self.current_waypoint_index = 0
        self.patrol()

    def create_waypoint(self, x, y, yaw):
        """Helper to create a PoseStamped waypoint"""
        waypoint = PoseStamped()
        waypoint.header.frame_id = "map"
        waypoint.header.stamp = self.get_clock().now().to_msg()
        waypoint.pose.position.x = x
        waypoint.pose.position.y = y
        waypoint.pose.orientation.w = 1.0  
        return waypoint

    def patrol(self):
        """Continuously patrol through waypoints."""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation server not available!")
            return

        # Send goal to the current waypoint
        self.send_goal_to_waypoint(self.waypoints[self.current_waypoint_index])

    def send_goal_to_waypoint(self, waypoint):
        """Send a navigation goal to the robot."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint
        self.get_logger().info(f"Moving to waypoint {self.current_waypoint_index}")

        # Send goal and wait for result
        self.nav_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback when a goal is accepted/rejected."""
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info("Goal accepted, moving...")
            goal_handle.get_result_async().add_done_callback(self.goal_completed_callback)
        else:
            self.get_logger().error("Goal rejected by the server.")

    def goal_completed_callback(self, future):
        """Callback when the robot reaches the waypoint."""
        self.get_logger().info("Reached waypoint successfully!")
        
        self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
        time.sleep(2) 
        self.patrol()

def main(args=None):
    rclpy.init(args=args)
    node = PatrollingRobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
