import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_util import SimpleNavigator

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.navigator = SimpleNavigator()

        # Wait for Nav2 to be active
        self.navigator.wait_until_nav2_active()

        # Example: Send a goal
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = 2.0
        goal_pose.pose.position.y = 2.0
        goal_pose.pose.orientation.w = 1.0

        self.navigator.go_to_pose(goal_pose)

        result = self.navigator.get_result()
        if result == 0:
            self.get_logger().info("Goal succeeded!")
        else:
            self.get_logger().warn("Goal failed!")

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
