# state_manager_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StateManagerNode(Node):
    def __init__(self):
        super().__init__('state_manager_node')
        self.state_publisher = self.create_publisher(String, '/robot_mode', 10)
        self.timer = self.create_timer(15.0, self.toggle_mode)
        self.mode = "baseline"
        self.get_logger().info(f"StateManager initialized. Starting in {self.mode} mode.")

    def toggle_mode(self):
        """Switch between baseline and patrol modes automatically."""
        self.mode = "patrol" if self.mode == "baseline" else "baseline"
        msg = String()
        msg.data = self.mode
        self.state_publisher.publish(msg)
        self.get_logger().info(f"Switched mode to: {self.mode}")

def main(args=None):
    rclpy.init(args=args)
    node = StateManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
