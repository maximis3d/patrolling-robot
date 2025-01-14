# state_manager_node.py (Updated to create baseline.json on startup)
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import SetBool
import json
import os

class StateManagerNode(Node):
    def __init__(self):
        super().__init__('state_manager_node')
        self.state_publisher = self.create_publisher(String, '/robot_mode', 10)
        self.mode = "baseline"
        self.baseline_file_path = "baseline.json"

        # Ensure baseline.json exists or create it
        self.create_baseline_file()

        # ROS2 Service for manual mode switching
        self.mode_service = self.create_service(
            SetBool, 'switch_mode', self.handle_mode_switch)
        
        self.publish_mode()
        self.get_logger().info(f"StateManager initialized. Starting in {self.mode} mode (manual control only).")

    def create_baseline_file(self):
        """Ensure the baseline file exists."""
        if not os.path.exists(self.baseline_file_path):
            self.get_logger().info("Creating a new baseline.json file.")
            with open(self.baseline_file_path, 'w') as file:
                json.dump({}, file)

    def handle_mode_switch(self, request, response):
        """Manual mode switching using a ROS2 service."""
        self.mode = "patrol" if request.data else "baseline"
        self.publish_mode()
        response.success = True
        response.message = f"Mode set to: {self.mode}"
        self.get_logger().info(f"[MANUAL] Mode set to: {self.mode}")
        return response

    def publish_mode(self):
        """Publish the current mode."""
        msg = String()
        msg.data = self.mode
        self.state_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StateManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
