import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import SetBool

class StateManagerNode(Node):
    def __init__(self):
        super().__init__('state_manager_node')
        self.state_publisher = self.create_publisher(String, '/robot_mode', 10)
        self.mode = "baseline"

        # Create the ROS2 service for manual mode switching
        self.mode_service = self.create_service(
            SetBool, 'switch_mode', self.handle_mode_switch)
        
        self.publish_mode()
        self.get_logger().info(f"StateManager initialized. Starting in {self.mode} mode (manual control only).")

    def handle_mode_switch(self, request, response):
        """ROS2 Service Handler for manual mode switching."""
        if request.data:  
            self.mode = "patrol"
        else:
            self.mode = "baseline"
        
        # Publish the new mode
        self.publish_mode()
        
        response.success = True
        response.message = f"Mode set to: {self.mode}"
        self.get_logger().info(f"[MANUAL] Mode set to: {self.mode}")
        return response

    def publish_mode(self):
        """Helper function to publish the current mode."""
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
