# baseline_manager_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from cv_bridge import CvBridge
import json
import os
import cv2

class BaselineManagerNode(Node):
    def __init__(self):
        super().__init__('baseline_manager_node')
        self.bridge = CvBridge()
        self.baseline_file_path = "baseline.json"
        self.baseline_objects = self.load_baseline()

        self.mode_subscription = self.create_subscription(String, '/robot_mode', self.mode_callback, 10)
        self.image_subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.latest_image = None
        self.current_mode = "baseline"

    def mode_callback(self, msg):
        self.current_mode = msg.data

    def load_baseline(self):
        if os.path.exists(self.baseline_file_path):
            with open(self.baseline_file_path, 'r') as file:
                return json.load(file)
        return {}

    def save_baseline(self):
        with open(self.baseline_file_path, 'w') as file:
            json.dump(self.baseline_objects, file)

    def image_callback(self, msg):
        if self.current_mode == "baseline":
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Dummy detection logic (replace with YOLO inference if needed)
            self.get_logger().info("Captured image during baseline.")

def main(args=None):
    rclpy.init(args=args)
    node = BaselineManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
