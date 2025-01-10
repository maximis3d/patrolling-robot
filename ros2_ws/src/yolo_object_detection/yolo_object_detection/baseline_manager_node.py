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
        self.image_save_path = "baseline_images"
        self.baseline_objects = self.load_baseline()
        os.makedirs(self.image_save_path, exist_ok=True)

        self.subscription = self.create_subscription(
            String, '/baseline_update', self.baseline_callback, 10)
        self.image_subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.pose_subscription = self.create_subscription(
            Pose, '/robot_pose', self.pose_callback, 10)

        self.latest_image = None
        self.latest_pose = None

        self.get_logger().info("Baseline Manager Initialized.")

    def load_baseline(self):
        """Load the baseline from a file if it exists."""
        if os.path.exists(self.baseline_file_path):
            try:
                with open(self.baseline_file_path, 'r') as file:
                    baseline_data = json.load(file)
                self.get_logger().info("Loaded existing baseline.")
                return baseline_data
            except json.JSONDecodeError:
                self.get_logger().warn("Failed to decode baseline file. Starting agaiin.")
        return {}

    def save_baseline(self):
        """Save the baseline to a file."""
        try:
            with open(self.baseline_file_path, 'w') as file:
                json.dump(self.baseline_objects, file)
            self.get_logger().info("Baseline successfully saved.")
        except Exception as e:
            self.get_logger().error(f"Failed to save baseline: {str(e)}")

    def image_callback(self, msg):
        """Save the latest image for potential baseline update."""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def pose_callback(self, msg):
        """Save the latest robot pose."""
        self.latest_pose = {
            "position": {
                "x": msg.position.x,
                "y": msg.position.y,
                "z": msg.position.z
            },
            "orientation": {
                "x": msg.orientation.x,
                "y": msg.orientation.y,
                "z": msg.orientation.z,
                "w": msg.orientation.w
            }
        }

    def baseline_callback(self, msg):
        """Update the baseline with a new detection."""
        try:
            detection_data = json.loads(msg.data)

            for class_name, boxes in detection_data.items():
                if class_name not in self.baseline_objects:
                    self.baseline_objects[class_name] = []

                for box in boxes:
                    self.baseline_objects[class_name].append({
                        "bounding_box": box,
                        "pose": self.latest_pose
                    })
                    # Save the image with a unique filename
                    img_path = os.path.join(self.image_save_path, f"{class_name}_{box[0]}_{box[1]}.jpg")
                    cv2.imwrite(img_path, self.latest_image)
                    self.get_logger().info(f"Saved baseline image: {img_path}")

            self.save_baseline()

        except json.JSONDecodeError:
            self.get_logger().error("Invalid baseline data received.")

def main(args=None):
    rclpy.init(args=args)
    node = BaselineManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
