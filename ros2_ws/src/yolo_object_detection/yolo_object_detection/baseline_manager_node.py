import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from cv_bridge import CvBridge
import json
import os
import cv2
import time

class BaselineManagerNode(Node):
    def __init__(self):
        super().__init__('baseline_manager_node')

        self.bridge = CvBridge()
        self.baseline_file_path = self.declare_parameter("baseline_file", "baseline.json").get_parameter_value().string_value
        self.image_save_path = self.declare_parameter("image_save_path", "baseline_images").get_parameter_value().string_value
        self.baseline_objects = self.load_baseline()
        os.makedirs(self.image_save_path, exist_ok=True)

        self.subscription = self.create_subscription(String, '/baseline_update', self.baseline_callback, 10)
        self.image_subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.pose_subscription = self.create_subscription(Pose, '/robot_pose', self.pose_callback, 10)

        self.latest_image = None
        self.latest_pose = None

        self.get_logger().info("Baseline Manager Initialized.")

    def load_baseline(self):
        if os.path.exists(self.baseline_file_path):
            try:
                with open(self.baseline_file_path, 'r') as file:
                    baseline_data = json.load(file)
                self.get_logger().info("Loaded existing baseline.")
                return baseline_data
            except json.JSONDecodeError:
                self.get_logger().warn("Failed to decode baseline file. Starting again.")
        return {}

    def save_baseline(self):
        try:
            with open(self.baseline_file_path, 'w') as file:
                json.dump(self.baseline_objects, file)
            self.get_logger().info("Baseline successfully saved.")
        except Exception as e:
            self.get_logger().error(f"Failed to save baseline: {str(e)}")

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")

    def pose_callback(self, msg):
        self.latest_pose = {
            "position": {"x": msg.position.x, "y": msg.position.y, "z": msg.position.z},
            "orientation": {"x": msg.orientation.x, "y": msg.orientation.y, "z": msg.orientation.z, "w": msg.orientation.w}
        }

    def baseline_callback(self, msg):
        if self.latest_image is None or self.latest_pose is None:
            self.get_logger().warn("No image or pose data available.")
            return

        try:
            detection_data = json.loads(msg.data)
            for class_name, boxes in detection_data.items():
                if class_name not in self.baseline_objects:
                    self.baseline_objects[class_name] = []

                for box in boxes:
                    box = [float(coord) for coord in box]
                    self.baseline_objects[class_name].append({"bounding_box": box, "pose": self.latest_pose})
                    x1, y1, x2, y2 = map(int, box)
                    image_copy = self.latest_image.copy()
                    cv2.rectangle(image_copy, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(image_copy, class_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    timestamp = int(time.time())
                    img_path = os.path.join(self.image_save_path, f"{class_name}_{x1}_{y1}_{timestamp}.jpg")
                    cv2.imwrite(img_path, image_copy)
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
