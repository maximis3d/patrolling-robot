import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import json

class PatrollingYOLONode(Node):
    def __init__(self):
        super().__init__('patrolling_yolo_node')
        
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.pose_subscription = self.create_subscription(
            Pose, '/robot_pose', self.pose_callback, 10)

        self.report_publisher = self.create_publisher(String, '/object_detection_report', 10)
        self.baseline_publisher = self.create_publisher(String, '/baseline_update', 10)
        self.image_publisher = self.create_publisher(Image, '/yolo_image_with_boxes', 10)

        self.baseline_objects = self.load_baseline()
        self.latest_pose = None

    def load_baseline(self):
        try:
            with open('baseline.json', 'r') as file:
                baseline_data = json.load(file)
                self.get_logger().info("Baseline successfully loaded.")
                return baseline_data
        except (FileNotFoundError, json.JSONDecodeError):
            self.get_logger().warn("No valid baseline found.")
            return {}

    def pose_callback(self, msg):
        """Update the latest robot pose."""
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

    def image_callback(self, msg):
        """Compare the current image against the baseline."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(cv_image)
        detections = results[0].boxes.data.cpu().numpy()
        current_detections = {}

        for detection in detections:
            x1, y1, x2, y2, conf, cls = detection
            class_name = self.model.names[int(cls)]
            bbox = [x1, y1, x2, y2]

            if class_name not in current_detections:
                current_detections[class_name] = [bbox]
            else:
                current_detections[class_name].append(bbox)

        # Check for missing or new objects
        self.compare_with_baseline(current_detections)

    def compare_with_baseline(self, current_detections):
        """Compare current detections with the stored baseline."""
        for class_name, boxes in current_detections.items():
            if class_name not in self.baseline_objects:
                self.log_unexpected_object(class_name, boxes)
            else:
                # Check for missing objects
                for baseline_entry in self.baseline_objects[class_name]:
                    if not self.is_object_present(baseline_entry["bounding_box"], boxes):
                        self.get_logger().warn(f"Object missing: {class_name}")

    def is_object_present(self, baseline_box, current_boxes):
        """Check if a baseline object is present in the current frame."""
        for box in current_boxes:
            if self.calculate_iou(baseline_box, box) > 0.5:
                return True
        return False

    def log_unexpected_object(self, class_name, boxes):
        """Log new objects detected."""
        for box in boxes:
            report_msg = String()
            report_msg.data = f"Unexpected Object Detected: {class_name} at {box}"
            self.report_publisher.publish(report_msg)
            self.get_logger().warn(report_msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = PatrollingYOLONode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
