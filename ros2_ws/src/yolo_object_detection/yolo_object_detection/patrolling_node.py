import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import json
import numpy as np

class PatrollingYOLONode(Node):
    def __init__(self):
        super().__init__('patrolling_yolo_node')
        
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.pose_subscription = self.create_subscription(
            Pose, '/robot_pose', self.pose_callback, 10)

        # Ensure image publishing is active
        self.report_publisher = self.create_publisher(String, '/object_detection_report', 10)
        self.baseline_publisher = self.create_publisher(String, '/baseline_update', 10)
        self.image_publisher = self.create_publisher(Image, '/yolo_image_with_boxes', 10)

        self.baseline_objects = self.load_baseline()
        self.latest_pose = None

    def load_baseline(self):
        try:
            with open('baseline.json', 'r') as file:
                return json.load(file)
        except (FileNotFoundError, json.JSONDecodeError):
            self.get_logger().warn("No valid baseline found.")
            return {}

    def pose_callback(self, msg):
        self.latest_pose = {
            "position": {"x": msg.position.x, "y": msg.position.y, "z": msg.position.z},
            "orientation": {"x": msg.orientation.x, "y": msg.orientation.y, "z": msg.orientation.z, "w": msg.orientation.w}
        }

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(cv_image)
        detections = results[0].boxes.data.cpu().numpy()
        current_detections = {}

        # Draw bounding boxes directly on the image
        for detection in detections:
            x1, y1, x2, y2, conf, cls = detection
            class_name = self.model.names[int(cls)]
            bbox = [float(x1), float(y1), float(x2), float(y2)]

            # Draw bounding box on the image
            cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(cv_image, f"{class_name} {conf:.2f}", (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Collect the detection data for comparison
            if class_name not in current_detections:
                current_detections[class_name] = [bbox]
            else:
                current_detections[class_name].append(bbox)

        # **Publish the annotated image with bounding boxes**
        image_with_boxes_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.image_publisher.publish(image_with_boxes_msg)

        # Check for missing or new objects
        self.compare_with_baseline(current_detections)

    def compare_with_baseline(self, current_detections):
        for class_name, boxes in current_detections.items():
            if class_name not in self.baseline_objects:
                self.log_unexpected_object(class_name, boxes)

    def log_unexpected_object(self, class_name, boxes):
        report_msg = String()
        report_msg.data = json.dumps({class_name: boxes})
        self.baseline_publisher.publish(report_msg)
        self.get_logger().warn(f"New object detected: {class_name} at {boxes}")

def main(args=None):
    rclpy.init(args=args)
    node = PatrollingYOLONode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
