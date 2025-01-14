# patrolling_yolo_node.py (Updated with Anomaly Handling)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import json
import math

class PatrollingYOLONode(Node):
    def __init__(self):
        super().__init__("patrolling_yolo_node")

        # Initialize variables
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")
        self.baseline_objects = self.load_baseline()
        self.current_mode = "patrol"
        self.latest_pose = None
        self.confidence_threshold = 0.5

        # Subscribers and Publishers
        self.mode_subscription = self.create_subscription(String, '/robot_mode', self.mode_callback, 10)
        self.image_subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.pose_subscription = self.create_subscription(Pose, '/robot_pose', self.pose_callback, 10)
        self.anomaly_publisher = self.create_publisher(String, '/anomaly_alert', 10)

    def load_baseline(self):
        try:
            with open("baseline.json", "r") as file:
                return json.load(file)
        except (FileNotFoundError, json.JSONDecodeError):
            self.get_logger().warn("No valid baseline found.")
            return {}

    def mode_callback(self, msg):
        self.current_mode = msg.data
        self.get_logger().info(f"Mode updated: {self.current_mode}")

    def pose_callback(self, msg):
        self.latest_pose = {
            "x": msg.position.x,
            "y": msg.position.y,
            "z": msg.position.z
        }

    def image_callback(self, msg):
        if self.current_mode != "patrol":
            return
        
        # Convert image and perform YOLO detection
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(cv_image)
        detections = results[0].boxes.data.cpu().numpy()
        current_detections = {}

        # Process YOLO results
        for detection in detections:
            x1, y1, x2, y2, conf, cls = detection
            if conf < self.confidence_threshold:
                continue

            class_name = self.model.names[int(cls)]
            bbox = [float(x1), float(y1), float(x2), float(y2)]

            if class_name not in current_detections:
                current_detections[class_name] = [bbox]
            else:
                current_detections[class_name].append(bbox)

        # Compare detections with baseline
        self.compare_with_baseline(current_detections)

    def compare_with_baseline(self, current_detections):
        """Compare detected objects with the baseline to detect anomalies."""
        anomalies_detected = False

        # Check for new or missing objects
        for class_name, boxes in current_detections.items():
            if class_name not in self.baseline_objects:
                self.report_anomaly(f"New object detected: {class_name}")
                anomalies_detected = True
            else:
                for box in boxes:
                    if not self.is_object_consistent(class_name, box):
                        self.report_anomaly(f"Object moved: {class_name}")
                        anomalies_detected = True

        # Check for missing objects
        for class_name in self.baseline_objects.keys():
            if class_name not in current_detections:
                self.report_anomaly(f"Object missing: {class_name}")
                anomalies_detected = True

        # Optional: Stop the robot if an anomaly is detected
        if anomalies_detected:
            self.stop_robot()

    def is_object_consistent(self, class_name, box):
        """Check if an object's position is consistent with the baseline."""
        for baseline_entry in self.baseline_objects[class_name]:
            baseline_box = baseline_entry['bounding_box']
            baseline_pose = baseline_entry['pose']['position']

            # Check if the detected object is within a certain distance from the baseline
            baseline_x = (baseline_box[0] + baseline_box[2]) / 2
            baseline_y = (baseline_box[1] + baseline_box[3]) / 2
            detected_x = (box[0] + box[2]) / 2
            detected_y = (box[1] + box[3]) / 2

            distance = math.sqrt((detected_x - baseline_x)**2 + (detected_y - baseline_y)**2)
            if distance < 1.0:  # Tolerance for minor variations
                return True
        return False

    def report_anomaly(self, message):
        """Publish an anomaly alert."""
        self.get_logger().warn(message)
        alert_msg = String()
        alert_msg.data = message
        self.anomaly_publisher.publish(alert_msg)

    def stop_robot(self):
        """Stop the robot in case of an anomaly."""
        self.get_logger().info("Stopping the robot due to detected anomaly.")
        stop_msg = String()
        stop_msg.data = "stop"
        self.anomaly_publisher.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PatrollingYOLONode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
