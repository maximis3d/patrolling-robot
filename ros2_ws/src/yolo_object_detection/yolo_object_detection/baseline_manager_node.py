import os
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class BaselineNode(Node):
    def __init__(self):
        super().__init__('baseline_node')

        # Initialize paths, load YOLO model
        self.bridge = CvBridge()
        self.baseline_file_path = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'baseline.json')
        os.makedirs(os.path.dirname(self.baseline_file_path), exist_ok=True)

        # Load YOLOv8 model from ultralytics
        self.model = YOLO('yolov8n.pt')  # Using YOLOv8 Nano pre-trained model

        # Subscribe to the robot's Camera Feed
        self.image_subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Store latest image data
        self.latest_image = None

        # Initialize baseline data (empty list initially)
        self.baseline_objects = {"detections": []}

        self.get_logger().info("Baseline Node Initialized with YOLOv8")

    def image_callback(self, msg):
        """Receive camera image and perform object detection."""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Run YOLOv8 object detection
        results = self.model(self.latest_image)

        # Process detections
        detected_objects = []
        for result in results[0].boxes.data.tolist():  # Each result corresponds to a detection box
            class_id = int(result[5])  # class ID (for the detected object)
            confidence = float(result[4])  # detection confidence
            x1, y1, x2, y2 = map(int, result[:4])  # bounding box coordinates

            if confidence > 0.75:  # Only consider high-confidence detections
                detected_object = {
                    "class": results[0].names[class_id],  # Access the names attribute from result[0]
                    "bounding_box": [x1, y1, x2, y2],
                    "confidence": confidence
                }
                detected_objects.append(detected_object)

                # Draw bounding box on the image
                label = f"{results[0].names[class_id]} {confidence:.2f}"
                cv2.rectangle(self.latest_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(self.latest_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Add the new detections to the baseline objects list
        if detected_objects:
            self.baseline_objects["detections"].extend(detected_objects)
            self.save_baseline()

        # Show the image with detections
        cv2.imshow("Detected Objects", self.latest_image)
        cv2.waitKey(1)

    def save_baseline(self):
        """Save the baseline data to baseline.json."""
        with open(self.baseline_file_path, 'w') as file:
            json.dump(self.baseline_objects, file, indent=4)
        self.get_logger().info(f"Baseline data saved to {self.baseline_file_path}")

def main(args=None):
    rclpy.init(args=args)
    node = BaselineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
