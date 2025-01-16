import os
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
from sort_tracker import Sort  # Using sort-tracker-py

class PatrolNode(Node):
    def __init__(self):
        super().__init__("patrol_node")

        # Initialize paths, load YOLO model, and set up SORT for object tracking
        self.bridge = CvBridge()

        # Set baseline file path within yolo_object_detection package directory
        package_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        self.baseline_file_path = os.path.join(package_directory, "baseline.json")

        # Ensure the directory exists
        try:
            os.makedirs(os.path.dirname(self.baseline_file_path), exist_ok=True)
        except PermissionError as e:
            self.get_logger().error(f"Permission denied while creating directory: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error while creating directory: {e}")

        # Load YOLOv8 model from ultralytics
        self.model = YOLO("yolov8n.pt")  # Using YOLOv8 Nano pre-trained model

        # Create SORT tracker object
        self.sort = Sort()  # Create SORT tracker instance

        # Subscribe to the robot"s Camera Feed
        self.image_subscription = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)

        # Load the baseline data (previous detections)
        self.baseline_objects = self.load_baseline()

        self.get_logger().info("Patrol Node Initialized")

    def load_baseline(self):
        """Load baseline data from the baseline.json."""
        if not os.path.exists(self.baseline_file_path):
            self.get_logger().warn("Baseline file not found. Creating a new baseline file.")
            return {"detections": []}

        try:
            with open(self.baseline_file_path, "r") as file:
                return json.load(file)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error decoding JSON from {self.baseline_file_path}: {e}")
            return {"detections": []}

    def image_callback(self, msg):
        """Receive camera image, perform object detection, and compare with baseline."""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Run YOLOv8 object detection
        results = self.model(self.latest_image)

        # Process detections and get bounding boxes
        detections = []
        for result in results[0].boxes.data.tolist():
            class_id = int(result[5])
            confidence = float(result[4])
            x1, y1, x2, y2 = map(int, result[:4])

            if confidence > 0.5:
                detections.append([x1, y1, x2, y2, confidence])

        # Convert detections list to a NumPy array before passing to SORT
        if detections:
            detections = np.array(detections)
            tracked_objects = self.sort.update(detections)

            # Process the tracked objects
            tracked_objects_list = []
            for track in tracked_objects:
                x1, y1, x2, y2 = map(int, track[:4])
                class_id = int(track[5]) if len(track) > 5 else -1  # Handle invalid class_id

                class_name = "Unknown"
                if class_id in results[0].names:
                    class_name = results[0].names[class_id]

                tracked_objects_list.append({
                    "bounding_box": [x1, y1, x2, y2],
                    "confidence": track[4],
                    "class": class_name
                })

            # Compare tracked objects with the baseline and log new objects only
            self.compare_with_baseline(tracked_objects_list)

            # Show the image with detections and tracked objects
            for obj in tracked_objects_list:
                x1, y1, x2, y2 = obj["bounding_box"]
                label = f"Conf: {obj['confidence']:.2f}, Class: {obj['class']}"
                cv2.rectangle(self.latest_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(self.latest_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Show the image in an OpenCV window
            cv2.imshow("Tracked Objects", self.latest_image)
            cv2.waitKey(1)
        else:
            self.get_logger().info("No objects detected in this frame.")

    def compare_with_baseline(self, tracked_objects):
        """Compare tracked objects with baseline data and log new objects."""
        new_objects = []

        # Compare tracked objects with baseline and log new objects
        for tracked in tracked_objects:
            found = False
            for baseline in self.baseline_objects["detections"]:
                if tracked["bounding_box"] == baseline["bounding_box"] and tracked["class"] == baseline["class"]:
                    found = True
                    break
            if not found:
                new_objects.append(tracked)

        for new_obj in new_objects:
            self.get_logger().info(f"New object detected: {new_obj['class']}")

        self.baseline_objects["detections"].extend(new_objects)
        self.save_baseline()

    def save_baseline(self):
        """Save the baseline data to baseline.json."""
        try:
            with open(self.baseline_file_path, "w") as file:
                json.dump(self.baseline_objects, file, indent=4)
            self.get_logger().info(f"Baseline data saved to {self.baseline_file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save baseline data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
