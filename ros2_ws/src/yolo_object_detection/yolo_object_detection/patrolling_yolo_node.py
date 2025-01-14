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
        super().__init__('patrol_node')

        # Initialize paths, load YOLO model, and set up SORT for object tracking
        self.bridge = CvBridge()
        self.baseline_file_path = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'baseline.json')
        os.makedirs(os.path.dirname(self.baseline_file_path), exist_ok=True)

        # Load YOLOv8 model from ultralytics
        self.model = YOLO('yolov8n.pt')  # Using YOLOv8 Nano pre-trained model

        # Create SORT tracker object
        self.sort = Sort()  # Create SORT tracker instance

        # Subscribe to the robot's Camera Feed
        self.image_subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Load the baseline data (previous detections)
        self.baseline_objects = self.load_baseline()

        self.get_logger().info("Patrol Node Initialized")

    def load_baseline(self):
        """Load baseline data from the baseline.json."""
        if not os.path.exists(self.baseline_file_path):
            self.get_logger().warn("Baseline file not found. Please ensure baseline data is created first.")
            return {"detections": []}
        
        with open(self.baseline_file_path, 'r') as file:
            return json.load(file)

    def image_callback(self, msg):
        """Receive camera image, perform object detection, and compare with baseline."""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Run YOLOv8 object detection
        results = self.model(self.latest_image)

        # Process detections and get bounding boxes
        detections = []
        for result in results[0].boxes.data.tolist():  # Each result corresponds to a detection box
            class_id = int(result[5])  # class ID (for the detected object)
            confidence = float(result[4])  # detection confidence
            x1, y1, x2, y2 = map(int, result[:4])  # bounding box coordinates

            if confidence > 0.5:  # Only consider high-confidence detections
                detections.append([x1, y1, x2, y2, confidence])

        # Convert detections list to a NumPy array before passing to SORT
        if detections:
            detections = np.array(detections)  # Convert the list to a NumPy array
            tracked_objects = self.sort.update(detections)  # Get the list of tracked objects

            # Process the tracked objects
            tracked_objects_list = []
            for track in tracked_objects:
                x1, y1, x2, y2 = map(int, track[:4])  # Bounding box coordinates
                class_id = int(track[5])  # Get class ID from tracker

                # Check if the class ID is valid and exists in the names dictionary
                class_name = "Unknown"  # Default value in case class ID is invalid
                if class_id in results[0].names:
                    class_name = results[0].names[class_id]  # Get the class name from YOLO

                tracked_objects_list.append({
                    "bounding_box": [x1, y1, x2, y2],
                    "confidence": track[4],
                    "class": class_name  # Store the class name
                })

            # Compare tracked objects with the baseline and log new objects only
            self.compare_with_baseline(tracked_objects_list)

            # Show the image with detections and tracked objects
            for obj in tracked_objects_list:
                x1, y1, x2, y2 = obj['bounding_box']
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
                # Compare based on bounding box and class name
                if tracked['bounding_box'] == baseline['bounding_box'] and tracked['class'] == baseline['class']:
                    found = True
                    break
            if not found:
                new_objects.append(tracked)

        # Log new objects (class name only)
        for new_obj in new_objects:
            self.get_logger().info(f"New object detected: {new_obj['class']}")

        # Optionally: You could add actions when new objects are detected
        # After logging, add these new objects to the baseline (if needed)
        self.baseline_objects["detections"].extend(new_objects)
        self.save_baseline()

    def save_baseline(self):
        """Save the baseline data to baseline.json."""
        with open(self.baseline_file_path, 'w') as file:
            json.dump(self.baseline_objects, file, indent=4)
        self.get_logger().info(f"Baseline data saved to {self.baseline_file_path}")

def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
