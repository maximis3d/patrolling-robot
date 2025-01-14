import os
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np  # Make sure to import NumPy
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

        # Use SORT to track the objects across frames
        tracked_objects = self.sort.update(detections)  # Get the list of tracked objects

        # Process the tracked objects
        tracked_objects_list = []
        for track in tracked_objects:
            object_id = int(track[4])  # The unique object ID from SORT
            x1, y1, x2, y2 = map(int, track[:4])  # Bounding box coordinates
            tracked_objects_list.append({
                "object_id": object_id,
                "bounding_box": [x1, y1, x2, y2],
                "confidence": track[4]
            })

        # Compare tracked objects with the baseline
        self.compare_with_baseline(tracked_objects_list)

        # Show the image with detections and tracked objects
        for obj in tracked_objects_list:
            x1, y1, x2, y2 = obj['bounding_box']
            label = f"ID: {obj['object_id']}, Conf: {obj['confidence']:.2f}"
            cv2.rectangle(self.latest_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(self.latest_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Show the image in an OpenCV window
        cv2.imshow("Tracked Objects", self.latest_image)
        cv2.waitKey(1)

    def compare_with_baseline(self, tracked_objects):
        """Compare tracked objects with baseline data and log new/missing objects."""
        new_objects = []
        missing_objects = []

        # Compare tracked objects with baseline
        for tracked in tracked_objects:
            found = False
            for baseline in self.baseline_objects["detections"]:
                if tracked['bounding_box'] == baseline['bounding_box']:
                    found = True
                    break
            if not found:
                new_objects.append(tracked)

        # Identify missing objects in the baseline
        for baseline in self.baseline_objects["detections"]:
            found = False
            for tracked in tracked_objects:
                if tracked['bounding_box'] == baseline['bounding_box']:
                    found = True
                    break
            if not found:
                missing_objects.append(baseline)

        # Log new or missing objects
        if new_objects:
            self.get_logger().info(f"New objects detected: {new_objects}")
            # Trigger alert for new object detection

        if missing_objects:
            self.get_logger().info(f"Missing objects: {missing_objects}")
            # Trigger alert for missing objects

def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
