import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import json
import os

class RealObjectDetectionNode(Node):
    def __init__(self):
        super().__init__("real_object_detection_node")
        
        # ROS2 Publisher
        self.publisher = self.create_publisher(String, "/baseline_update", 10)

        # Load YOLO model and classes
        self.net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
        self.classes = self.load_classes("coco.names")
        self.layer_names = self.net.getUnconnectedOutLayersNames()
        
        # Initialize camera
        self.cap = cv2.VideoCapture(0)  # Open default camera
        self.timer = self.create_timer(0.1, self.detect_objects)  # Real-time detection (10 Hz)

        self.get_logger().info("Real Object Detection Node Initialized with YOLOv3")

    def load_classes(self, filename):
        """Load the COCO classes."""
        with open(filename, "r") as f:
            return [line.strip() for line in f.readlines()]

    def detect_objects(self):
        """Capture video feed and run object detection."""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame from camera.")
            return

        # Prepare the image for YOLO
        height, width, _ = frame.shape
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)

        # Perform forward pass and get detections
        outputs = self.net.forward(self.layer_names)
        boxes, confidences, class_ids = self.process_detections(outputs, width, height)

        # Create detection message for ROS2
        detection_data = self.create_detection_data(boxes, class_ids)

        # Publish detection data
        msg = String()
        msg.data = json.dumps(detection_data)
        self.publisher.publish(msg)

        # Display detections on the video feed
        self.display_detections(frame, boxes, class_ids)

    def process_detections(self, outputs, width, height):
        """Process YOLO detections."""
        boxes, confidences, class_ids = [], [], []

        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = int(scores.argmax())
                confidence = scores[class_id]

                if confidence > 0.5:  # Detection threshold
                    center_x, center_y, w, h = (
                        int(detection[0] * width),
                        int(detection[1] * height),
                        int(detection[2] * width),
                        int(detection[3] * height)
                    )
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, x + w, y + h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
        
        return boxes, confidences, class_ids

    def create_detection_data(self, boxes, class_ids):
        """Format detection data for the baseline manager."""
        detection_data = {}
        for i, box in enumerate(boxes):
            class_name = self.classes[class_ids[i]]
            if class_name not in detection_data:
                detection_data[class_name] = []
            detection_data[class_name].append(box)
        return detection_data

    def display_detections(self, frame, boxes, class_ids):
        """Show the detected objects with bounding boxes."""
        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = box
            label = self.classes[class_ids[i]]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("Real-Time Object Detection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RealObjectDetectionNode()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
