# yolo_object_detection_node.py (Updated for RViz2 Image Display)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YOLOObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_object_detection_node')
        
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt') 
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.report_publisher = self.create_publisher(String, '/object_detection_report', 10)
        self.image_publisher = self.create_publisher(Image, '/yolo_image_with_boxes', 10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            results = self.model(cv_image)
            detections = results[0].boxes.data.cpu().numpy()  

            
            for detection in detections:
                x1, y1, x2, y2, conf, cls = detection
                class_name = self.model.names[int(cls)]
                confidence = conf

                # Draw bounding box and label on the image
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(cv_image, f"{class_name} {confidence:.2f}", (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Publish detection info
                report_msg = String()
                report_msg.data = f"Detected: {class_name} with confidence {confidence:.2f}"
                self.report_publisher.publish(report_msg)
                self.get_logger().info(report_msg.data)

            image_with_boxes = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_publisher.publish(image_with_boxes)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = YOLOObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
