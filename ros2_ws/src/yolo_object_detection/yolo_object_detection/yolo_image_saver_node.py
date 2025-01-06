import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        # Create a dataset directory if it doesn't exist
        self.dataset_dir = "dataset/images"
        os.makedirs(self.dataset_dir, exist_ok=True)
        self.counter = 0
        self.get_logger().info("Image Saver Node Initialized.")

    def image_callback(self, msg):
        try:
            # Convert ROS2 image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Define the file name and save the image as .jpg
            file_path = os.path.join(self.dataset_dir, f"img_{self.counter:04d}.jpg")
            cv2.imwrite(file_path, cv_image)
            
            # Log the saved image and increment the counter
            self.get_logger().info(f"Saved Image: {file_path}")
            self.counter += 1

        except Exception as e:
            self.get_logger().error(f"Error saving image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
