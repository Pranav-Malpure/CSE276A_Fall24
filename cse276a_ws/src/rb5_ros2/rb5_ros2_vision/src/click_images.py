import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        # Create a subscriber to the 'camera/image' topic
        self.subscription = self.create_subscription(
            Image,
            'camera_0/image',
            self.listener_callback,
            10)
        self.save_directory = 'clicked_images'
        self.image_counter = 0
        os.makedirs(self.save_directory, exist_ok=True)
        # Initialize the CvBridge object
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Display the image using OpenCV
        file_path = os.path.join(self.save_directory, f'image_{self.image_counter:04d}.png')
        # Save the image to the specified directory
        if (self.image_counter <3):
            cv2.imwrite(file_path, frame)
            self.get_logger().info(f'Saved image {file_path}')
        # Increment the image counter
        self.image_counter += 1

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)
    # Create an instance of the ImageSubscriber class
    image_subscriber = ImageSubscriber()
    # Keep the node running to listen for incoming messages
    rclpy.spin(image_subscriber)
    # Cleanup on shutdown
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()