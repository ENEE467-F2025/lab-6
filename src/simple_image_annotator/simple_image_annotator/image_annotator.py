#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class SimpleImageAnnotator(Node):
    def __init__(self):
        super().__init__('basic_image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/rgbd_camera/image',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

    def image_callback(self, msg: Image):
        # Print message header and key metadata
        self.get_logger().info(f"\n--- New Image Received ---")
        self.get_logger().info(f"Encoding: {msg.encoding}")
        self.get_logger().info(f"Height x Width: {msg.height} x {msg.width}")
        self.get_logger().info(f"Step (bytes per row): {msg.step}")
        self.get_logger().info(f"Data length: {len(msg.data)} bytes")

        # Compute and print number of channels from step
        bytes_per_pixel = msg.step // msg.width
        self.get_logger().info(f"Approx. {bytes_per_pixel} bytes/pixel")

        # Access the first pixel (raw bytes)
        first_pixel = msg.data[0 : bytes_per_pixel]
        self.get_logger().info(f"First pixel bytes: {list(first_pixel)}")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleImageAnnotator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()