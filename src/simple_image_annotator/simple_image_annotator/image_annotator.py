#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import sys

from simple_image_annotator.utils.cv2_utils import (
    sample_pixels,
    check_px_count,
    img_annotation_helper,
    crop_box,
    simple_annot,
)
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

class SimpleImageAnnotator(Node):
    def __init__(self):
        super().__init__('image_annotator')
        self.px_specified = any('px_count' in arg for arg in sys.argv)

        # class variables to store image metadata
        self.img_width: int = None
        self.img_height: int = None
        self.step: int = None
        self.bytes_per_pixel: int = None
        self.encoding: str = None
        self.img_data = None  # data buffer in the Image message

        # timer for metadata print method
        self.md_printer_timer = self.create_timer(0.5, self.print_image_metadata)

        # params
        self.declare_parameter("px_count", 10)
        self.declare_parameter("annot_depth", value=False)

        self.px_count = self.get_parameter("px_count").get_parameter_value().integer_value
        self.annot_depth = self.get_parameter("annot_depth").get_parameter_value().bool_value


        ##################################################
         # TODO (Section 2.3.1): Create bridge 
         # and initialize full_color_image
        ##################################################


        ##################################################

        
        ##################################################
        # TODO (Section 2.3.1): Initialize the class variables
        #  latest_color_box and color_box_center_px
        ##################################################
        # Initialize variables for image annotation


        ##################################################
        

        ##################################################
        # TODO (Section 2.3.1): Initialize roi
        ##################################################
        # Define ROI by bottom-left corner (u0, v0), width w,
        #  and height h


        ##################################################

        # color image subscriber
        self.create_subscription(
            Image,
            '/rgbd_camera/image',
            self.color_image_callback,
            10)
    
        ##################################################
        # TODO (Ex. 1 a): Initialize latest_depth_box
        # max_box_depth, center_box_depth, full_depth_image
        # set all to None
        ##################################################
        # Initialize variables for depth processing

        ##################################################
        
        ##################################################
        # TODO (Ex. 1 b): Create subscription to
        # '/rgbd_camera/depth_image'
        # use callback; self.depth_cb
        # QoS 10
        ##################################################


        ##################################################

    def color_image_callback(self, msg: Image):
        # update basic info once
        if self.img_data is None:
            self.img_height, self.img_width = msg.height, msg.width
            self.step = msg.step
            self.encoding = msg.encoding
            self.img_data = list(msg.data)
            self.bytes_per_pixel = self.step // self.img_width  # s in manual

        ##################################################
        # TODO (Section 2.3.1): try-except if block
        ##################################################


        ##################################################

    def print_image_metadata(self):

        # Print message header and key metadata
        if self.img_data is not None and self.px_specified:
            self.get_logger().info(f"\n--- New Image Received ---")
            self.get_logger().info(f"Encoding: {self.encoding}")
            self.get_logger().info(f"Height x Width: {self.img_height} x {self.img_width}")
            self.get_logger().info(f"Step (bytes per row): {self.step}")
            self.get_logger().info(f"Data length: {len(self.img_data)} bytes")
            self.get_logger().info(f"Bytes/pixel (s): {self.bytes_per_pixel}")

            
            # Access <px_count> pixels centered horizontally around the image midpoint (bottom row)
            mid_col = self.img_width // 2
            bottom_row = self.img_height - 1
            px_count = self.px_count
            check_px_count(px_count, self.img_width)
            half = px_count // 2 

            # even px_count; symmetric but no repeated center
            # odd px_count includes the center pixel
            start_col = mid_col - half
            end_col = mid_col + half + (px_count % 2)

            # clamp to bounds
            start_col = max(0, start_col)
            end_col = min(self.img_width, end_col)

            # collect pixel coordinates
            pixel_coords = [(bottom_row, n) for n in range(start_col, end_col)]

            sampled_pixels = sample_pixels(
                pixel_coords,
                step=self.step,
                bytes_per_pixel=self.bytes_per_pixel,
                img_data=self.img_data
            )

            # Print only pixel arrays with 20 elements at most
            if self.px_specified:
                if len(sampled_pixels) <= 20:
                    self.get_logger().info(
                        f"Sampled pixels: {[list(p) for p in sampled_pixels]}"
                        ) # casting to list expunges typecode 'B' from output
                else:
                    self.get_logger().info(
                        f"There are too many sampled pixels; skipping printing..."
                        )
                self.get_logger().info(f"Number of sampled pixels: {len(sampled_pixels)}")
                self.md_printer_timer.cancel() # cancel this timer so it fires only once
                # no exit to allow OpenCV window to stay open

        ##################################################
        # NOTE (Ex. 1 c): Complete callback
        ##################################################
    def depth_cb(self, msg: Image):
        """ Function to process incoming depth images
        """
        # Convert to OpenCV depth image 
        depth_image = self.bridge.imgmsg_to_cv2(
                        msg,
                        desired_encoding='passthrough')
        self.full_depth_image = depth_image

        # If depth image is ready, crop 
        if self.full_depth_image is not None:
            self.latest_depth_box = crop_box(self.full_depth_image,
                                             self.roi)

            # Calculate max depth ignoring zeros 
            valid_depths = self.latest_depth_box[self.latest_depth_box > 0] 

            if valid_depths.size > 0:
                ##################################################
                # TODO (Ex. 1 c): Complete callback
                ##################################################
                self.max_box_depth = None # <-- MODIFY; use np.max()
                if self.color_box_center_px is not None:
                    cx, cy = self.color_box_center_px
                else:
                    cx, cy = np.mean(self.roi, axis=0).astype(int)
                ##################################################
                # NOTE: center_box_depth is the depth value at
                # the center pixel;
                # slice self.full_depth_image at center pixel 
                # coordinates to get it
                ##################################################

                self.center_box_depth = None  # <-- MODIFY; see NOTE
                ##################################################

# class end

#
def main(args=None):
    node = None
    try:
        rclpy.init(args=args)
        node = SimpleImageAnnotator()
        rclpy.spin(node)
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"Exception: {e}")
        else:
            print(f"Exception before node creation: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()