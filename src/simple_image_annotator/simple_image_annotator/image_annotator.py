#!/usr/bin/env python3

# Lab 6: Visual Robot Perception in ROS 2
# Copyright (C) 2025 Clinton Enwerem

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


"""
ROS2 node for demonstrating basic interoperability with the OpenCV
library via CvBridge.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge
from utils.cv2_utils import img_annotation_helper, crop_box

class SimpleImageAnnotator(Node):
    def __init__(self):
        super().__init__("simple_image_subscriber_node")
        self.bridge = CvBridge() # object for OpenCV-ROS2 translation

        # Initialize variables for box images
        self.latest_color_box = None
        self.latest_depth_box = None

        # Initialize objects for lab
        self.max_box_depth = None
        self.center_box_pixel = None
        self.center_box_depth = None

        # Also store full images for cropping
        self.full_color_image = None
        self.full_depth_image = None

        # Corresponds to one face of the aruco cube
        self.box_coords = np.array([[536., 314.],
                                    [545., 283.],
                                    [639., 283.],
                                    [636., 315.]])
        
        # Subscribe to color and depth image streams
        self.create_subscription(
            Image, 
            '/rgbd_camera/image', 
            self.color_cb, 
            10) # color
        self.create_subscription(
            Image, 
            '/rgbd_camera/depth_image', 
            self.depth_cb, 
            10) # depth
        
    def color_cb(self, msg: Image):
        """
        msg.encoding is rgb8, which represents 
        an unsigned 8-bit integer.
        
        Image message data is in raw bytes! Must convert via CvBridge 
        to use OpenCV.
        """
        
        # Cast ROS2 Image to OpenCV BGR
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.full_color_image = cv_image

        # If color image is ready, crop box
        if self.full_color_image is not None:

            self.latest_color_box = crop_box(
                                    self.full_color_image,
                                    self.box_coords
                                    )
            ######## your code here ############### 
            self.center_box_pixel = np.mean(self.box_coords, axis=0).astype(int)
            ######## your code here ###############

            # Display cropped color box
            if (
                self.max_box_depth is not None
                and self.center_box_pixel is not None
                and self.center_box_depth is not None
                and self.box_coords is not None
            ):
                img_annotation_helper(
                max_box_depth=self.max_box_depth, 
                center_box_depth=self.center_box_depth,
                box_coords=self.box_coords
                )
        cv2.imshow("Cropped Color Box", self.full_color_image)
        cv2.waitKey(1) # wait for user to destroy process to close window

    def depth_cb(self, msg: Image):
        """ Message fields are the same as in color_cb. Only encoding 
        is different.
        
        msg.encoding is 16UC1, which means that the image is single-channel, 
        & each pixel is represented by 16-bit unsigned values on [0, (2^16)-1)

        "passthrough" will keep the input encoding.
        """
        # Convert to OpenCV depth image 
        depth_image = self.bridge.imgmsg_to_cv2(
                        msg,
                        desired_encoding='passthrough')
        self.full_depth_image = depth_image

        # If depth image is ready, crop 
        if self.full_depth_image is not None:
            self.latest_depth_box = crop_box(self.full_depth_image,
                                             self.box_coords)

            # Calculate max depth ignoring zeros 
            valid_depths = self.latest_depth_box[self.latest_depth_box > 0] 
            if valid_depths.size > 0:

            ######## your code here ###############
                self.max_box_depth = np.max(valid_depths)
                cx, cy = np.mean(self.box_coords, axis=0).astype(int)
                self.center_box_depth = self.full_depth_image[int(cy), int(cx)]
            ######## your code here ###############

def main(args = None):
    rclpy.init(args=args)
    image_subscriber_node = SimpleImageAnnotator()
    rclpy.spin(image_subscriber_node)
    image_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
