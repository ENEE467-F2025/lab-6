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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from rclpy.action import ActionServer # our node will implement an action server
from aruco_markers_msgs.msg import Marker
from rclpy.action.server import ServerGoalHandle # helper class for accessing objects
                                                 # of server's goal handle instance
from geometry_msgs.msg import PoseStamped
from aruco_interfaces.action import GetArucoPose # import our custom action interface for use within the server

# vision tools
import cv2
import numpy as np
from cv_bridge import CvBridge

class EstimateArucoPose(Node):
    def __init__(self):
        super().__init__("estimate_aruco_pose")
        self._printed_marker_ids = set()  # track which markers have been printed
        self._printed_camera_info = False  # track if camera info has been printed

        # Subscribers
        self.create_subscription(Image, "/rgbd_camera/depth_image", self.rgbd_depth_cb, 10)
        self.create_subscription(Image, "aruco_cube_image", self.aruco_image_cb, 10)
        self.create_subscription(Marker, "aruco_cube_marker", self.aruco_marker_cb, 10)

        # #############################################################################
        # TODO Ex. 2a: Subscribe to camera info topic "/rgbd_camera/camera_info"
        # #############################################################################

        # #############################################################################


        # Action Server
       # #############################################################################
        # TODO Ex. 2c: Create the action server for GetArucoPose
        # set it to call self.get_aruco_pose_cb when a new goal is received
        # #############################################################################

        # #############################################################################

        # Latest data
        self.latest_rgbd_depth_image = None
        self.latest_aruco_marker = None
        self.latest_aruco_image = None
        self.K = None  # camera intrinsic matrix

        self.bridge = CvBridge()

        # #############################################################################
        # TODO Ex. 2b: Create the camera_info_cb callback to get K
        # #############################################################################


        # #############################################################################

    def rgbd_depth_cb(self, depth_msg: Image):
        self.latest_rgbd_depth_image = depth_msg

    def aruco_marker_cb(self, aruco_marker_msg: Marker):
        self.latest_aruco_marker = aruco_marker_msg

    def aruco_image_cb(self, aruco_image_msg: Image):
        self.latest_aruco_image = aruco_image_msg

    def get_aruco_pose_cb(self, goal_handle: ServerGoalHandle):
        result = GetArucoPose.Result()
        feedback = GetArucoPose.Feedback()

        marker_id = goal_handle.request.marker_id
        if not marker_id:
            self.get_logger().error("No marker ID received.")
            feedback.marker_found = False
            goal_handle.publish_feedback(feedback)
            goal_handle.abort()
            return result

        if self.latest_aruco_marker is None or self.latest_rgbd_depth_image is None or self.K is None:
            self.get_logger().warn("Missing data: waiting for depth image, ArUco marker, and camera info.")
            feedback.marker_found = False
            goal_handle.publish_feedback(feedback)
            goal_handle.abort()
            return result

        # Only compute if the detected marker matches requested ID
        if self.latest_aruco_marker.id != marker_id:
            feedback.marker_found = False
            goal_handle.publish_feedback(feedback)
            goal_handle.abort()
            return result

        # Convert depth image to numpy array
        depth_img = self.bridge.imgmsg_to_cv2(self.latest_rgbd_depth_image, desired_encoding="passthrough")

        # Compute centroid pixel of ArUco marker
        u = int(self.latest_aruco_marker.pixel_x)
        v = int(self.latest_aruco_marker.pixel_y)

        # Get intrinsics
        z = depth_img[v, u]  # depth in meters; row, column format
        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]

        # Back-project pixel to 3D camera coordinates
        #####################################
        # TODO: Ex 1d. Use Equation 2.
        # fx = f_x ...
        ####################################
        X = None
        Y = None
        Z = None
        ####################################

        # Only print once per marker ID
        if marker_id not in self._printed_marker_ids:
            self.get_logger().info(f"Estimated 3D position of marker ID {marker_id}: X={X}, Y={Y}, Z={Z}")
            self._printed_marker_ids.add(marker_id)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "camera_depth_frame"
        pose.pose.position.x = float(X)
        pose.pose.position.y = float(Y)
        pose.pose.position.z = float(Z)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0  

        feedback.marker_found = True
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        result.aruco_pose = pose
        return result

def main(args=None):
    rclpy.init(args=args)
    node = EstimateArucoPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()