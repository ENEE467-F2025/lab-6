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
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from rclpy.action import ActionServer # our node will implement an action server
from aruco_markers_msgs.msg import Marker
from rclpy.action.server import ServerGoalHandle # helper class for accessing objects
                                                 # of server's goal handle instance
from geometry_msgs.msg import PoseStamped
from aruco_interfaces.action import GetArucoPose # import our custom action interface for use within the server

class EstimateArucoPose(Node):
    def __init__(self):
        super().__init__("estimate_aruco_pose")
        self.create_subscription(
            Image,
            "/rgbd_camera/depth",
            self.rgbd_depth_cb,
            10
        )
        self.create_subscription(
            Image,
            'aruco_cube_image', 
            self.aruco_image_cb,
            10
        )
        self.create_subscription(
            Marker,
            'aruco_cube_marker',
            self.aruco_marker_cb,
            10
        )
        self.aruco_pose_server = ActionServer(
            self,
            GetArucoPose,
            "get_aruco_pose",
            self.get_aruco_pose_cb
        )

        # initialize objects we need
        self.latest_rgbd_depth_image = None
        self.latest_aruco_marker = None
        self.latest_aruco_image = None

    def rgbd_depth_cb(self, depth_msg: Image):
        if not depth_msg:
            self.get_logger().error("No depth image received.")
            return
        self.latest_rgbd_depth_image = depth_msg

    def aruco_marker_cb(self, aruco_marker_msg: Marker):
        if not aruco_marker_msg:
            self.get_logger().error("No ArUco marker received.")
            return
        self.latest_aruco_marker = aruco_marker_msg

    def aruco_image_cb(self, aruco_image_msg: Image):
        if not aruco_image_msg:
            self.get_logger().error("No ArUco image received.")
            return
        self.latest_aruco_image = aruco_image_msg
        # Logic to process the ArUco marker detection
        pass

    def get_aruco_pose_cb(self, goal_handle: ServerGoalHandle):
        result = GetArucoPose.Result()
        feedback = GetArucoPose.Feedback()

        if not goal_handle.request.marker_id:
            self.get_logger().error("No marker ID received. Marker ID is required to estimate pose.")
            feedback.marker_found = False
            goal_handle.publish_feedback(feedback)
            goal_handle.abort()
            return result

        else:
            # Pose estimation logic
            feedback.marker_found = True
            feedback.pose = PoseStamped()  # Populate with actual pose data
            goal_handle.publish_feedback(feedback)
            goal_handle.succeed()
            return result

def main(args=None):
    rclpy.init(args=args)
    move_to_aruco_node = EstimateArucoPose()
    rclpy.spin(move_to_aruco_node)
    move_to_aruco_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()