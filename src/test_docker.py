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
Test script to verify Docker container setup for Lab 6.

Author: Clinton Enwerem
Developed for the course ENEE467: Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
"""

import rclpy
import pathlib, os
from rclpy.node import Node
from ament_index_python import get_package_share_directory

# Lab 6 packages
lab6_packages = [
    "aruco_pose_estimation",
    "aruco_interfaces",
    "simple_image_annotator",
    "robotiq_hande_description",
    "ur3e_hande_description",
    "ur3e_hande_gz"
]
lab6_pkg_share_dirs = [get_package_share_directory(pkg) for pkg in lab6_packages]

class TestDockerNode(Node):
    def __init__(self):
        super().__init__('test_docker_node')
        self.docker_timer = self.create_timer(2.0, self.test_docker_cb)

    def test_docker_cb(self):

        # Check if lab 6 packages are accessible
        for pkg, share_dir in zip(lab6_packages, lab6_pkg_share_dirs):
            if not pathlib.Path(share_dir).exists():
                self.get_logger().error(f"Package '{pkg}' not found in lab 6 workspace at {share_dir}. Docker setup may be incorrect.")
                return

        self.get_logger().info("\x1b[92mAll packages for Lab 6. Docker setup is correct.\x1b[0m")
        self.docker_timer.cancel()

        # emulate Ctrl-C by shutting down rclpy, causing rclpy.spin() to exit
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TestDockerNode()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()