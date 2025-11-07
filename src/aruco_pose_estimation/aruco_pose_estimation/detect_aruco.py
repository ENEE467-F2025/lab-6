#!/usr/bin/env python3

# Lab 6: Visual Robot Perception with ROS 2
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
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from aruco_markers_msgs.msg import Marker
import numpy as np

class ArucoCubeDetector(Node):
    def __init__(self):
        super().__init__('aruco_cube_detector')

        # Parameters
        self.declare_parameter("aruco_marker_id", value=80)
        self.marker_id = self.get_parameter("aruco_marker_id").get_parameter_value().integer_value
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/rgbd_camera/image', self.image_cb, 10)
        self.aruco_img_pub = self.create_publisher(Image, '/aruco_cube_image', 10)
        self.aruco_marker_pub = self.create_publisher(Marker, '/aruco_cube_marker', 10)

        # OpenCV ArUco detection setup
        # We must pick a dictionary that contains our cube's ID (80) 
        # unless the detection will fail
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        assert self.marker_id in self.aruco_dict.bytesList, "Marker ID must be in the dictionary"
        self.params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.params)

        # Timers
        self.detection_clk = self.create_timer(0.5, self.detection_clk_cb) 

        # Latest ArUco detection
        self.latest_aruco_det = None
    
    def detection_clk_cb(self):
        if self.latest_aruco_det is not None:
            self.get_logger().info("ArUco cube found. Canceling timer...")
            self.detection_clk.cancel()
        self.get_logger().warning("No objects with ArUco tags have been detected.")

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        corners, ids, _ = self.detector.detectMarkers(frame) # corners is a 1x4x2 array for each detected marker

        if ids is not None:
            det_aruco_ids = ids.flatten().tolist() # get ids of detections
            self.get_logger().info(f"Detected cube with ID: {det_aruco_ids[0]}") 

            # Annotate image to highlight detection
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Fill ArUco Marker to publish:
            # make the pose empty for now; we tackle this in a different unit!
            self.latest_aruco_det = Marker()
            self.latest_aruco_det.header.frame_id = 'camera_color_optical_frame'
            now = self.get_clock().now().to_msg()
            self.latest_aruco_det.header.stamp = now
            self.latest_aruco_det.id = det_aruco_ids[0] # to handle
                                                        # case where
                                                        # multiple faces are detected
            self.latest_aruco_det.pose = PoseStamped()
            box = corners[0].reshape(4, 2)  # remove extra dim to get actual ArUco box
            centroid = np.mean(box,axis=0) # center px is mean across rows

            self.latest_aruco_det.pixel_x = float(centroid[0] )# px coords must be float
            self.latest_aruco_det.pixel_y = float(centroid[1]) # for Marker message; 
                                                        # int o.w.
            # Annotate image
            cv2.circle(img=frame, 
                        center=(int(self.latest_aruco_det.pixel_x), int(self.latest_aruco_det.pixel_y)), 
                        color=(0, 0, 255),
                        radius=2, 
                        thickness=-1 
                    )
            # Publish annotated image
            out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.aruco_img_pub.publish(out_msg)

            # Publish ArUco marker
            self.aruco_marker_pub.publish(self.latest_aruco_det)

        cv2.imshow("Aruco Cube", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

def main():
    rclpy.init()
    node = ArucoCubeDetector()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()