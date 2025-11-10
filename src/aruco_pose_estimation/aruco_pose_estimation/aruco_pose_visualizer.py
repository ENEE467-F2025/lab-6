
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
from geometry_msgs.msg import PoseStamped, Pose, Point
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  
from aruco_interfaces.action import GetArucoPose
from rclpy.action import ActionClient
from std_msgs.msg import ColorRGBA

class ArucoPoseVisualizer(Node):
    def __init__(self):
        super().__init__('aruco_pose_visualizer')

        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for RViz marker
        self.marker_pub = self.create_publisher(Marker, 'aruco_marker_rviz', 10)

        # Action client to call the pose server
        self.client = ActionClient(self, GetArucoPose, 'get_aruco_pose')
        self.marker_id = 80  #
        self._goal_processed = False # goal state tracker

        # Wait a bit for server
        self.create_timer(1.0, self.send_goal)

    def send_goal(self):
        if self._goal_processed:
            return
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Pose server not available yet.')
            return

        goal_msg = GetArucoPose.Goal()
        goal_msg.marker_id = self.marker_id

        self.client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)
        self.get_logger().info(f'Sent goal for ArUco ID {self.marker_id}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected.')
            return
        self.get_logger().info('Goal accepted.')

        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        if self._goal_processed:
            return
        result = future.result().result
        if not result.aruco_pose:
            self.get_logger().warn('Marker not found.')
            return

        pose_camera: PoseStamped = result.aruco_pose  # PoseStamped in camera frame
        # Transform pose to base_link
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link', 
                pose_camera.header.frame_id,
                rclpy.time.Time()
            )
            pose_base = tf2_geometry_msgs.do_transform_pose(pose_camera.pose, trans)

            # Adjust z if it's negative, based on URDF setting for the table mesh
            if pose_base.position.z < 0:
                pose_base.position.z += 0.14
            self.get_logger().info(f'Transformed pose to base_link frame {pose_base.position.x, pose_base.position.y, pose_base.position.z}.')
        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}')
            return
        
        # publish cube and axes
        self.publish_marker(pose_base)
        self._goal_processed = True  # mark goal as handled

    def publish_marker(self, pose_base: Pose):
        # Publish RViz marker
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'aruco'
        marker.id = self.marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose_base # pose_base is already a Pose
        marker.pose.position.z = 0.001  # slight offset for table
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03  
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        self.marker_pub.publish(marker)

        # Axes marker (for orientation)
        axes = Marker()
        axes.header.frame_id = 'base_link'
        axes.header.stamp = self.get_clock().now().to_msg()
        axes.ns = "aruco_axes"
        axes.id = self.marker_id * 10 + 1
        axes.type = Marker.LINE_LIST
        axes.action = Marker.ADD
        axes.scale.x = 0.01  # line thickness

        # Define axis endpoints 
        axis_length = 0.1
        x_axis = Point()
        x_axis.x = axis_length
        x_axis.y = 0.0
        x_axis.z = 0.0

        y_axis = Point()
        y_axis.x = 0.0
        y_axis.y = axis_length
        y_axis.z = 0.0

        z_axis = Point()
        z_axis.x = 0.0
        z_axis.y = 0.0      
        z_axis.z = axis_length

        origin = Point()
        origin.x = 0.0
        origin.y = 0.0
        origin.z = 0.0

        # add line segments; (origin→X), (origin→Y), (origin→Z)
        axes.points = [origin, x_axis, origin, y_axis, origin, z_axis]

        # add corresponding colors
        red = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        green = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        blue = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        axes.colors = [red, red, green, green, blue, blue]

        # axes have the same pose as the cube
        axes.pose = pose_base
        axes.lifetime.sec = 0  # persistent

        self.marker_pub.publish(axes)
        self.get_logger().info(f'Published marker for ArUco ID {self.marker_id}')

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
