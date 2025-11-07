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
from rcl_interfaces.msg._parameter_descriptor import ParameterDescriptor
from rclpy.action import ActionClient # this node isn't serving anything
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from control_msgs.action import FollowJointTrajectory # action interface from ros2_control
from aruco_interfaces.action import GetArucoPose # our custom action interface from before
import roboticstoolbox as rtb                    # the ``Toolbox''


class MoveToAruco(Node):
    def __init__(self):
        super().__init__("move_to_aruco")
        self.aruco_pose: PoseWithCovarianceStamped = None
        self.declare_parameter("marker_id", value=80)
        self.ctrl_client = ActionClient(
            self, # this client is associated with the node
            FollowJointTrajectory, # action type
            "/scaled_joint_trajectory_controller/follow_joint_trajectory" # action name
        )
        self.aruco_client = ActionClient(
            self,
            GetArucoPose,
            "get_aruco_pose"
        )

        # Create trajectory for the UR
        self.traj = JointTrajectory()
        self.traj.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, -1.57, 1.57, -1.57, 1.57, 0] # get aruco pose from the server
        point1.time_from_start.sec = 3

        self.traj.points.append(point1)

    def send_aruco_pose_goal(self, goal_msg):
        goal_msg = GetArucoPose.Goal()
        goal_msg.marker_id = 80
        return
    
    def send_ctrl_goal(self, goal_msg):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = self.traj

        # Wait for server
        self.get_logger().info("Waiting for ROS2 control action server...")
        self.ctrl_client.wait_for_server()

        # Send goal
        send_goal_future = self.ctrl_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()
        self.get_logger().info(f"Result: {result}")

def main(args=None):
    rclpy.init(args=args)
    move_to_aruco_node = MoveToAruco()
    rclpy.spin(move_to_aruco_node)
    move_to_aruco_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()