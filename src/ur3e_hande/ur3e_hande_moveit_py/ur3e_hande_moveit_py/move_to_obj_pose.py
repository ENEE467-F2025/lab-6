#!/usr/bin/env python3
"""
ROS 2 node that requests a detected object's pose through the GetTargetObjPose
action and moves the UR3e arm to a standoff pose above it with MoveIt. Arm only;
the Hand-E gripper is not used in this part.

Usage:
    ros2 run ur3e_hande_moveit_py move_to_obj_pose

Author: Clinton Enwerem
Developed for the course ENEE467: Robotics Project Laboratory, University of Maryland, College Park, MD.
"""

import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from geometry_msgs.msg import Point
from ur3e_hande_planning_interfaces.action import GetTargetObjPose
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur as robot

STANDOFF_Z = 0.10  # meters above the detected object


class MoveToObjPose(Node):
    def __init__(self):
        super().__init__("move_to_obj_pose")

        self.declare_parameter("target_x", 0.28)
        self.declare_parameter("target_y", 0.50)
        self.declare_parameter("target_z", 0.05)
        self.declare_parameter("target_height", 0.12)
        self.declare_parameter("target_obj_bounds", [0.20, 0.33, 0.0, 0.0])

        self._cb_group = ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self._cb_group,
        )
        self.moveit2.max_velocity = 0.2
        self.moveit2.max_acceleration = 0.2

        self._action_client = ActionClient(
            self, GetTargetObjPose, "get_target_obj_pose", callback_group=self._cb_group
        )

        self._start_timer = self.create_timer(
            1.5, self._request_pose, callback_group=self._cb_group
        )

    def _request_pose(self):
        self._start_timer.cancel()

        # Add the mounting table to the planning scene before any planning. The stock
        # ur_moveit_config loads only the bare UR3e description, so the table from
        # ur3e_hande_description is absent on hardware; re-add it as a collision box
        # (dims/pose mirror table.urdf.xacro: 1.5x1.5x0.15 m at xyz 0 0.5 -0.095 in
        # base_link).
        self._add_support_surface()

        goal = GetTargetObjPose.Goal()
        goal.target_position = Point(
            x=self.get_parameter("target_x").value,
            y=self.get_parameter("target_y").value,
            z=self.get_parameter("target_z").value,
        )
        goal.target_height = float(self.get_parameter("target_height").value)
        goal.target_obj_bounds = [
            float(b) for b in self.get_parameter("target_obj_bounds").value
        ]

        self.get_logger().info("Requesting object pose...")
        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _add_support_surface(self):
        """Add the mounting table as a collision box and attach it to the base.

        Geometry/pose mirror ur3e_hande_description/urdf/table.urdf.xacro so the
        bare-hardware planning scene (stock ur_moveit_config) is aware of the bench.
        The box is attached to base_link with the base links as touch_links: the base
        is allowed to rest on the table (otherwise MoveIt reports the start state in
        collision and silently refuses to plan), while the forearm/wrist are still
        planned collision-free above it.
        """
        try:
            self.moveit2.add_collision_box(
                id="support_surface",
                size=(1.5, 1.5, 0.15),
                position=(0.0, 0.5, -0.095),
                quat_xyzw=(0.0, 0.0, 0.0, 1.0),
                frame_id=robot.base_link_name(),
            )
            time.sleep(0.5)  # let the planning scene register the world object first
            # Allow base contact only (touch_links); other links still avoid the table.
            self.moveit2.attach_collision_object(
                id="support_surface",
                link_name=robot.base_link_name(),
                touch_links=["base", "base_link", "base_link_inertia"],
            )
            time.sleep(0.3)
            self.get_logger().info("Added 'support_surface' (table); base contact allowed.")
        except Exception as e:
            self.get_logger().warn(f"Could not add support surface to planning scene: {e}")

    def _goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by the object pose server.")
            return
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result

        ####################################################################
        # TODO (Exercise 2b): Guard against a failed detection.
        # If result.target_obj_found is 0, log an error with
        # self.get_logger().error(...) and return without planning.
        ####################################################################

        target = result.target_obj_pose
        x = target.pose.position.x
        y = target.pose.position.y

        ####################################################################
        # TODO (Exercise 2a): Add a vertical standoff above the object so the
        # tool stops above it instead of colliding with it. Set z to the
        # detected z plus STANDOFF_Z.
        ####################################################################
        z = target.pose.position.z  # <--- MODIFY (add the standoff)

        quat = target.pose.orientation
        quat_xyzw = [quat.x, quat.y, quat.z, quat.w]

        self.get_logger().info(
            f"Moving arm to standoff above object at ({x:.3f}, {y:.3f}, {z:.3f})"
        )
        q = self.moveit2.compute_ik([x, y, z], quat_xyzw)
        if q is None:
            self.get_logger().error("IK failed for the standoff pose; aborting move.")
            return
        self.moveit2.move_to_configuration(list(q))
        self.moveit2.wait_until_executed()
        self.get_logger().info("Reached standoff pose.")


def main(args=None):
    rclpy.init(args=args)
    node = MoveToObjPose()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
