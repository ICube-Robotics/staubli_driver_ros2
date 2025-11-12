#!/usr/bin/env python3
# Copyright 2025 ICube Laboratory
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# From ur_robot_driver/scripts/wait_for_robot_description.py
# Copyright (c) 2024 FZI Forschungszentrum Informatik

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import String

from rclpy.task import Future
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class WaitForRoboReady(Node):

    def __init__(self):
        super().__init__("wait_for_robot_ready")

        # Declare parameters
        self.declare_parameter("description_topic", "robot_description")
        self.declare_parameter("joint_states_topic", "joint_states")

        # Get parameter values
        self.description_topic = (
            self.get_parameter("description_topic").get_parameter_value().string_value
        )
        self.joint_states_topic = (
            self.get_parameter("joint_states_topic").get_parameter_value().string_value
        )

        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.robot_description_subscription = self.create_subscription(
            String, self.description_topic, self.description_callback, qos_profile=qos_profile
        )
        self.joint_states_subscription = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self.joint_states_callback,
            qos_profile=qos_profile,
        )
        self.robot_description_future = Future()
        self.joint_states_future = Future()

        self.get_logger().info(
            f"Waiting for topics '{self.description_topic}' and '{self.joint_states_topic}'..."
        )
        self.robot_is_ready_future = Future()

        # Throttled logging
        self.create_timer(5.0, self.log_waiting_status)

    def log_waiting_status(self):
        """Log waiting status every 5 seconds to avoid spam."""
        if not self.robot_is_ready_future.done():
            desc_status = "✓" if self.robot_description_future.done() else "✗"
            joint_status = "✓" if self.joint_states_future.done() else "✗"
            self.get_logger().info(
                f"""Still waiting... robot_description: {
                    desc_status}, joint_states: {joint_status}"""
            )

    def joint_states_callback(self, msg):
        if not self.joint_states_future.done():
            self.get_logger().info(
                f"""Received joint state message on {
                    self.resolve_topic_name(self.joint_states_topic)}."""
            )
            self.joint_states_future.set_result(True)

            if self.robot_description_future.done():
                self.emit_robot_ready()

    def description_callback(self, msg):
        if not self.robot_description_future.done():
            self.get_logger().info(
                f"""Received robot description on {
                    self.resolve_topic_name(self.description_topic)}."""
            )
            self.robot_description_future.set_result(True)

            if self.joint_states_future.done():
                self.emit_robot_ready()

    def emit_robot_ready(self):
        self.get_logger().info("Robot is ready.")
        self.robot_is_ready_future.set_result(True)


def main(args=None):
    rclpy.init(args=args)

    sub = WaitForRoboReady()
    rclpy.spin_until_future_complete(sub, sub.robot_is_ready_future)

    sub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
