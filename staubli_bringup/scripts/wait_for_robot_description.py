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

from std_msgs.msg import String

from rclpy.task import Future
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class DescriptionSubscriber(Node):

    def __init__(self):
        super().__init__("wait_for_robot_description")
        self.topic = "robot_description"
        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.subscription = self.create_subscription(
            String, self.topic, self.desc_callback, qos_profile=qos_profile
        )
        self.future = Future()

        self.get_logger().info(f"Waiting for message on {self.resolve_topic_name(self.topic)}.")

    def desc_callback(self, msg):
        self.get_logger().info(
            f"Received message on {self.resolve_topic_name(self.topic)}. Shutting down."
        )
        self.future.set_result(True)


def main(args=None):
    rclpy.init(args=args)

    sub = DescriptionSubscriber()

    rclpy.spin_until_future_complete(sub, sub.future)

    sub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
