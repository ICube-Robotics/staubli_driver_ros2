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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Launch arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_model",
            description="Model of the robot (e.g, 'tx2_60l').",
            choices=[
                "tx2_60l",
            ],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("staubli_robot_description"), "rviz", "view_robot.rviz"]
            ),
            description="RViz config file.",
        )
    )

    robot_model = LaunchConfiguration("robot_model")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    # Robot description

    description_file = PathJoinSubstitution(
        [FindPackageShare("staubli_robot_description"), "urdf", "staubli.urdf.xacro"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",
            "robot_model:=",
            robot_model,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    # Nodes

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(
        declared_arguments
        + [
            joint_state_publisher_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
