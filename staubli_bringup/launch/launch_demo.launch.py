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

# Author: Thibault Poignonec <thibault.poignonec@gmail.com>

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch demo with robot control and MoveIt."""
    # Launch arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_model",
            description="Model of the robot (e.g, 'tx2_60l').",
            choices=[
                "tx2_60l",
            ],
            default_value="tx2_60l",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Use mock hardware for robot control.",
            choices=["true", "false"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_servo",
            default_value="false",
            description="Whether to launch the Moveit servo node",
        )
    )

    # Launch robot control
    launch_robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("staubli_bringup"), "launch", "launch_robot_control.launch.py"]
            )
        ),
        launch_arguments={
            "robot_model": LaunchConfiguration("robot_model"),
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
            "use_sim_time": "false",
            "start_controller": "joint_trajectory_controller",
            "gui": "false",  # Disable RViz in robot control
        }.items(),
    )

    # Launch MoveIt
    launch_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("staubli_bringup"), "launch", "launch_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "robot_model": LaunchConfiguration("robot_model"),
            "use_sim_time": "false",
            "launch_servo": LaunchConfiguration("launch_servo"),
            "gui": "true",
        }.items(),
    )

    return LaunchDescription(
        declared_arguments
        + [
            launch_robot_control,
            launch_moveit,
        ]
    )
