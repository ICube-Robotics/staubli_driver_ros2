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
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotEqualsSubstitution,
    PathJoinSubstitution,
)
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
            "use_mock_hardware",
            default_value="true",
            description="Use mock hardware for robot control.",
            choices=["true", "false"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true.",
            choices=["true", "false"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "start_controller",
            default_value="none",
            description="Controller to start at launch. None by default.",
            choices=[
                "none",
                "joint_trajectory_controller",
            ],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Launch Rviz if true.",
            choices=["true", "false"],
        )
    )

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    start_controller = LaunchConfiguration("start_controller")

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
            LaunchConfiguration("robot_model"),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    # Robot state publisher

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    # ros2_control node
    controllers = PathJoinSubstitution(
        [
            FindPackageShare("staubli_bringup"),
            "config",
            "controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers, {"use_sim_time": False}],
        output="both",
    )

    # Load controllers
    load_controllers = []
    load_controllers += [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        )
    ]
    # Add other controllers here, inactive at startup
    for controller in [
        "joint_trajectory_controller",
    ]:
        load_controllers += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    controller,
                    "--controller-manager",
                    "/controller_manager",
                    "--inactive",
                ],
            )
        ]

    # Start a controller if required
    start_controller_service = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/controller_manager/switch_controller",
            "controller_manager_msgs/srv/SwitchController",
            "{activate_controllers: [",
            start_controller,
            "]",
            ", deactivate_controllers: [], strictness: 1, activate_asap: true}",
        ],
        condition=IfCondition(NotEqualsSubstitution(start_controller, "none")),
        output="screen",
    )

    load_controllers.append(start_controller_service)

    # Rviz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("staubli_bringup"), "rviz", "default.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    # Launch description
    nodes = [
        robot_state_publisher_node,
        control_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes + load_controllers)
