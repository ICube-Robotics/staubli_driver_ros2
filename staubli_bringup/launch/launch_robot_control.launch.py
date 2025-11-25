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
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    EqualsSubstitution,
    FindExecutable,
    LaunchConfiguration,
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
            "robot_ip",
            default_value="192.168.0.254",
            description="IP address of the robot.",
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
            " ",
            "robot_ip:=",
            LaunchConfiguration("robot_ip"),
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

    # Always load and activate joint state broadcaster
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Load controllers
    load_controllers = [load_joint_state_broadcaster]

    # Add your controllers here
    controllers_to_load = [
        "joint_trajectory_controller",
    ]

    def load_controller_cmd(controller_name: str):
        # Spawn controller, inactive by default
        load_node = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                controller_name,
                "--controller-manager",
                "/controller_manager",
                "--inactive",
            ],
        )
        # Activate controller if specified at launch
        activation_condition = IfCondition(
            EqualsSubstitution(LaunchConfiguration("start_controller"), controller_name)
        )
        post_load_action = RegisterEventHandler(
            OnProcessExit(
                target_action=load_node,
                on_exit=[
                    LogInfo(
                        condition=activation_condition,
                        msg=[
                            "\033[32mStarting controller ",
                            controller_name,
                            "\033[0m",
                        ],
                    ),
                    ExecuteProcess(
                        condition=activation_condition,
                        cmd=[
                            "ros2",
                            "control",
                            "set_controller_state",
                            controller_name,
                            "active",
                        ],
                    ),
                ],
            )
        )
        return [load_node, post_load_action]

    for controller in controllers_to_load:
        load_controllers += load_controller_cmd(controller)

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
