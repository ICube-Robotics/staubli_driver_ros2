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
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
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
            "robot_ip",
            default_value="172.31.0.1",
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

    # Start controller when control_node starts, with a delay for initialization
    start_controller_cmd = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "set_controller_state",
            LaunchConfiguration("start_controller"),
            "active",
        ],
        output="screen",
    )

    start_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=2.0,  # Wait a bit after controller manager starts
                    actions=[
                        LogInfo(
                            msg=[
                                "\033[32mStarting controller ",
                                LaunchConfiguration("start_controller"),
                                "\033[0m",
                            ]
                        ),
                        start_controller_cmd,
                    ],
                    condition=IfCondition(
                        NotEqualsSubstitution(LaunchConfiguration("start_controller"), "none")
                    ),
                )
            ],
        )
    )

    # Launch description
    nodes = [
        robot_state_publisher_node,
        control_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes + load_controllers + [start_controller])
