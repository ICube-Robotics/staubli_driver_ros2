# staubli_driver_ros2

ROS2 stack to control Staubli robots using [ros2_control](https://github.com/ros-controls/ros2_control).

Current developments are based on the jazzy ROS 2 distribution (Ubuntu 24.04 LTS).
The driver is designed for Staubli robot with CS9 controllers, older versions are not supported.
To date, only position control is supported. More control modes (velocity and hand-guiding) as well as diagnostics support will be included in subsequent releases.

> [!NOTE]
> This driver is currently in development.
> However, due to interrupted funding on this project, active developments have been paused after the initial POC (november 2025). They should resume in Q3/Q4 of 2026.
> The driver is shared in its current state due to solicitations from the robotics community, but please keep in mind that it is more of an prototype than an industry-ready software.
>
> If your organisation relies on this software or would like to accelerate the implementation of specific features, project-specific funding or a research collaboration agreement can be arranged through the ICube Laboratory.
> To do so, please contact [Laurent Barbé](mailto:laurent.barbe@unistra.fr).

See full documentation at [https://icube-robotics.github.io/staubli_driver_ros2/](https://icube-robotics.github.io/staubli_driver_ros2/).

## ⚠️ Disclaimer and safety instructions

This software is provided "as is" without any warranties. The authors or distributors are not liable for any damages, including physical harm or financial loss, arising from the use or inability to use this software. **Users assume all responsibility and risk associated with its use.**

> [!IMPORTANT]
> In order to test the driver safely:
> - Always **test first in `MANUAL` mode**
> - Once you switch to `AUTO` mode, keep the E-stop close by...
> - When testing the driver with Moveit, start planning trajectories with a (very) low velocity and acceleration scaling (e.g., 10% max). It is also recommended to check the expected motion in Rviz before executing it.

## 🛠️ Installation

### Install the ROS2 driver

```bash
source /opt/ros/jazzy/setup.bash

mkdir ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ICube-Robotics/staubli_driver_ros2.git
cd ..

# Install ROS2 dependencies
rosdep install --ignore-src --from-paths . -y -r

# Build packages
colcon build
```

### Install the VAL3 driver on the robot

- Follow instructions in [staubli_robot_driver/val3/README.md](staubli_robot_driver/README.md).
- Transfer the VAL3 application:

```bash
source install/setup.bash

ros2 run staubli_robot_driver upload_val3_server.py
```

## 🚀 Quickstart

```bash
source install/setup.bash

ros2 launch staubli_bringup launch_robot_control.launch.py \
    robot_model:=tx2_60l \
    robot_ip:=<ip_of_your_robot> \
    use_mock_hardware:=false \
    gui:=true
```

## Contacts

<img src="staubli_driver_ros2/sphinx/images/logo-icube.png" alt="drawing" width="200"/>

[ICube Laboratory](https://icube.unistra.fr), [University of Strasbourg](https://www.unistra.fr/), France

__Thibault Poignonec:__ [tpoignonec@unistra.fr](mailto:tpoignonec@unistra.fr) / [thibault.poignonec@gmail.com](mailto:thibault.poignonec@gmail.com)
__Laurent Barbé:__ [laurent.barbe@unistra.fr](mailto:laurent.barbe@unistra.fr)
