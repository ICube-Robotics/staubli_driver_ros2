# staubli_driver_ros2
ROS2 stack to control Staubli robots

## Installation

```bash
source /opt/ros/jazzy/setup.bash

mkdir ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/tpoignonec/staubli_driver_ros2.git
# git checkout jazzy

# Build ros2 packages
cd ..
colcon build
source install/setup.bash
```

Then follow instruction at [staubli_robot_driver/val3/README.md](staubli_robot_driver/README.md).
