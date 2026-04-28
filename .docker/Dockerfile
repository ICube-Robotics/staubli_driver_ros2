# syntax=docker/dockerfile:1

ARG ROS_DISTRO=jazzy

FROM osrf/ros:${ROS_DISTRO}-desktop-full

ARG ROS_DISTRO=jazzy

# Create workspace
WORKDIR /ws

# Copy source code
COPY . /ws/src/staubli_driver_ros2

# Install ROS2 dependencies
RUN apt-get update && apt-get upgrade -y \
    && rosdep install --from-paths src --ignore-src -r -y

# Build and install the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
        --install-base /opt/ros/staubli/ \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    rm -rf /ws && \
    rm -rf /opt/ros/staubli/staubli_robot_driver/include/ && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Source ROS and workspace setup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /opt/ros/staubli/setup.bash" >> /root/.bashrc

# Set entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/staubli/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
