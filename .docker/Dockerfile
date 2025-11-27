# syntax=docker/dockerfile:1

FROM osrf/ros:jazzy-desktop-full

# Create workspace
WORKDIR /ws

# Copy source code
COPY . /ws/src/staubli_driver_ros2

# Install dependencies, build, and cleanup in single layer to reduce image size
RUN apt-get update && \
    touch /ws/src/staubli_driver_ros2/staubli_driver_ros2/COLCON_IGNORE && \
    rosdep install --from-paths src --ignore-src -r -y

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build \
        --install-base /opt/ros/icube/ \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --packages-select \
            staubli_bringup \
            staubli_moveit_config \
            staubli_robot_description \
            staubli_robot_driver && \
    rm -rf /ws && \
    rm -rf /opt/ros/icube/staubli_robot_driver/include/ && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Source ROS and workspace setup
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /opt/ros/icube/setup.bash" >> /root/.bashrc

# Set entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /opt/ros/icube/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
