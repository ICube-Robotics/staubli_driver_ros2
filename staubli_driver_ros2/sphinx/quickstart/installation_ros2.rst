.. _installation_ros2:

Installation ROS2 package
===============================

Prerequisites
**************

Make sure you have **Ubuntu 24.04 LTS** installed on your machine.
Ideally, use a RT kernel for better performance.

Installation
*************

1. Install the ROS2 distribution (current dev. based on ``ros2 jazzy``).
See the `official documentation <https://docs.ros.org/en/jazzy/Installation.html>`_ for ROS2 installation steps.

2. Source the ROS2 environment:

.. code-block:: bash

    source /opt/ros/jazzy/setup.bash

3. Prepare the workspace and install dependencies

.. code-block:: bash

    sudo apt install python3-colcon-common-extensions

    mkdir ~/ros2_staubli_ws
    cd ~/ros2_staubli_ws
    git clone https://github.com/tpoignonec/staubli_driver_ros2.git src/staubli_driver_ros2

    rosdep install --ignore-src --from-paths . -y -r

4. Build stack

.. code-block:: bash

    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

5. Run automated tests (Optional)

.. code-block:: bash

    colcon test && colcon test-result

6. Source workspace

.. code-block:: bash

    source install/setup.bash
