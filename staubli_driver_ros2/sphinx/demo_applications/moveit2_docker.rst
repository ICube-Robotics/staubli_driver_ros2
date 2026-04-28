.. _demo_docker:

Idem, but using a pre-built Docker image
=========================================

The driver is also available as a pre-built Docker image, which can be used to test the driver without going through the build process.
The image is available on GitHub Container Registry `here <https://github.com/ICube-Robotics/staubli_driver_ros2/pkgs/container/staubli_driver_ros2>`_.

Pre-requisites
---------------

First, make sure you have Docker installed on your system:

.. code-block:: bash

    docker --version

Then, install `rocker <https://github.com/osrf/rocker>`_, a tool to run ROS applications in Docker containers with support for hardware access and GUI forwarding:

.. code-block:: bash

    sudo apt install python3-rocker

Retrieve and test the Docker image
-----------------------------------

Pull the Docker image (pick a version) from the registry:

.. code-block:: bash

    export IMG_TAG=<TAG_NAME>  # e.g., 0.1.0
    docker pull ghcr.io/icube-robotics/staubli_driver_ros2:${IMG_TAG}

Run the demo with a mock robot (no connection to a physical robot) to make sure everything is working:

.. code-block:: bash

    export IMG_TAG=<TAG_NAME>  # e.g., 0.1.0
    rocker --net=host --devices /dev/dri --x11 \
        ghcr.io/icube-robotics/staubli_driver_ros2:${IMG_TAG} \
        ros2 launch staubli_bringup launch_demo.launch.py \
            robot_model:=tx2_60l \
            use_mock_hardware:=true

.. note::

    The `rocker` command below uses the following options:

    - ``--net=host``: Uses host networking to allow communication with the robot without additional configuration
    - ``--x11``: Enables X11 forwarding for GUI applications
    - ``--devices /dev/dri``: Mounts Direct Rendering Infrastructure for Intel hardware-accelerated graphics

Connect to the physical robot
------------------------------

Setup as usual (VAL3 application running on the robot, E-stop released, etc.) and run the demo with the real robot:

.. code-block:: bash

    export IMG_TAG=<TAG_NAME>  # e.g., 0.1.0
    rocker --net=host --devices /dev/dri --x11 \
        ghcr.io/icube-robotics/staubli_driver_ros2:${IMG_TAG} \
        ros2 launch staubli_bringup launch_demo.launch.py  \
            use_mock_hardware:=false \
            robot_model:=<ROBOT_MODEL> \
            robot_ip:=<ROBOT_IP_ADDRESS>
