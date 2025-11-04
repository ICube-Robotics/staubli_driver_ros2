Bringup the driver
===================

Setup the robot
****************

Follow steps in :ref:`installation_val3` to install the VAL3 application.

Start the VAL3 application on the robot
*****************************************

Load and run the application using the robot pendant:

1. Goto **VAL3 > Stockages > Disk**
2. Select ``ros2_server`` and load the application
3. Goto **VAL3 > Mémoire**
4. Select ``ros2_server`` and run

Launch the ROS2 driver
************************

Display robot in Rviz
~~~~~~~~~~~~~~~~~~~~~~

Connect to the robot using the driver and display its current pose in Rviz:

.. code-block:: bash

    source install/setup.bash

    ros2 launch staubli_bringup launch_robot_control.launch.py \
        robot_model:=tx2_60l \
        use_mock_hardware:=false \
        gui:=true

In case you have not set the IP address of your robot (J204) to ``192.168.0.254``, you can provide the actual value as a launch argument:

.. code-block:: bash

    source install/setup.bash

    ros2 launch staubli_bringup launch_robot_control.launch.py \
        robot_model:=tx2_60l \
        robot_ip:=<ip_of_your_robot> \
        use_mock_hardware:=false \
        gui:=true


Check available state / command interfaces
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can use the ros2_control `CLI interface <https://control.ros.org/master/doc/ros2_control/ros2controlcli/doc/userdoc.html>`__ to introspect the state and command interfaces provided by the driver:

.. code-block:: bash

    ros2 control list_hardware_interfaces

.. warning::
    The ``<joint_name>/velocity`` command interfaces are provided, but the internal implementation is not yet finalized.
    For now, the HW will reject the request for joint velocity control mode.

.. note::
    The ``<joint_name>/acceleration`` command interfaces are a by-product of providing joint limits to the ``Joint Limiter``.
    It is not an actual command mode and if you try to use it, the HW will reject the request.
    Hopefully, a more user-friendly way of setting the max acceleration parameter will be included in future releases of
    ros2_control, at which point this interface type will be removed.

What's next?
*************

See the **Demo Applications** section for some examples using the Staubli ROS2 driver.
