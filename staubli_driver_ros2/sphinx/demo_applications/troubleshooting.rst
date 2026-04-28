.. _troubleshooting:

Troubleshooting
===============

This page covers some common problems encountered when installing and using the
Staubli ROS2 driver. If your issue is not listed here, check the robot logs first
(see :ref:`demo_get_logs`) — the VAL3 application logs most error conditions with
a description.

----

Connection issues
-----------------

**"Failed to connect to robot" at driver startup**

The driver prints this error and exits if it cannot receive a valid state message
from the robot within the connection timeout (5 seconds).

Check in order:

1. The VAL3 application is loaded and running on the pendant (the status LED next
   to ``ros2_server`` should be green, not flashing).
2. The robot IP is reachable from your PC:

   .. code-block:: bash

       ping 192.168.0.254

3. The PC Ethernet interface is configured with address ``192.168.0.1`` and mask
   ``255.255.255.0``. Any other address will be silently ignored by the robot.
4. The cable is connected to the **J204** port on the CS9 controller, not J205
   or any other port.
5. The sockets are configured on the pendant (**E/S > Socket**) with the correct
   remote IP matching your PC address.

----

**Robot connects but disconnects after ~2 hours**

The VAL3 addons ``alter`` and ``advCtrlFunctions`` are running in **demo mode**.
In demo mode, the addons stop functioning after 2 hours, which causes the control
task to fail.
To fix this, obtain a full license for both addons from Staubli and install it on
the controller.

----

Motion issues
--------------

**Driver connects but the robot does not move**

The hardware interface connects and publishes joint states, but sending a trajectory
has no effect. Check the following:

1. **Operation mode**: the robot must be in ``AUTO`` mode, or in ``MANUAL`` mode
   with motion explicitly enabled (press the *pause* button — the blue LED should
   be ON and steady, not flashing).
2. **E-stop**: verify the emergency stop button is released and the safety has been
   restarted if needed.
3. **Brakes**: the brakes must also be released to exit the STOP mode.
4. **Controller state**: confirm the trajectory controller is active:

   .. code-block:: bash

       ros2 control list_controllers

   The ``joint_trajectory_controller`` must be listed as ``active``.

----

**Robot stops unexpectedly mid-motion**

The VAL3 application transitions to ``STOP`` mode if it detects a problem.
Check the robot logs for the cause (see :ref:`demo_get_logs`). Common entries:

.. code-block:: text

    ERROR ros2_server::controlTask : Error, command sequence delay too high (5)

This means the robot did not receive a command from the ROS2 driver for 5 or more
consecutive control cycles (~20 ms at 250 Hz). Causes:

- The Ethernet link is shared with other traffic or goes through a switch with
  buffering. Use a **dedicated point-to-point Ethernet cable** between the PC and
  the J204 port.
- The ROS2 PC is overloaded and missing control loop deadlines. Check CPU usage;
  consider using a real-time kernel.
- The driver crashed. Check the driver terminal for errors.
  A typical error occurs when a controller requests a mode switch (e.g., position → velocity) without passing
  through ``STOP`` first. This is a driver-level constraint; see
  :ref:`control_modes` for the correct transition sequence.

----

**"NaN detected in command reference" in driver log**

The active controller sent a ``NaN`` position command. This typically means the
controller was activated before the robot state was valid, so its internal
reference was never initialized.

----

Visualization issues
---------------------

**Rviz opens but the robot model is not visible**

The URDF may not have loaded correctly, or the fixed frame is wrong.

- Check that the ``Fixed Frame`` in Rviz is set to ``world`` (or whatever frame is appropriate for your setup).
- In the launch terminal, look for URDF parse errors or mesh loading errors.

----

**Rviz shows the robot but joint positions do not update**

The ``joint_state_broadcaster`` is not active or has crashed.

.. code-block:: bash

    ros2 control list_controllers

If ``joint_state_broadcaster`` is active, check that a ``robot_state_publisher`` node is running and that the TF tree is being published:

.. code-block:: bash

   ros2 node list | grep robot_state_publisher

You can also verify that the ``/joint_states`` topic is being published:

.. code-block:: bash

    ros2 topic hz /joint_states
