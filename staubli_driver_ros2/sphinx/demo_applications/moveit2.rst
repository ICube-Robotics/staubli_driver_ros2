.. _demo_moveit2:

Plan a motion with Moveit2
============================

Before you start
~~~~~~~~~~~~~~~~~

Make sure the robot is ready to accept commands before starting any controller:

* AUTO mode (or MANUAL, but with the **motion enabled**)
* E-Stop not pressed and safety restarted if needed
* Brakes open
* Motion enabled (press "pause" button, the blue LED should be ON and not flashing)

Bringup robot control and MoveIt2 move group
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In a first terminal, launch the Staubli robot control node (i.e., ``ros2_control``):

.. code-block:: bash

    source install/setup.bash

    ros2 launch staubli_bringup launch_robot_control.launch.py \
        robot_model:=tx2_60l \
        use_mock_hardware:=false \
        start_controller:=joint_trajectory_controller \
        gui:=false

This will start the Staubli hardware interface in ``ros2_control``, along with two controllers:
* `Joint State Broadcaster <https://control.ros.org/master/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__ to publish the joint state of the robot;
* `Joint Trajectory Controller <https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html>`__ to execute joint trajectory commands.

In a second terminal, launch MoveIt2:

.. code-block:: bash

    source install/setup.bash

    ros2 launch staubli_bringup launch_moveit.launch.py robot_model:=tx2_60l

You should be able to plan and execute a trajectory as follows:

1. Drag the handle attached to the end effector in Rviz to set the desired robot pose.
2. Press **Plan** in the MoveIt widget on the left.
3. If successful, press **Execute** to start robot motion.


.. warning::

    When testing the driver with Moveit, start planning trajectories with a (very) low velocity and acceleration scaling (e.g., 10% max).
    It is also recommended to check the expected motion in Rviz before executing it.
