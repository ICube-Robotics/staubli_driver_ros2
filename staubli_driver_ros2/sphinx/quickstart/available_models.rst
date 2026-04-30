.. _supported_robots:

Available Robot Models
============================

The Staubli ROS2 Driver currently supports the following robot models "out-of-the-box":

.. list-table::
   :header-rows: 1
   :widths: 20 20 20 40

   * - Model
     - Robot type
     - Tag for launch
     - Notes
   * - TX2-60L
     - 6DoF arm
     - ``tx2_60l``
     - N/A
   * - TX2-60L MR (medical)
     - 6DoF arm
     - ``tx2_60l_med``
     - F/T sensor not yet supported

.. note::

  Depending on the specific robot, the robot controller sampling time may differ.
  For example, the TX2-60L has a sampling time of 4 ms (250 Hz), while the TX2-60L MR has a sampling time of 1 ms (1 kHz).
  The VAL3 application provided with the driver is designed to run at 250 Hz.
  However, if needed, it can be adapted to run faster by adjusting the control task period:

  1. Open the VAL3 application and navigate to the control task (``staubli_robot_driver/val3/userapp/ros2_server/start.pgx``).
  2. Find the line that sets the control period (around lines 35–36) and set it to the desired sampling time in second:

  .. code-block:: text

      // Setup control task
      l_nCtrlPeriod = <your sampling time in seconds>

  3. Redeploy the application to the robot
  4. Update ROS2 control parameters accordingly (i.e., set the control loop period to match the control task frequency).

  For reference, for the TX2-60L MR, the task frequency has been successfully tested at 500 Hz (2 ms control period).
