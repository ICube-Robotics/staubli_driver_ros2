Staubli ROS2 Driver Stack
=============================

.. image:: ./images/staubli_robot_header.jpg
   :align: center
   :width: 100%

This package provides a ROS2 driver for Staubli robots, enabling integration and control within ROS2-based applications.

Current developments are based on the **jazzy** ROS 2 distribution (Ubuntu 24.04 LTS).
The driver is designed for Staubli robot with CS9 controllers, older versions are not supported.
Refer to :ref:`supported_robots` for a list of supported robot models "out-of-the-box".

.. note::
  This driver is currently in development and some features will only be available in subsequent releases.

.. line-block::

   **Project GitHub repository:** https://github.com/ICube-Robotics/staubli_driver_ros2
   **Version:** |release|
   **Copyright:** © 2025 ICube Laboratory, University of Strasbourg, France.
   **License:** Apache License 2.0.
   **Author:** Thibault Poignonec <tpoignonec@unistra.fr> / <thibault.poignonec@gmail.com>

.. danger::
   🛑 **IMPORTANT - TEST SAFELY** 🛑

   In order to test the driver safely:

   * Always **test first in MANUAL mode**
   * Once you switch to **AUTO mode**, keep the **E-stop close by**
   * When testing the driver with MoveIt, start planning trajectories with a **(very) low velocity and acceleration scaling** (e.g., 10% max)
   * It is also recommended to **check the expected motion in RViz before executing it**

   ⚠️ **DISCLAIMER** ⚠️

   This software is provided "as is" **without any warranties**. The authors or distributors are **not liable for any damages**, including physical harm or financial loss, arising from the use or inability to use this software.

   **Users assume all responsibility and risk associated with its use.**

.. include:: frontmatter_pdf/toc.rst
