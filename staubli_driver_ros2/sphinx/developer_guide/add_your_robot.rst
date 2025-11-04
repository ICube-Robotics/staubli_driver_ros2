.. _developer_adding_robot:

Add support for a new robot model
==================================

If your robot is not listed in :ref:`supported_robots`, you can add support for it by following these steps.

Prepare robot mesh files
-------------------------

1. Download the CAD model and technical specifications from the `MyStaubli`_ portal.

2. Export each robot link as a separate STL file.

3. Rename the STL files according to the following convention:

* Base link: ``base_link.stl``
* Subsequent links: ``link_<n>.stl``, where ``<n>`` is the link index starting from 1 (e.g., ``link_1.stl``, ``link_2.stl``, etc.).

4. (Opt., but **recommended**) Simplify the STL files to reduce their size using a CAD software (e.g., FreeCAD, Blender, etc.).

5. (Opt.) Return to your CAD software and export the robot meshes as COLLADA (.dae) files to improve visualization (colors, textures, etc.).


Gather the technical specifications of the robot
-------------------------------------------------

You will need the following information about the robot:

* Joint limits (position, velocity, acceleration).
* Link dimensions.
* Kinematic structure (Denavit-Hartenberg parameters or equivalent).
* (Opt.) Mass and inertia matrices of each link, not mandatory for the driver to work.

.. hint::
    Joint limit data can often be found in the robot's datasheet or technical manual available on the `MyStaubli`_ portal. Always **report joint limits strictly more conservative than the specifications** (e.g., if max acceleration is 1500, report 1000 or less). For the kinematic structure, Denavit-Hartenberg parameters are usually not readily provided, but you can derive them from the technical drawings in the datasheet.

.. note::
    Mass and inertia data might not be readily available; in that case, you can leave them as default values or contact Staubli if you need accurate data for simulation purposes (e.g., compute kinetic energy during motion).
    Mass and inertia data are not critical for the driver functionality

Update robot description files
---------------------------------

1. Fork the ``staubli_driver_ros2`` repository and create a new branch from ``main`` called ``feat/add_<robot_model>_description``.

2. Create a new folder in ``staubli_robot_driver/robots_description/meshes`` named ``<robot_model>`` (e.g., ``tx2_60l``).

3. Create two subfolders inside ``<robot_model>``: ``visual`` and ``collision``.

4. Copy the exported STL files to the ``collision`` folder.

5. If you exported COLLADA files, copy them to the ``visual`` folder. Else, just add an empty file named ``.gitkeep``.

6. Add a folder named ``<robot_model>`` in ``staubli_robot_driver/robots_description/config``.

7. Populate the folder by copying the contents of ``config/tx2_60l/`` as a starting point.

8. Update mesh file paths in ``config/<robot_model>/meshes.yaml`` to point to the new mesh files.

.. hint::
    If you don't have COLLADA files, you can use the collision meshes for the ``visual`` mesh:

    .. code-block:: yaml

        # File "config/<robot_model>/meshes.yaml"
        meshes:
            base_link:
                visual:
                package: staubli_robot_description
                path: meshes/tx2_60l/collision/base_link.stl
                collision:
                package: staubli_robot_description
                path: meshes/tx2_60l/collision/base_link.stl

9. Update joint limits in ``config/<robot_model>/joint_limits.yaml`` according to the robot specifications.

10. Update link dimensions in ``config/<robot_model>/kinematics.yaml`` according to the robot specifications.

.. hint::
    The parameters ``axis`` and ``rpy``, as well as zeroed fields in ``xyz`` should remain the same between robot models unless your robot has a different joint configuration. If it is not the case, please check the orientation of the joint axes in your CAD software, **it might indicate an inconsistency in the robot model**.

11. If you have mass and inertia data, update ``config/<robot_model>/dynamics.yaml``. Else, leave it as is for now or eyeball it. It is only used for simulation purposes when the robot is loaded in a physics engine (e.g., Gazebo).

12. Update the high-level parameters in ``config/<robot_model>/robot_model.yaml`` (e.g., number of joints, kinematic structure, etc.). For reference, here is a typical configuration:

.. code-block:: yaml

    # File "config/<robot_model>/robot_model.yaml"
    robot_model:
        # General robot capabilities
        has_joint_torque_sensor: false  # Whether the robot provides joint torque measurements
        has_ft_sensor: false  # Whether the robot has a force/torque sensor at the end-effector

        end_effector:
            # Config of the end-effector frame (tool0)
            parent_link: link_6  # Parent link of the end-effector (last robot link or F/T sensor if present)
            # Transformation from parent_link to end_effector
            xyz: [0.0, 0.0, 0.0]
            rpy: [0.0, 0.0, 0.0]

        io:
            num_digital_inputs: 16   # Number of digital inputs
            num_digital_outputs: 16  # Number of digital outputs
            num_analog_inputs: 2     # Number of analog inputs
            num_analog_outputs: 2    # Number of analog outputs

13. Register the new robot model in the tests and acceptable launch arguments:

* In ``staubli_robot_description/test/test_default_urdf.py``, add your robot model to the list of tested models:

.. code-block:: python

    ROBOT_MODELS = [
        "tx2_60l",
        "<robot_model>",  # Add your robot models here
    ]

* Add your robot model to the list of acceptable values for the ``robot_model`` launch argument in the files:

    * ``staubli_bringup/launch/launch_robot_control.launch.py``
    * ``staubli_bringup/launch/launch_moveit.launch.py``
    * ``staubli_robot_description/launch/display_robot.launch.py``

.. code-block:: python

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_model",
            description="Model of the robot (e.g, 'tx2_60l').",
            choices=[
                "tx2_60l",
                "<robot_model>",  # Add your robot models here
            ],
        )
    )

* Run tests on ``staubli_robot_description`` to ensure everything is working as expected (i.e., no error, no failure):

.. code-block:: bash

    colcon build --packages-select staubli_robot_description

    colcon test-result --delete
    # Type "Y" to delete previous test results

    colcon test --packages-select staubli_robot_description && colcon test-result

.. _mystaubli: https://www.staubli.com/global/en/robotics/services/MyStaubli-portal.html

14. Add documentation about the new robot model in :ref:`supported_robots`.

15. Test the new robot model thoroughly in simulation and on the real robot (if possible) to ensure all functionalities work as expected.

16. Create a pull request to merge your changes into the ``main`` branch. Et voila!

.. hint::
    If you need help or have questions about adding a new robot model, feel free to open an issue on the GitHub repository.
