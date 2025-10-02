# Copyright 2025 ICube Laboratory
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Author: Thibault Poignonec <thibault.poignonec@gmail.com>
# Adapted from https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/rolling/test/test_ur_urdf_xacro.py  # noqa: E501

import os
import shutil
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory

import pytest


@pytest.mark.parametrize("robot_model", ["tx2_60l"])
@pytest.mark.parametrize("prefix", ["", "staubli_"])
@pytest.mark.parametrize("use_mock_hardware", [True, False])
def test_urdf_xacro(robot_model, prefix, use_mock_hardware):
    print(f'Test XACRO description for model "{robot_model}".')
    print(f'  - prefix: "{prefix}"')
    print(f"  - use_mock_hardware: {use_mock_hardware}")

    description_file_path = os.path.join(
        get_package_share_directory("staubli_robot_description"), "urdf", "staubli.urdf.xacro"
    )

    (_, tmp_urdf_file) = tempfile.mkstemp(suffix=".urdf")

    # Compose `xacro` and `check_urdf` command
    xacro_command = (
        f'{shutil.which("xacro")} '
        f"{description_file_path} "
        f"name:={robot_model} "
        f"robot_model:={robot_model} "
        f"prefix:={prefix} "
        f"use_mock_hardware:={use_mock_hardware} "
        f"> {tmp_urdf_file}"
    )
    check_urdf_command = f'{shutil.which("check_urdf")} {tmp_urdf_file}'
    print("\nTest command: \ncheck_urdf < (" + xacro_command + ")\n")
    try:
        # Try to parse the XACRO file
        xacro_process = subprocess.run(
            xacro_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True
        )

        assert xacro_process.returncode == 0, " --- XACRO command failed ---"

        # Check the generated URDF file
        check_urdf_process = subprocess.run(
            check_urdf_command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            shell=True,
        )

        assert check_urdf_process.returncode == 0, "\n --- URDF check failed! --- "  # noqa: E501

    finally:
        os.remove(tmp_urdf_file)


if __name__ == "__main__":
    test_urdf_xacro()
