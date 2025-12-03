#!/usr/bin/env python3
# Copyright 2025 ICUBE Laboratory, University of Strasbourg
# License: Apache-2.0
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
#
# Author: Thibault Poignonec (thibault.poignonec@gmail.fr)

"""Minimalist helper script to transfer the VAL3 scripts to the robot."""

import os

from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from staubli_robot_driver.staubli_ftp import StaubliFTP
from staubli_robot_driver.staubli_ftp_utils import blue_text, upload_application, upload_sio_config


def main():  # noqa: D103
    ftp_client = StaubliFTP()
    if not ftp_client.is_connected:
        ftp_client.logger.error("Could not connect to the Staubli robot via FTP. Exiting...")
        return

    # Declare parameter to optionally upload SIO config files
    ftp_client.declare_parameter(
        "upload_sio_config",
        True,
        ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description="""Whether to upload the SIO configuration file(s) for
            the UDP/TCP sockets.
            WARNING: enabling this will overwrite any existing SIO
            configuration on the robot!
            """,
        ),
    )
    should_upload_sio_config = (
        ftp_client.get_parameter("upload_sio_config").get_parameter_value().bool_value
    )

    # Upload ros2_server VAL3 scripts
    package_share_directory = get_package_share_directory("staubli_robot_driver")
    app_name = "ros2_server"
    val3_scripts_path = os.path.join(package_share_directory, "val3", "userapp", app_name)

    if not upload_application(ftp_client, val3_scripts_path):
        ftp_client.logger.error("Failed to upload application. Exiting...")

    # Upload SIO config files
    if should_upload_sio_config:
        sio_file_path = os.path.join(package_share_directory, "val3", "configs", "sio.cfx")

        if not upload_sio_config(ftp_client, sio_file_path):
            ftp_client.logger.error("Failed to upload SIO configuration file. ")

    # Verbose success message
    ftp_client.logger.info(
        blue_text(
            f"TIP: don't forget to reload the application from the robot \
            pendant (Disk -> {app_name})."
        )
    )


if __name__ == "__main__":
    main()
