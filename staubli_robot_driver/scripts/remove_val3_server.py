#!/usr/bin/env python3
# Copyright 2025 ICUBE Laboratory, University of Strasbourg
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Thibault Poignonec (thibault.poignonec@gmail.fr)

"""Minimalist helper script to remove the VAL3 scripts from the robot."""

from staubli_robot_driver.staubli_ftp import StaubliFTP


def main():  # noqa: D103
    ftp_client = StaubliFTP()
    if not ftp_client.is_connected:
        ftp_client.logger.error("Could not connect to the Staubli robot via FTP. Exiting...")
        return

    # Remove ros2_server VAL3 scripts
    remote_usrapp_dir = "/usr/usrapp"
    appname = "ros2_server"
    remote_app_path = f"{remote_usrapp_dir}/{appname}"
    if ftp_client.remote_directory_exists(remote_app_path):
        ftp_client.logger.info(
            f"Removing application directory '{remote_app_path}' from the robot..."
        )
        if not ftp_client.delete_remote_directory(remote_app_path):
            ftp_client.logger.error(
                f"Could not remove contents of application directory '{remote_app_path}'."
            )
        else:
            ftp_client.logger.info(
                f"Successfully removed contents of application directory '{remote_app_path}'."
            )


if __name__ == "__main__":
    main()
