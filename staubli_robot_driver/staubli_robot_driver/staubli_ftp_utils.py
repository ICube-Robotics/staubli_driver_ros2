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

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from staubli_robot_driver.staubli_ftp import StaubliFTP


def green_text(message: str) -> str:  # noqa: D103
    return f"\033[32m{message}\033[0m"


def blue_text(message: str) -> str:  # noqa: D103
    return f"\033[34m{message}\033[0m"


def download_logs(ftp_client: StaubliFTP, log_dir: str) -> bool:
    """
    Download robot logs to local directory.

    :param ftp_client: Connected StaubliFTP client
    :type ftp_client: StaubliFTP

    :param log_dir: Local directory to download logs to
    :type log_dir: str

    :return: True if logs were successfully downloaded, False otherwise
    :rtype: bool
    """
    if not ftp_client.is_connected:
        ftp_client.logger.error("Could not connect to the Staubli robot via FTP. Exiting...")
        return False

    # Get VAL3 scripts path
    ftp_client.declare_parameter(
        "log_path",
        ".staubli_robot_logs",
        ParameterDescriptor(
            description="Path to the directory where robot logs will be downloaded.",
            type=ParameterType.PARAMETER_STRING,
        ),
    )
    ftp_client.logger.info(f"Downloading robot logs to local directory '{log_dir}'...")

    # Create local log directory if it does not exist
    os.makedirs(log_dir, exist_ok=True)

    # Download logs
    remote_log_dir = "/log"
    if not ftp_client.remote_directory_exists(remote_log_dir):
        ftp_client.logger.error(
            f"Remote log directory '{remote_log_dir}' does not exist. Aborting..."
        )
        False

    # Download logs
    log_files = [
        "arm.json",
        "staubli-update.log",
        "system.log",
        "thread_usage.json",
        "user.log",
        "user.old",
    ]

    all_ok = True
    for log_file in log_files:
        remote_log_file_path = f"{remote_log_dir}/{log_file}"
        local_log_file_path = os.path.join(log_dir, log_file)
        ftp_client.logger.info(
            f"Downloading log file '{remote_log_file_path}' to '{local_log_file_path}'..."
        )
        if not ftp_client.download_file(remote_log_file_path, local_log_file_path):
            all_ok = False
            ftp_client.logger.warning(f"Could not download log file '{remote_log_file_path}'!")

    ftp_client.logger.info(green_text("Successfully downloaded robot logs."))

    # Clean up
    return all_ok


def upload_application(ftp_client: StaubliFTP, application_path: str) -> bool:
    """
    Upload VAL3 application scripts to the robot.

    :param ftp_client: Connected StaubliFTP client
    :type ftp_client: StaubliFTP
    :param app_dir: Valid path to the directory containing VAL3 application scripts
    :type app_dir: str
    :return: True if application was successfully uploaded, False otherwise
    :rtype: bool
    """
    if not ftp_client.is_connected:
        ftp_client.logger.error("Could not connect to the Staubli robot via FTP. Exiting...")
        return False

    # Create remote application directory if it does not exist
    remote_usrapp_dir = "/usr/usrapp"
    if not ftp_client.remote_directory_exists(remote_usrapp_dir):
        ftp_client.logger.info(
            f"Remote USRAPP directory '{remote_usrapp_dir}' does not exist. Creating it..."
        )
        if not ftp_client.create_remote_directory(remote_usrapp_dir):
            return False

    # Upload ros2_server VAL3 scripts
    app_name = os.path.basename(application_path)
    ftp_client.logger.info(
        f"Preparing to upload application '{app_name}' from '{application_path}'..."
    )
    remote_app_path = f"{remote_usrapp_dir}/{app_name}"
    if ftp_client.remote_directory_exists(remote_app_path):
        ftp_client.logger.info(
            f"Remote application directory '{remote_app_path}' already exists. Deleting it first..."
        )
        ftp_client.delete_remote_directory(remote_app_path)
    ftp_client.logger.info(
        f"Uploading VAL3 scripts for application '{app_name}' to '{remote_app_path}'..."
    )
    if not ftp_client.upload_directory(application_path, remote_app_path):
        ftp_client.logger.error(f"Failed to upload VAL3 scripts for application '{app_name}'.")
        return False
    ftp_client.logger.info(green_text(f"Successfully uploaded application '{app_name}'."))
    return True


def upload_sio_config(ftp_client: StaubliFTP, sio_config_path: str) -> bool:
    """
    Upload SIO configuration file to the robot.

    :param ftp_client: Connected StaubliFTP client
    :type ftp_client: StaubliFTP
    :param sio_config_path: Valid path to the SIO configuration file
    :type sio_config_path: str
    :return: True if SIO configuration file was successfully uploaded,
             False otherwise
    :rtype: bool
    """
    if not ftp_client.is_connected:
        ftp_client.logger.error("Could not connect to the Staubli robot via FTP. Exiting...")
        return False

    # Check if is a valid file
    if not os.path.isfile(sio_config_path):
        ftp_client.logger.error(f"SIO config file '{sio_config_path}' does not exist!")
        return False

    if os.path.basename(sio_config_path) != "sio.cfx":
        ftp_client.logger.error("SIO config file should be named 'sio.cfx'!")
        return False

    # Upload SIO config file
    remote_sio_file_path = "/usr/configs/sio.cfx"
    ftp_client.logger.info(
        f"Uploading SIO configuration file from '{sio_config_path}' "
        f"to '{remote_sio_file_path}'..."
    )
    if not ftp_client.upload_file(sio_config_path, remote_sio_file_path):
        ftp_client.logger.error("Failed to upload SIO configuration file.")
        return False
    ftp_client.logger.info(green_text("Successfully uploaded SIO configuration file."))
    return True
