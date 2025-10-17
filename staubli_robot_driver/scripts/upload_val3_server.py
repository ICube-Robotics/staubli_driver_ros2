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

"""Minimalist helper script to transfer the VAL3 scripts to the robot."""

import ftplib
import os

from ament_index_python.packages import get_package_share_directory
import rclpy.logging
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class StaubliFTP(Node):
    def __init__(self):
        super().__init__("staubli_ftp_client")
        self.declare_all_parameters()

        # Retrieve parameters
        self.robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value
        self.ftp_username = self.get_parameter("username").get_parameter_value().string_value
        self.ftp_password = self.get_parameter("password").get_parameter_value().string_value
        self.upload_sio_config = (
            self.get_parameter("upload_sio_config").get_parameter_value().bool_value
        )

        # Set up logger
        self.logger = rclpy.logging.get_logger("staubli_ftp_client")

        # Init FTP client
        self._ftp = None
        self.connect()

    def __del__(self):
        if self._ftp is not None:
            self.disconnect()

    def declare_all_parameters(self):
        self.declare_parameter(
            "robot_ip",
            "192.168.0.254",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="IP address of the Staubli robot controller (ethernet interface J205)",
            ),
        )

        self.declare_parameter(
            "username",
            "default",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="FTP username for the Staubli robot controller",
            ),
        )

        self.declare_parameter(
            "password",
            "default",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="FTP password for the Staubli robot controller",
            ),
        )

        self.declare_parameter(
            "upload_sio_config",
            True,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="""Whether to upload the SIO configuration file(s) for the UDP/TCP sockets
                WARNING: enabling this will overwrite any existing SIO configuration on the robot!
                """,
            ),
        )

    def connect(self):
        if self._ftp is not None:
            self.logger.warning("FTP error: cannot connect, FTP client already exists.")
            self.logger.info("Trying to disconnect FTP before reconnecting...")
            if not self.disconnect():
                return False
        # Establish FTP connection
        self.logger.info(
            f"Trying to connect to Staubli robot at {self.robot_ip} via FTP, please wait...")
        try:
            self._ftp = ftplib.FTP(self.robot_ip)
            self._ftp.login(user=self.ftp_username, passwd=self.ftp_password)
            self.logger.info(f"Connected to Staubli robot at {self.robot_ip} via FTP.")
            return True
        except ftplib.all_errors as e:
            self._ftp = None
            self.logger.error(f"FTP error: {e}")
            return False

    def disconnect(self):
        if self._ftp is not None:
            self._ftp.quit()
            self._ftp = None

    @property
    def is_connected(self):
        return self._ftp is not None

    def upload_file(self, local_path: str, remote_path: str) -> bool:
        if self._ftp is None:
            self.logger.error("FTP error: cannot upload file, not connected to any FTP server.")
            return False
        try:
            with open(local_path, "rb") as file:
                self._ftp.storbinary(f"STOR {remote_path}", file)
            self.logger.debug(f"Successfully uploaded {local_path} to {remote_path}.")
            return True
        except ftplib.all_errors as e:
            self.logger.error(f"FTP error while uploading file: {e}")
            return False

    def remote_directory_exists(self, remote_dir: str) -> bool:
        if self._ftp is None:
            self.logger.error(
                "FTP error: cannot check directory, not connected to any FTP server."
            )
            return False
        current_dir = self._ftp.pwd()
        try:
            self._ftp.cwd(remote_dir)
            self._ftp.cwd(current_dir)
            return True
        except ftplib.error_perm:
            return False
        except ftplib.all_errors as e:
            self.logger.error(f"FTP error while checking directory: {e}")
            return False

    def create_remote_directory(self, remote_dir: str) -> bool:
        if self._ftp is None:
            self.logger.error(
                "FTP error: cannot create directory, not connected to any FTP server."
            )
            return False
        try:
            self._ftp.mkd(remote_dir)
            self.logger.debug(f"Successfully created remote directory {remote_dir}.")
            return True
        except ftplib.error_perm:
            self.logger.warning(f"Remote directory {remote_dir} probably already exists.")
            return True
        except ftplib.all_errors as e:
            self.logger.error(f"FTP error while creating remote directory: {e}")
            return False

    def delete_remote_directory(self, remote_dir: str) -> bool:
        if self._ftp is None:
            self.logger.error(
                "FTP error: cannot delete directory, not connected to any FTP server."
            )
            return False
        # Check if directory exists
        if self.remote_directory_exists(remote_dir):
            self.logger.debug(f"Deleting remote directory {remote_dir}...")
        else:
            self.logger.warning(
                f"Remote directory {remote_dir} does not exist, nothing to delete."
            )
            return True
        # Recursively delete directory contents
        try:
            items = self._ftp.nlst(remote_dir)
            for item in items:
                try:
                    self._ftp.delete(item)
                    self.logger.debug(f"Deleted remote file {item}.")
                except ftplib.error_perm:
                    self.delete_remote_directory(item)
            self._ftp.rmd(remote_dir)
            self.logger.debug(f"Successfully deleted remote directory {remote_dir}.")
            return True
        except ftplib.all_errors as e:
            self.logger.error(f"FTP error while deleting remote directory: {e}")
            return False

    def upload_directory(self, local_dir: str, remote_dir: str) -> bool:
        if not self.create_remote_directory(remote_dir):
            self.logger.warning(f"Could not create remote directory {remote_dir}")
        # Recursively upload directory contents
        for item in os.listdir(local_dir):
            local_path = os.path.join(local_dir, item)
            remote_path = f"{remote_dir}/{item}"
            if os.path.isdir(local_path):
                if not self.upload_directory(local_path, remote_path):
                    return False
            else:
                if not self.upload_file(local_path, remote_path):
                    return False
        return True


def main():
    rclpy.init()
    ftp_client = StaubliFTP()
    if not ftp_client.is_connected:
        ftp_client.logger.error("Could not connect to the Staubli robot via FTP. Exiting...")
        return

    # Define helper functions
    def abort():
        ftp_client.logger.error("Aborting script due to previous errors.")
        ftp_client.disconnect()
        rclpy.shutdown()
        exit(1)

    def green_text(message: str) -> str:
        return f"\033[32m{message}\033[0m"

    def blue_text(message: str) -> str:
        return f"\033[34m{message}\033[0m"

    # Get VAL3 scripts path
    package_share_directory = get_package_share_directory("staubli_robot_driver")

    # Create remote application directory if it does not exist
    remote_usrapp_dir = "/usr/usrapp"
    if not ftp_client.remote_directory_exists(remote_usrapp_dir):
        ftp_client.logger.info(
            f"Remote USRAPP directory '{remote_usrapp_dir}' does not exist. Creating it..."
        )
        if not ftp_client.create_remote_directory(remote_usrapp_dir):
            abort()

    # Upload ros2_server VAL3 scripts
    appname = "ros2_server"
    val3_scripts_path = os.path.join(package_share_directory, "val3", "userapp", "ros2_server")

    remote_app_path = f"{remote_usrapp_dir}/{appname}"
    if ftp_client.remote_directory_exists(remote_app_path):
        ftp_client.logger.info(
            f"Remote application directory '{remote_app_path}' already exists. Deleting it first..."
        )
        ftp_client.delete_remote_directory(remote_app_path)
    ftp_client.logger.info(
        f"Uploading VAL3 scripts for application '{appname}' to '{remote_app_path}'..."
    )
    if not ftp_client.upload_directory(val3_scripts_path, remote_app_path):
        abort()
    else:
        ftp_client.logger.info(
            green_text(f"Successfully uploaded VAL3 scripts for application '{appname}'.")
        )

    # Upload SIO config files
    if ftp_client.upload_sio_config:
        sio_file_path = os.path.join(package_share_directory, "val3", "configs", "sio.cfx")
        remote_sio_file_path = "/usr/configs/sio.cfx"
        ftp_client.logger.info(
            f"Uploading SIO configuration file 'sio.cfx' to '{remote_sio_file_path}'..."
        )
        if not ftp_client.upload_file(sio_file_path, remote_sio_file_path):
            abort()
        else:
            ftp_client.logger.info(green_text("Successfully uploaded SIO configuration file."))
    # Verbose success message
    ftp_client.logger.info(
        blue_text(
            f"TIP: don't forget to reload the application from the robot pendant (Disk -> {appname})."
        )
    )

    # Clean up
    ftp_client.disconnect()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
