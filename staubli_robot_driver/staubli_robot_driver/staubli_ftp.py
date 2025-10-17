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

"""Minimalist helper to transfer VAL3 scripts and config files to the robot."""

import ftplib
import os

import rclpy.logging
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class StaubliFTP(Node):
    """
    FTP client for Staubli robot.

    This class provides methods to connect, disconnect, and transfer files to the
    Staubli robot via FTP.

    :param Node: ROS2 Node
    :type Node: rclpy.node.Node
    """

    def __init__(self):
        """Initialize the StaubliFTP client from ROS2 parameters."""
        rclpy.init()
        super().__init__("staubli_ftp_client")
        self.declare_all_parameters()

        # Retrieve parameters
        self.robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value
        self.ftp_username = self.get_parameter("username").get_parameter_value().string_value
        self.ftp_password = self.get_parameter("password").get_parameter_value().string_value

        # Set up logger
        self.logger = rclpy.logging.get_logger("staubli_ftp_client")

        # Init FTP client
        self._ftp = None
        self.connect()

    def __del__(self):
        if self._ftp is not None:
            self.disconnect()
        rclpy.shutdown()

    def declare_all_parameters(self):
        self.declare_parameter(
            "robot_ip",
            "192.168.0.254",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="IP address of the Staubli robot controller (J205)",
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

    def connect(self):
        if self._ftp is not None:
            self.logger.warning("FTP error: cannot connect, FTP client already exists.")
            self.logger.info("Trying to disconnect FTP before reconnecting...")
            if not self.disconnect():
                return False
        # Establish FTP connection
        self.logger.info(
            f"Trying to connect to Staubli robot at {self.robot_ip} via FTP, please wait..."
        )
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

    # Remote directory operations

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

    # File upload operations

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

    # File download operations

    def download_file(self, remote_path: str, local_path: str) -> bool:
        if self._ftp is None:
            self.logger.error("FTP error: cannot download file, not connected to any FTP server.")
            return False
        try:
            with open(local_path, "wb") as file:
                self._ftp.retrbinary(f"RETR {remote_path}", file.write)
            self.logger.debug(f"Successfully downloaded {remote_path} to {local_path}.")
            return True
        except ftplib.all_errors as e:
            self.logger.error(f"FTP error while downloading file: {e}")
            return False

    def download_directory(self, remote_dir: str, local_dir: str) -> bool:
        if not os.path.exists(local_dir):
            os.makedirs(local_dir)
        try:
            items = self._ftp.nlst(remote_dir)
            for item in items:
                local_path = os.path.join(local_dir, os.path.basename(item))
                try:
                    self._ftp.cwd(item)
                    # If we can change directory, it's a folder
                    if not self.download_directory(item, local_path):
                        return False
                    self._ftp.cwd("..")
                except ftplib.error_perm:
                    # Not a directory, download the file
                    if not self.download_file(item, local_path):
                        return False
            return True
        except ftplib.all_errors as e:
            self.logger.error(f"FTP error while downloading directory: {e}")
            return False
