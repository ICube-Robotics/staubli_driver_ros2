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

from staubli_robot_driver.staubli_ftp import StaubliFTP
from staubli_robot_driver.staubli_ftp_utils import download_logs


def main():  # noqa: D103
    ftp_client = StaubliFTP()
    if not ftp_client.is_connected:
        ftp_client.logger.error("Could not connect to the Staubli robot via FTP. Exiting...")
        return

    if not download_logs(ftp_client, log_dir=".staubli_robot_logs"):
        ftp_client.logger.error("Failed to download logs. Exiting...")


if __name__ == "__main__":
    main()
