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

"""
Minimalist helper script to display downloaded Staubli robot logs.

Note, this script requires 'jq' to be installed on your system.
"""

import argparse
import os
import subprocess
import sys


def convert_log_to_csv(log_path, latest_run_only=False):
    """
    Convert log file to CSV format.

    Parameters
    ----------
    log_path : str
        Path to the log file
    latest_run_only : bool
        If True, only include logs since last reboot (last RUN entry)

    Returns
    -------
    str
        Path to the generated CSV file

    """
    # Create CSV file path
    csv_path = log_path.replace(".log", "_formatted.csv")

    # Generate CSV file (in reverse order - newest first)
    # Determine input command based on latest_run_only flag
    if latest_run_only:
        # Find the line number of the last RUN entry
        find_last_run = f"""
        grep -n '"type":"run"' "{log_path}" | tail -n 1 | cut -d: -f1
        """
        result = subprocess.run(
            find_last_run,
            shell=True,
            executable="/bin/bash",
            capture_output=True,
            text=True,
            check=False,
        )

        last_run_line = result.stdout.strip()
        if last_run_line:
            # Extract logs from the last RUN entry onwards
            input_cmd = f"tail -n +{last_run_line}"
        else:
            # No RUN entry found, use all logs
            input_cmd = "cat"
    else:
        input_cmd = "cat"

    csv_command = f"""
    {{
      echo "Date,Time,Timestamp,Type,Severity,Message";
      {input_cmd} "{log_path}" | jq -r '
        .entry |
        (.date | strptime("%Y-%m-%dT%H:%M:%S") | strftime("%d/%m/%Y")) as $date_fmt |
        (.date | strptime("%Y-%m-%dT%H:%M:%S") | strftime("%H:%M:%S")) as $time_fmt |
        ((.ts // "-") | if . == "-" then "-" else (tonumber | tostring + "000" | .[0:(.| index(".") + 4)]) end) as $ts_fmt |
        [$date_fmt, $time_fmt, $ts_fmt, .type, (.lvl // "-"), (.val // "-")] | @csv' | tac
    }} > "{csv_path}"
    """

    subprocess.run(csv_command, shell=True, executable="/bin/bash", check=True)
    return csv_path


def display_logs_in_terminal(csv_path, line_range=None):
    """
    Display the log entries with colored lines for ERROR and WARN.

    Parameters
    ----------
    csv_path : str
        Path to the CSV file
    line_range : tuple, optional
        Tuple of (start, end) line numbers, or (start, None) for start to end,
        or (None, end) for first to end lines. Default (None, 200) shows last 200 lines.

    """
    # Set default range if not specified
    if line_range is None:
        line_range = (None, 200)

    # Determine preprocessing command based on line_range
    start, end = line_range
    if start is not None and end is not None:
        # Show lines from start to end
        total = end - start + 1
        preprocess_command = f"cat {csv_path} | tail -n +{start + 1} | head -n {total}"
    elif start is not None and end is None:
        # Show from start to end of file
        preprocess_command = f"cat {csv_path} | tail -n +{start + 1}"
    elif start is None and end is not None:
        # Show first end lines
        preprocess_command = f"cat {csv_path} | head -n {end}"
    else:
        # Both None, show all
        preprocess_command = f"cat {csv_path}"

    display_command = f"""
    {preprocess_command} | while IFS=',' read -r date time ts type lvl msg; do
      # Remove quotes from CSV fields
      date=$(echo $date | tr -d '"')
      time=$(echo $time | tr -d '"')
      ts=$(echo $ts | tr -d '"')
      type=$(echo $type | tr -d '"')
      lvl=$(echo $lvl | tr -d '"')
      msg=$(echo $msg | tr -d '"')

      # Add blank lines before RUN type entries
      if [ "$type" = "run" ]; then
        echo ""
        echo "                Probable robot reboot detected (RUN entry)"
        echo ""
      fi

      if [ "$lvl" = "ERROR" ]; then
        echo -e "\\033[31m$date $time $ts $type $lvl $msg\\033[0m"
      elif [ "$lvl" = "WARN" ]; then
        echo -e "\\033[33m$date $time $ts $type $lvl $msg\\033[0m"
      else
        echo "$date $time $ts $type $lvl $msg"
      fi
    done | less -RMX --quit-if-one-screen
    """
    print("(Press 'q' to quit, or use arrow keys to navigate)\n")

    try:
        # Run the command directly without extra wrapping
        subprocess.run(display_command, shell=True, executable="/bin/bash", check=False)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        sys.exit(0)
    finally:
        # Reset terminal to ensure it works properly after less exits
        subprocess.run("stty sane", shell=True, check=False)


def main():  # noqa: D103
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description="Display Staubli robot logs with color coding and filtering."
    )
    parser.add_argument(
        "--file",
        "-f",
        type=str,
        default=None,
        help="Path to the log file (default: .staubli_robot_logs/user.log)",
    )
    parser.add_argument(
        "--range",
        type=str,
        default=None,
        help='Line range to display (e.g., "10:50", "100:", ":200"). Default: last 200 lines',
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Show all logs (default: only show logs since last reboot)",
    )

    args = parser.parse_args()

    # Determine log file path
    if args.file:
        user_log_path = args.file
    else:
        log_dir = ".staubli_robot_logs"
        user_log_path = f"{log_dir}/user.log"

    # Check if log file exists
    if not os.path.exists(user_log_path):
        print(f"User log file not found at {user_log_path}")
        return

    if args.all:
        print(f"Displaying log from '{user_log_path}' (new entries first):\n")
    else:
        print(f"Displaying log from '{user_log_path}' since last reboot (new entries first):\n")

    # Convert user log to CSV
    latest_run_only = not args.all
    user_log_csv_path = convert_log_to_csv(user_log_path, latest_run_only)

    # Parse range if provided
    line_range = None
    if args.range:
        try:
            parts = args.range.split(":")
            if len(parts) != 2:
                print(f"Invalid range format: {args.range}. Use 'start:end' format.")
                return
            start = int(parts[0]) if parts[0] else None
            end = int(parts[1]) if parts[1] else None
            line_range = (start, end)
        except ValueError:
            print(f"Invalid range values: {args.range}. Use integers for start and end.")
            return

    # Display logs
    display_logs_in_terminal(user_log_csv_path, line_range=line_range)


if __name__ == "__main__":
    main()
