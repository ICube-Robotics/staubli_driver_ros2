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

"""
Minimalist helper script to display downloaded Staubli robot logs.

You can achieve the same result by running:

```bash
(
  echo -e "Date\tTimestamp\tType\tSeverity\tMessage";
  tail -n 200 .staubli_robot_logs/user.log | jq -r '
    .entry |
    if .lvl == "ERROR" then
      "\u001b[31m\\(.date)\t\\(.ts // "-")\t\\(.type)\t\\(.lvl)\t\\(.val // "-")\u001b[0m"
    elif .lvl == "WARN" then
      "\u001b[33m\\(.date)\t\\(.ts // "-")\t\\(.type)\t\\(.lvl)\t\\(.val // "-")\u001b[0m"
    else
      "\\(.date)\t\\(.ts // "-")\t\\(.type)\t\\(.lvl // "-")\t\\(.val // "-")"
    end'
) | column -t -s $'\t' | less -R
```

Note, this script requires 'jq' to be installed on your system.
"""

import os
import subprocess
import signal
import sys


def display_logs(log_path, num_lines=200):
    """Display the log entries with colored lines for ERROR and WARN."""
    command = f"""
    bash -c '
    (
      echo -e "Date\\t\\t\\tTimestamp\\t\\tType\\tSeverity\\tMessage";
      tail -n {num_lines} "{log_path}" | jq -r \\
        ".entry |
         if .lvl == \\"ERROR\\" then
           \\"\\\\u001b[31m\\(.date)\\t\\(.ts // \\"-\\")\\t\\(.type)\\t\\(.lvl)\\t\\(.val // \\"-\\")\\\\u001b[0m\\"
         elif .lvl == \\"WARN\\" then
           \\"\\\\u001b[33m\\(.date)\\t\\(.ts // \\"-\\")\\t\\(.type)\\t\\(.lvl)\\t\\(.val // \\"-\\")\\\\u001b[0m\\"
         else
           \\"\\(.date)\\t\\(.ts // \\"-\\")\\t\\(.type)\\t\\(.lvl // \\"-\\")\\t\\(.val // \\"-\\")\\"
         end"
    ) | less -R
    '
    """
    print(f"Displaying logs from {log_path}...")

    # Use Popen to allow for signal handling
    process = subprocess.Popen(
        command,
        shell=True,
        executable="/bin/bash",
        preexec_fn=os.setsid,  # Create a new process group
    )

    def handle_sigint(signum, frame):
        # Forward SIGINT to the child process group
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        sys.exit(0)

    # Set up signal handler for Ctrl+C
    signal.signal(signal.SIGINT, handle_sigint)

    # Wait for the process to complete
    process.wait()


def main():  # noqa: D103
    log_dir = ".staubli_robot_logs"
    user_log_path = f"{log_dir}/user.log"
    if not os.path.exists(user_log_path):
        print(f"User log file not found at {user_log_path}")
        return
    print("Displaying USER log (new entries first):\n\n")
    display_logs(user_log_path, num_lines=200)


if __name__ == "__main__":
    main()
