# VAL3 application for Staubli ROS2 server

> [!note] See `staubli_robot_driver` package documentation for more detailed informations.

## Pre-requisites

This application is designed for CS9 controller and VAL3 versions `8.x.x` or above.
It has been tested with a **TX2-60L** robot but should be compatible with most 6DoF Staubli robots.

## Installation

### Install required addons

You will need the following addons :
- motion
- velocity

Additionally, the addons `alter` and `advCtrlFunctions` are used, but they come shipped with the robot.

> [!tip] Download the addons from the MyStaubli website and copy the libraries (`<addon>.so`) to the `/usr/app` folder using FTP. You might have to create the folder first.

> [!warning] If the `alter` or `advCtrlFunctions` license are in demo mode, the code will stop working after 2 hours.

### Transfer app to robot controller

1) Connect the `J205` Ethernet port to the remote controller PC (ROS2-side)
2) Setup your IP address as `192.168.0.1` with mask `255.255.255.0`
3) Connect to FTP server at `192.168.0.254` (e.g., using filezilla app)
4) Copy the contents of `staubli_robot_driver/val3/userapp` to the `userapp` folder of the controller.
5) Replace / modify the controller network config. Either by transferring `staubli_robot_driver/val3/configs/sio.cfx` manually:

| Socket type | Socket name | Timeout | Fin de string  | port  | IP remote  |
|-------------|-------------|---------|----------------|-------|------------|
|     UDP     |   control   |    -1   |  10 (linux)    | 11000 | 172.31.0.2 |
|     UDP     | diagnostics |    -1   |  10 (linux)    | 11001 | 172.31.0.2 |

## Load and start application

1) Connect the `J204` Ethernet port to the remote controller PC (ROS2-side)
2) Setup your IP address as `172.30.0.2` with mask `255.255.255.0`
3) Power the robot ON
4) Load the application from the disk
5) Start the application on the pendant
6) Start the ROS2 driver

