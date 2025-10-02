# staubli_robot_driver

ROS2 control interface for Staubli robots.

## Abstract Communication Architecture

The driver uses an abstracted communication architecture that allows for multiple transport protocols:

- **UDP** (Default implementation)
- **TCP/IP** (Placeholder for future implementation)
- **EtherCAT** (Placeholder for future implementation)

The communication abstraction allows for easily switching between protocols without changing the core robot interface code.

### Communication Components

1. **CommunicationInterface**: Abstract base class defining the interface for all communication protocols
2. **SocketFactory**: Factory class for creating communication interfaces of different types
3. **UDPCommunicator**: Implementation of the communication interface for UDP protocol
4. **TCPCommunicator**: (Future implementation) for TCP/IP protocol
5. **EtherCATCommunicator**: (Future implementation) for EtherCAT protocol

## Protocol Implementation

The driver uses two UDP socket connections to communicate with the robot controller:

### 1. Realtime UDP Socket (Bidirectional)
- **Direction**: Bidirectional (input/output)
- **Purpose**:
  - Receive RobotState data from the robot (in a detached thread)
  - Send RobotCommand messages to the robot
- **Default Port**: 8080

### 2. Diagnostic UDP Socket (Input only)
- **Direction**: Input only
- **Purpose**: Receive DiagnosticData from the robot (in a detached thread)
  - Robot version information
  - Error log with timestamps
  - Robot task statuses (motion control, IO control, etc.)
- **Default Port**: 8081

### Message Structure

All UDP messages follow this format:
```
+-----------------+----------------+
| UDPFrameHeader  |    Payload     |
| (14 bytes)      | (variable size)|
+-----------------+----------------+
```

#### UDP Frame Header (14 bytes)
- `protocol_version` (uint16_t): Protocol version (currently 1)
- `message_type` (uint16_t): Type of message (see MessageType enum)
- `sequence_number` (uint32_t): Sequence number for tracking messages
- `timestamp` (uint32_t): Timestamp (milliseconds since epoch)
- `payload_size` (uint16_t): Size of the payload in bytes

#### Message Types
- `ROBOT_STATE` (0x0001): Robot state data
- `ROBOT_COMMAND` (0x0002): Robot command data
- `DIAGNOSTIC_DATA` (0x0003): Diagnostic data

## Main Interface

The main interface is `StaubliRobotInterface` (defined in staubli_robot_interface.hpp), used to communicate with the robot:

### Constructor
```cpp
StaubliRobotInterface(
    const std::string& robot_ip,
    SocketFactory::ProtocolType protocol_type = SocketFactory::ProtocolType::UDP,
    uint16_t realtime_port = DEFAULT_REALTIME_PORT,
    uint16_t diagnostic_port = DEFAULT_DIAGNOSTIC_PORT);
```

### Connection Management
- `connect() / disconnect()`
  - Connect to robot using the specified protocol
  - Wait for first data to arrive
  - Check all is OK

### Operation Control
- `start() / stop()`
  - `start()`: Check robot can move
  - `stop()`: Send STOP signal to RobotCommand

### Communication Methods
- `send_robot_command(RobotCommand)`
  - Perform some basic checks
  - Encode RobotCommand msg and send using the selected protocol
- `wait_for_robot_state(double timeout)`
- `get_robot_state()`
- `get_diagnostic_data()`

## Usage Example

```cpp
// Create robot interface with UDP protocol (default)
staubli_robot_driver::StaubliRobotInterface robot(
    "192.168.1.10",
    staubli_robot_driver::SocketFactory::ProtocolType::UDP);

// Alternatively, use TCP protocol (when implemented)
// staubli_robot_driver::StaubliRobotInterface robot(
//    "192.168.1.10",
//    staubli_robot_driver::SocketFactory::ProtocolType::TCP);

// Connect to robot
if (!robot.connect()) {
    // Handle connection error
}

// Start robot control
if (!robot.start()) {
    // Handle start error
}

// Send a joint position command
staubli_robot_driver::RobotCommand command;
command.command_type = static_cast<uint8_t>(staubli_robot_driver::CommandType::POSITION);
command.joint_position_targets = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // 6-axis robot
robot.send_robot_command(command);

// Get robot state
auto state = robot.get_robot_state();
std::cout << "Joint positions: ";
for (const auto& pos : state.joint_positions) {
    std::cout << pos << " ";
}
std::cout << std::endl;

// Stop and disconnect
robot.stop();
robot.disconnect();
```
