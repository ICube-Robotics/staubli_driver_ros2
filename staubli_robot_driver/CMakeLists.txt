# Copyright 2025 ICUBE Laboratory, University of Strasbourg
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


cmake_minimum_required(VERSION 3.8)
project(staubli_robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)

set(DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  control_msgs
  realtime_tools
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)
foreach(DEPENDENCY ${DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Backward ROS for stack traces
find_package(backward_ros REQUIRED)

# ASIO for networking and async sockets
find_package(Boost REQUIRED COMPONENTS system)

# Library sources
set(LIBRARY_SOURCES
  # Communication protocol and sockets
  src/communication/protocol.cpp
  src/communication/messages.cpp
  src/communication/socket_factory.cpp
  src/communication/udp_socket.cpp
  # Socket pub / sub
  src/real_time_socket_subscriber.cpp
  src/real_time_socket_interface.cpp
  # Robot interface
  src/robot_driver.cpp
  # Mock robot server for testing
  src/mock_robot_server.cpp
)

# Main library
add_library(${PROJECT_NAME} SHARED ${LIBRARY_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${asio_SOURCE_DIR}/asio/include
)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

target_link_libraries(${PROJECT_NAME}
    Boost::system
)

# Hardware interface library
add_library(${PROJECT_NAME}_hardware_interface SHARED
  src/staubli_hardware_interface.cpp
)
target_include_directories(${PROJECT_NAME}_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_hardware_interface ${DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_hardware_interface
  ${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_hardware_interface
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


# Install Python scripts
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/upload_val3_server.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install other files (i.e., VAL3 scripts)
install(
  DIRECTORY val3
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  # Add unit tests
  find_package(ament_cmake_gtest REQUIRED)
  function(add_unit_test TEST_NAME TEST_SOURCE)
    ament_add_gtest(${TEST_NAME}
      ${TEST_SOURCE}
    )
    target_include_directories(${TEST_NAME} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(${TEST_NAME}
      ${PROJECT_NAME}
    )
  endfunction()

  add_unit_test(test_primitives_serialization
    test/unit_tests/communication/test_primitives_serialization.cpp)

  add_unit_test(test_message_serialization
    test/unit_tests/communication/test_message_serialization.cpp)

  add_unit_test(test_sockets
    test/unit_tests/communication/test_sockets.cpp)

  add_unit_test(test_rt_socket_interface
    test/unit_tests/communication/test_rt_socket_interface.cpp)

  add_unit_test(test_robot_driver test/unit_tests/test_robot_driver.cpp)

  add_unit_test(test_mock_robot_server test/unit_tests/test_mock_robot_server.cpp)

  # Add hardware interface test with additional dependencies
  ament_add_gtest(test_staubli_hardware_interface
    test/unit_tests/test_staubli_hardware_interface.cpp
  )
  target_include_directories(test_staubli_hardware_interface PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_staubli_hardware_interface
    ${PROJECT_NAME}
    staubli_robot_driver_hardware_interface
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  control_msgs
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

# Simple message receiver executable (for testing VAL3 communication)
add_executable(test_receive_robot_state
  src/test_receive_robot_state.cpp
)
target_include_directories(test_receive_robot_state PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_receive_robot_state
  ${PROJECT_NAME}
)

# Install the executable
install(TARGETS test_receive_robot_state
  DESTINATION lib/${PROJECT_NAME}
)

# Export plugin
pluginlib_export_plugin_description_file(hardware_interface staubli_hardware_interface.xml)

ament_package()
