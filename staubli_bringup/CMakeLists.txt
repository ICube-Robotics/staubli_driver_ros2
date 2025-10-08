cmake_minimum_required(VERSION 3.8)
project(staubli_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz scripts test
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/wait_for_robot_description.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
