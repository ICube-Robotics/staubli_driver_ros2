cmake_minimum_required(VERSION 3.8)
project(staubli_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes urdf r2c rviz test
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  # Test URDF files
  ament_add_pytest_test(test_default_urdf_xacro_parsing test/test_default_urdf.py)
endif()

# Ament package
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")
ament_package()
