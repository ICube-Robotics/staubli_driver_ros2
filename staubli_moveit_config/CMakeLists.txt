cmake_minimum_required(VERSION 3.22)
project(staubli_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch srdf test
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  # Test SRDF files
  ament_add_pytest_test(test_default_srdf_xacro_parsing test/test_default_srdf.py)
endif()

ament_package()
