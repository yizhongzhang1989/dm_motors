cmake_minimum_required(VERSION 3.8)
project(dm_motor_bringup)

find_package(ament_cmake REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch test/test_launch.py TIMEOUT 30)
  ament_add_pytest_test(test_bringup test/test_bringup.py TIMEOUT 60)
endif()

ament_package()
