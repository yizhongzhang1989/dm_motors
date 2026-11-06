cmake_minimum_required(VERSION 3.8)
project(dm_motor_bringup)

find_package(ament_cmake REQUIRED)
install(CODE "file(REMOVE
  \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/launch/dual_motor_hardware.launch.py\"
  \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/launch/lead_follow_hardware.launch.py\")")
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch test/test_launch.py TIMEOUT 90)
  ament_add_pytest_test(test_bringup test/test_bringup.py TIMEOUT 90)
  ament_add_pytest_test(test_physical_simulation test/test_physical_simulation.py TIMEOUT 240)
  ament_add_pytest_test(test_motor_feedback_delivery test/test_motor_feedback_delivery.py TIMEOUT 120)
  ament_add_pytest_test(test_bilateral_simulation test/test_bilateral_simulation.py TIMEOUT 240)
  ament_add_pytest_test(test_bilateral_auto_test test/test_bilateral_auto_test.py TIMEOUT 240)
  ament_add_pytest_test(test_bilateral_angle_test test/test_bilateral_angle_test.py TIMEOUT 180)
  ament_add_pytest_test(test_bilateral_shared_reference test/test_bilateral_shared_reference.py TIMEOUT 100)
  ament_add_pytest_test(test_bilateral_bandwidth test/test_bilateral_bandwidth.py TIMEOUT 90)
  ament_add_pytest_test(test_bilateral_manual_test test/test_bilateral_manual_test.py TIMEOUT 120)
endif()

ament_package()
