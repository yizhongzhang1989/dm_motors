cmake_minimum_required(VERSION 3.8)
project(dm_motor_dashboard)
find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/dashboard.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY web DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_test(test_integration
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_integration.py"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 60)
endif()
ament_package()
