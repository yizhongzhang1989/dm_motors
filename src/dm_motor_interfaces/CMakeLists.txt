cmake_minimum_required(VERSION 3.8)
project(dm_motor_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommand.msg" "msg/MotorState.msg" "msg/MotorTarget.msg"
  "msg/MotorSetpoint.msg" "msg/RegisterDescriptor.msg"
  "srv/ControlLease.srv" "srv/MotorOperation.srv" "srv/RegisterAccess.srv"
  "srv/GetOperation.srv" "srv/GetCapabilities.srv" "action/RunTask.action"
  DEPENDENCIES std_msgs action_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
