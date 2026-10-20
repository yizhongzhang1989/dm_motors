cmake_minimum_required(VERSION 3.8)
project(dm_motor_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommand.msg" "msg/MotorState.msg" "msg/MotorTarget.msg"
  "msg/MotorSetpoint.msg" "msg/RegisterDescriptor.msg"
  "msg/MotorFeedback.msg"
  "msg/MotorRequest.msg" "msg/MotorReply.msg"
  "msg/MotorConfiguration.msg"
  "msg/LeadFollowState.msg"
  "srv/LeadFollowControl.srv"
  "msg/MotorTask.msg" "msg/MotorTaskFeedback.msg" "msg/MotorTaskResult.msg" "msg/MotorTaskEvent.msg"
  DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
