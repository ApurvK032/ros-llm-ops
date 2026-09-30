cmake_minimum_required(VERSION 3.16)
project(warehouse_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Parcel.msg"
  "msg/Stop.msg"
  "msg/Hold.msg"
  "msg/MissionStatus.msg"
  "srv/SubmitRequest.srv"
  "srv/OperatorCommand.srv"
  "action/ApproachAndVerify.action"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
