cmake_minimum_required(VERSION 3.8)
project(c2x_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PositionCommand.msg"
  "msg/Bspline.msg"
  # Multi-UAV: task negotiation, neighbour discovery and map-chunk exchange.
  "msg/DroneState.msg"
  "msg/ExplorationReady.msg"
  "msg/MeetingOpt.msg"
  "msg/MeetingOptResponse.msg"
  "msg/Heartbeat.msg"
  "msg/IdxList.msg"
  "msg/ChunkStamps.msg"
  "msg/ChunkData.msg"
  "srv/SolveTSP.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
