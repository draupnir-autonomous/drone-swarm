cmake_minimum_required(VERSION 3.8)
project(plan_env)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_srvs REQUIRED)
find_package(c2x_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

add_library(plan_env
  src/sdf_map.cpp
  src/obj_predictor.cpp
  src/edt_environment.cpp
  src/raycast.cpp
  src/map_ros.cpp
  src/communication_graph.cpp
  src/multi_map_manager.cpp
)

ament_target_dependencies(plan_env
  rclcpp
  c2x_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  message_filters
  pcl_conversions
  std_srvs
)

target_link_libraries(plan_env ${PCL_LIBRARIES})

add_executable(plan_env_test_node src/plan_env_test_node.cpp)
target_link_libraries(plan_env_test_node plan_env)
ament_target_dependencies(plan_env_test_node rclcpp)
install(TARGETS plan_env_test_node DESTINATION lib/${PROJECT_NAME})

target_include_directories(plan_env PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  TARGETS plan_env
  EXPORT plan_env_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(plan_env_export HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(plan_env)
ament_export_dependencies(
  rclcpp
  c2x_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  message_filters
  pcl_conversions
  std_srvs
  Eigen3
  PCL
)

ament_package()
