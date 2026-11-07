cmake_minimum_required(VERSION 3.8)
project(c2x_px4_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(c2x_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(odometry_bridge src/odometry_bridge.cpp)
ament_target_dependencies(odometry_bridge rclcpp px4_msgs px4_ros_com nav_msgs Eigen3)

add_executable(lidar_lio_adapter src/lidar_lio_adapter.cpp)
ament_target_dependencies(lidar_lio_adapter rclcpp sensor_msgs nav_msgs Eigen3)

add_executable(lio_to_px4_bridge src/lio_to_px4_bridge.cpp)
ament_target_dependencies(lio_to_px4_bridge rclcpp nav_msgs px4_msgs Eigen3)

add_executable(cloud_transform_bridge src/cloud_transform_bridge.cpp)
ament_target_dependencies(cloud_transform_bridge rclcpp nav_msgs sensor_msgs message_filters
  pcl_conversions Eigen3 rosgraph_msgs)
target_link_libraries(cloud_transform_bridge ${PCL_LIBRARIES})

add_executable(position_command_bridge src/position_command_bridge.cpp)
ament_target_dependencies(position_command_bridge rclcpp px4_msgs px4_ros_com c2x_msgs Eigen3
  nav_msgs sensor_msgs pcl_conversions std_msgs)
target_link_libraries(position_command_bridge ${PCL_LIBRARIES})

add_executable(swarm_relay src/swarm_relay.cpp)
ament_target_dependencies(swarm_relay rclcpp c2x_msgs)

install(TARGETS odometry_bridge cloud_transform_bridge position_command_bridge swarm_relay
                lidar_lio_adapter lio_to_px4_bridge
  DESTINATION lib/${PROJECT_NAME})

ament_package()
