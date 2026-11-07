cmake_minimum_required(VERSION 3.8)
project(traj_utils)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(bspline REQUIRED)
find_package(poly_traj REQUIRED)
find_package(plan_env REQUIRED)
find_package(active_perception REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_library(traj_utils src/planning_visualization.cpp)

ament_target_dependencies(traj_utils
  rclcpp
  geometry_msgs
  visualization_msgs
  bspline
  poly_traj
  plan_env
  active_perception
)

target_include_directories(traj_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS traj_utils
  EXPORT traj_utils_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(traj_utils_export HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(traj_utils)
ament_export_dependencies(
  rclcpp geometry_msgs visualization_msgs bspline poly_traj plan_env active_perception Eigen3
)

ament_package()
