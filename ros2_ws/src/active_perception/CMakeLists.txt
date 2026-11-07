cmake_minimum_required(VERSION 3.8)
project(active_perception)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)

include_directories(include ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# heading_planner.cpp here only defines ctor/dtor stubs (see that file's
# comment) - the class is never instantiated (its one construction site in
# planner_manager.cpp is commented out even upstream), but FastPlannerManager
# still owns a unique_ptr<HeadingPlanner> member whose implicit destructor
# needs HeadingPlanner's destructor defined somewhere in the link graph.
add_library(active_perception
  src/perception_utils.cpp
  src/graph_node.cpp
  src/connectivity_graph.cpp
  src/uniform_grid.cpp
  src/hgrid.cpp
  src/frontier_finder.cpp
  src/heading_planner.cpp
)

ament_target_dependencies(active_perception
  rclcpp
  visualization_msgs
  pcl_conversions
  plan_env
  path_searching
)
target_link_libraries(active_perception ${PCL_LIBRARIES})

target_include_directories(active_perception PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS active_perception
  EXPORT active_perception_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(active_perception_export HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(active_perception)
ament_export_dependencies(rclcpp visualization_msgs pcl_conversions plan_env path_searching Eigen3 PCL)

ament_package()
