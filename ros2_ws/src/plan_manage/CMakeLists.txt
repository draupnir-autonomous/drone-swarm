cmake_minimum_required(VERSION 3.8)
project(plan_manage)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(c2x_msgs REQUIRED)
find_package(bspline REQUIRED)
# bspline_opt's exported target links NLopt::nlopt_cxx; ament_export_dependencies
# can't propagate a non-ament CONFIG package (NLopt's split C/C++ Debian
# packaging), so consumers must resolve this imported target themselves.
find_package(NLopt REQUIRED CONFIG HINTS /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/nlopt_cxx)
find_package(bspline_opt REQUIRED)
find_package(poly_traj REQUIRED)
find_package(path_searching REQUIRED)
find_package(plan_env REQUIRED)
find_package(active_perception REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

include_directories(include ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# planner_manager: trajectory generation orchestration, ties together
# path_searching/bspline_opt/active_perception. Matches the original catkin
# split: a library (linked by exploration_manager) plus a standalone
# traj_server executable that never needed it.
add_library(plan_manage
  src/planner_manager.cpp
)
ament_target_dependencies(plan_manage
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  bspline
  bspline_opt
  poly_traj
  path_searching
  plan_env
  active_perception
  pcl_conversions
)
target_link_libraries(plan_manage ${PCL_LIBRARIES})
target_include_directories(plan_manage PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Symbolized stack traces. backward.hpp (vendored in plan_manage/include) picks
# its symbol backend from BACKWARD_HAS_* macros; with none set it falls back to
# backtrace_symbol, which only reports the shared object and a raw address - the
# reason every segfault trace so far read "Object .../exploration_node, at
# 0x..., in " with an empty function name. libdw reads the DWARF this build
# already emits, giving function names and line numbers instead.
find_library(LIBDW_LIBRARY NAMES dw)
find_path(LIBDW_INCLUDE_DIR NAMES elfutils/libdw.h)

add_executable(traj_server src/traj_server.cpp)
if(LIBDW_LIBRARY AND LIBDW_INCLUDE_DIR)
  target_compile_definitions(traj_server PRIVATE BACKWARD_HAS_DW=1)
  target_link_libraries(traj_server ${LIBDW_LIBRARY})
else()
  message(WARNING "libdw not found - traj_server stack traces will have no symbols")
endif()

ament_target_dependencies(traj_server
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  visualization_msgs
  c2x_msgs
  bspline
  active_perception
  pcl_conversions
)
target_link_libraries(traj_server ${PCL_LIBRARIES})

target_include_directories(traj_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS plan_manage
  EXPORT plan_manage_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS traj_server DESTINATION lib/${PROJECT_NAME})

ament_export_targets(plan_manage_export HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(plan_manage)
ament_export_dependencies(rclcpp nav_msgs geometry_msgs std_msgs bspline bspline_opt poly_traj
  path_searching plan_env active_perception pcl_conversions Eigen3 PCL)

ament_package()
