cmake_minimum_required(VERSION 3.8)
project(exploration_manager)

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
find_package(visualization_msgs REQUIRED)
find_package(c2x_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(poly_traj REQUIRED)
find_package(active_perception REQUIRED)
find_package(traj_utils REQUIRED)
# bspline_opt's exported target links NLopt::nlopt_cxx; any consumer of
# plan_manage (which links bspline_opt) must resolve that imported target
# itself first - ament_export_dependencies can't propagate a non-ament CONFIG
# package like NLopt's split C/C++ Debian packaging.
find_package(NLopt REQUIRED CONFIG HINTS /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/nlopt_cxx)
find_package(plan_manage REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

include_directories(include ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

add_library(exploration_manager_core
  src/c2_exploration_manager.cpp
  src/c2_exploration_fsm.cpp
  src/copertura_camera.cpp
  src/copertura_visita.cpp
)

ament_target_dependencies(exploration_manager_core
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  c2x_msgs
  plan_env
  path_searching
  bspline
  bspline_opt
  poly_traj
  active_perception
  traj_utils
  plan_manage
  pcl_conversions
)
target_link_libraries(exploration_manager_core ${PCL_LIBRARIES})

target_include_directories(exploration_manager_core PUBLIC
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

add_executable(exploration_node src/exploration_node.cpp)
if(LIBDW_LIBRARY AND LIBDW_INCLUDE_DIR)
  target_compile_definitions(exploration_node PRIVATE BACKWARD_HAS_DW=1)
  target_link_libraries(exploration_node ${LIBDW_LIBRARY})
else()
  message(WARNING "libdw not found - exploration_node stack traces will have no symbols")
endif()
target_link_libraries(exploration_node exploration_manager_core)
ament_target_dependencies(exploration_node rclcpp)
install(TARGETS exploration_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  TARGETS exploration_manager_core
  EXPORT exploration_manager_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(exploration_manager_export HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(exploration_manager_core)
ament_export_dependencies(rclcpp nav_msgs geometry_msgs std_msgs visualization_msgs c2x_msgs
  plan_env path_searching bspline bspline_opt poly_traj active_perception plan_manage
  pcl_conversions Eigen3 PCL)

ament_package()
