cmake_minimum_required(VERSION 3.8)
project(lkh_mtsp_solver C CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)
# LKH.h declares its globals as plain tentative definitions (no extern),
# repeated verbatim in every .c file and in mtsp_node.cpp via lkh3_interface.h.
# That relies on pre-GCC10 common-symbol linkage; Ubuntu 24.04's GCC defaults
# to -fno-common, which turns each repeat into a conflicting strong symbol.
add_compile_options(-fcommon)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(c2x_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

file(GLOB LKH_C_SRCS src/*.c)

# LKH-3 solver core, ported verbatim from the ROS1 lkh_mtsp_solver package
# (167 files: pure C + one ROS-free C++ wrapper, zero ROS dependency).
#
# Must be SHARED, not static: LKH.h declares its ~150 global solver-state
# variables as plain (non-extern) globals, so both this library's
# lkh3_interface.cpp AND mtsp_node.cpp end up with their own defining copies
# (both transitively #include LKH.h). Static linking treats that as a real
# multiple-definition error in C++ (unlike C, -fcommon doesn't rescue this
# for C++ translation units). A shared library keeps mtsp_node.cpp's unused
# copies in a separate ELF module from the .so's real ones, exactly like the
# original catkin build (which also produces a shared liblkh_mtsp_solver.so).
add_library(lkh_mtsp_solver_core SHARED
  ${LKH_C_SRCS}
  src2/lkh3_interface.cpp
)
target_include_directories(lkh_mtsp_solver_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Unqualified #include "LKH.h" etc. inside the original .c/.cpp sources
# resolve against this directory, kept unmodified from upstream.
target_include_directories(lkh_mtsp_solver_core PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/lkh_mtsp_solver
)
target_link_libraries(lkh_mtsp_solver_core m)

add_executable(mtsp_node src2/mtsp_node.cpp)
target_link_libraries(mtsp_node lkh_mtsp_solver_core)
ament_target_dependencies(mtsp_node rclcpp c2x_msgs ament_index_cpp)

# Standalone solver binary used for the ACVRP allocation (see src2/lkh_main.cpp
# for why that one runs out-of-process).
add_executable(LKH src2/lkh_main.cpp)
target_link_libraries(LKH lkh_mtsp_solver_core)

install(TARGETS mtsp_node LKH DESTINATION lib/${PROJECT_NAME})

# LKH's scratch dir: exploration_node writes the .atsp/.par problem files here
# and mtsp_node reads them back, so both sides must agree on one path. Shipping
# it into share/ lets the launch files resolve it with
# get_package_share_directory instead of hardcoding an absolute source path.
install(
  DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

install(
  TARGETS lkh_mtsp_solver_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
