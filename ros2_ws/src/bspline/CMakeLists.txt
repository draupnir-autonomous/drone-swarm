cmake_minimum_required(VERSION 3.8)
project(bspline)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_library(bspline src/non_uniform_bspline.cpp)
ament_target_dependencies(bspline rclcpp)

target_include_directories(bspline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS bspline
  EXPORT bspline_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(bspline_export HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(bspline)
ament_export_dependencies(rclcpp Eigen3)

ament_package()
