cmake_minimum_required(VERSION 3.8)
project(path_searching)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plan_env REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_library(path_searching
  src/astar2.cpp
  src/kinodynamic_astar.cpp
)

ament_target_dependencies(path_searching rclcpp plan_env)

target_include_directories(path_searching PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS path_searching
  EXPORT path_searching_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(path_searching_export HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(path_searching)
ament_export_dependencies(rclcpp plan_env Eigen3)

ament_package()
