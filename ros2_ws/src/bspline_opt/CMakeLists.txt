cmake_minimum_required(VERSION 3.8)
project(bspline_opt)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bspline REQUIRED)
find_package(plan_env REQUIRED)
find_package(active_perception REQUIRED)
find_package(Eigen3 REQUIRED)
# Debian splits the NLopt cmake config into C (nlopt) and C++ (nlopt_cxx)
# packages; bspline_optimizer.cpp uses the C++ API (<nlopt.hpp>, nlopt::opt).
find_package(NLopt REQUIRED CONFIG HINTS /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/nlopt_cxx)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_library(bspline_opt src/bspline_optimizer.cpp)

ament_target_dependencies(bspline_opt rclcpp bspline plan_env active_perception)
target_link_libraries(bspline_opt NLopt::nlopt_cxx)

target_include_directories(bspline_opt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS bspline_opt
  EXPORT bspline_opt_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(bspline_opt_export HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(bspline_opt)
ament_export_dependencies(rclcpp bspline plan_env active_perception Eigen3)

ament_package()
