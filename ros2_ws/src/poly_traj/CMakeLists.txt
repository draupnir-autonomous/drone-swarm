cmake_minimum_required(VERSION 3.8)
project(poly_traj)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_library(poly_traj src/polynomial_traj.cpp)

target_include_directories(poly_traj PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS poly_traj
  EXPORT poly_traj_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(poly_traj_export HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(poly_traj)
ament_export_dependencies(Eigen3)

ament_package()
