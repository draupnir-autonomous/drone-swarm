cmake_minimum_required(VERSION 3.10)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
project(fast_livo2)

set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set common compile options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

# Specific settings for Debug build
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")

# Detect CPU architecture
message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# Specific settings for Release build
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    # 64-bit ARM optimizations (e.g., RK3588 and Jetson Orin NX)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -ffast-math")
    message(STATUS "Using 64-bit ARM optimizations: -O3 -mcpu=native -mtune=native -ffast-math")
  else()
    # 32-bit ARM optimizations with NEON support
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
    message(STATUS "Using 32-bit ARM optimizations: -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
  endif()
  add_definitions(-DARM_ARCH)
else()
  # x86-64 (Intel/AMD) optimizations
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -mtune=native -funroll-loops") #-flto
  message(STATUS "Using general x86 optimizations: -O3 -march=native -mtune=native -funroll-loops") 
  add_definitions(-DX86_ARCH)
endif()

# Define project root directory
# ROOT_DIR (cartella dei sorgenti fissata in compilazione) non si usa piu': vedi
# cartella_livo() in include/common_lib.h (C2-Explorer, ottobre 2026).

# Per-frame LIO/VIO timing CSV export (adds fopen/fprintf overhead per frame,
# for offline latency analysis). Disabled by default for production builds;
# enable explicitly for experiments with -DENABLE_PERFRAME_TIMING=ON.
# C2-Explorer (ottobre 2026): acceso di serie, scrive Log/lio_timing.csv e Log/vio_timing.csv
# nella cartella della run ($C2X_LIVO_DIR): costa una riga di file per fotogramma.
option(ENABLE_PERFRAME_TIMING "Enable per-frame LIO/VIO timing CSV export" ON)
if(ENABLE_PERFRAME_TIMING)
  add_definitions(-DENABLE_PERFRAME_TIMING)
  message(STATUS "Per-frame LIO/VIO timing CSV export: ENABLED")
else()
  message(STATUS "Per-frame LIO/VIO timing CSV export: DISABLED (default)")
endif()

# Detect CPU core count for potential multithreading optimization
include(ProcessorCount)
ProcessorCount(N)
message(STATUS "Processor count: ${N}")

# Set the number of cores for multithreading
if(N GREATER 4)
  math(EXPR PROC_NUM "4")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
elseif(N GREATER 1)
  math(EXPR PROC_NUM "${N}")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
else()
  add_definitions(-DMP_PROC_NUM=1)
  message(STATUS "Single core detected. Multithreading disabled.")
endif()

# Check for OpenMP support
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
  add_compile_options(${OpenMP_CXX_FLAGS})
else()
  message(STATUS "OpenMP not found, proceeding without it")
endif()

# Check for mimalloc support
find_package(mimalloc QUIET)
if(mimalloc_FOUND)
  message(STATUS "mimalloc found")
else()
  message(STATUS "mimalloc not found, proceeding without it")
endif()

# Find ament and required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)  # C2-Explorer: /lio_salute
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(vikit_common REQUIRED)
find_package(vikit_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)

# C2-Explorer (ottobre 2026): Sophus 1.22 esporta solo il target Sophus::Sophus, non piu'
# la variabile Sophus_INCLUDE_DIRS usata qui sotto: si ricava dal target.
if(TARGET Sophus::Sophus AND NOT Sophus_INCLUDE_DIRS)
  get_target_property(Sophus_INCLUDE_DIRS Sophus::Sophus INTERFACE_INCLUDE_DIRECTORIES)
endif()
find_package(Boost REQUIRED COMPONENTS thread)
find_package(fmt REQUIRED)

get_filename_component(VIKIT_ROS_PREFIX "${vikit_ros_DIR}/../../.." ABSOLUTE)
find_library(VIKIT_ROS_LIBRARY
  NAMES vikit_ros
  HINTS "${VIKIT_ROS_PREFIX}/lib"
  REQUIRED
)

find_library(VIKIT_COMMON_LIBRARY
  NAMES vikit_common
  HINTS ${vikit_common_LIBRARY_DIRS}
  REQUIRED
)

message(STATUS "Using vikit_ros library: ${VIKIT_ROS_LIBRARY}")
message(STATUS "Using vikit_common library: ${VIKIT_COMMON_LIBRARY}")

# Include directories for dependencies
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${vikit_common_INCLUDE_DIRS}
  ${vikit_ros_INCLUDE_DIRS}
  include
)

set(dependencies
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  vikit_common
  vikit_ros
  image_transport
  pcl_ros
  pcl_conversions
  tf2_ros
  livox_ros_driver2
)

set(COMMON_DEPENDENCIES OpenMP::OpenMP_CXX fmt::fmt)

# Add libraries
add_library(vio src/vio.cpp src/frame.cpp src/visual_point.cpp)
add_library(lio src/voxel_map.cpp)
add_library(pre src/preprocess.cpp)
add_library(imu_proc src/IMU_Processing.cpp)
add_library(laser_mapping src/LIVMapper.cpp)

ament_target_dependencies(vio ${dependencies} )
ament_target_dependencies(lio ${dependencies})
ament_target_dependencies(pre ${dependencies})
ament_target_dependencies(imu_proc ${dependencies})
ament_target_dependencies(laser_mapping ${dependencies})

# linking libraries or executables to public dependencies
target_link_libraries(laser_mapping
  ${VIKIT_ROS_LIBRARY}
  ${VIKIT_COMMON_LIBRARY}
  ${COMMON_DEPENDENCIES}
)
target_link_libraries(vio
  ${VIKIT_COMMON_LIBRARY}
  ${COMMON_DEPENDENCIES}
)
target_link_libraries(lio ${COMMON_DEPENDENCIES})
target_link_libraries(pre ${COMMON_DEPENDENCIES})
target_link_libraries(imu_proc ${COMMON_DEPENDENCIES})

# Add the main executable
add_executable(fastlivo_mapping src/main.cpp)

ament_target_dependencies(fastlivo_mapping ${dependencies})

# Link libraries to the executable
target_link_libraries(fastlivo_mapping
  laser_mapping
  vio
  lio
  pre
  imu_proc
  ${VIKIT_ROS_LIBRARY}
  ${VIKIT_COMMON_LIBRARY}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Link mimalloc if found
if(mimalloc_FOUND)
  target_link_libraries(fastlivo_mapping mimalloc)
endif()

# Install the executable
install(TARGETS
  fastlivo_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  livox_ros_driver2
  Eigen3
  PCL
  OpenCV
  Sophus
)

ament_package()