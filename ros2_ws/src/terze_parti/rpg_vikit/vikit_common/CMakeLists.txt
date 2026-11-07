SET(PROJECT_NAME vikit_common)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 3.0)
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
SET(USE_ROS False) # Set False if you want to build this package without Catkin

# Set build flags. Set IS_ARM on odroid board as environment variable
SET(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
# C2-Explorer (Jetson, ottobre 2026): tolto il ramo che senza la variabile d'ambiente
# ARM_ARCHITECTURE aggiungeva opzioni solo x86 (-mmmx -msse ...) anche su aarch64; la
# scelta per architettura qui sotto basta.
message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
else()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a")
endif()

IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")   # era c++0x: Sophus 1.22 vuole almeno C++14
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Add plain cmake packages 
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)

# C2-Explorer (ottobre 2026): Sophus 1.22 esporta solo il target Sophus::Sophus, non piu'
# la variabile Sophus_INCLUDE_DIRS usata qui sotto: si ricava dal target.
if(TARGET Sophus::Sophus AND NOT Sophus_INCLUDE_DIRS)
  get_target_property(Sophus_INCLUDE_DIRS Sophus::Sophus INTERFACE_INCLUDE_DIRECTORIES)
endif()
FIND_PACKAGE(fmt REQUIRED)

# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  fmt
)

IF(USE_ROS)
  FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp cmake_modules)
  LIST(APPEND INCLUDE_DIRECTORIES ${catkin_INCLUDE_DIRS})
  catkin_package(
    DEPENDS Eigen OpenCV Sophus
    CATKIN_DEPENDS roscpp
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
  )
ENDIF()
# C2-Explorer (ottobre 2026): libreria e programmi di prova restano nella cartella di
# compilazione (prima finivano in bin/ e lib/ dentro i sorgenti).

# Set Sourcefiles
LIST(APPEND SOURCEFILES src/atan_camera.cpp
                        src/omni_camera.cpp
                        src/math_utils.cpp
                        src/vision.cpp
                        src/performance_monitor.cpp
                        src/robust_cost.cpp
                        src/user_input_thread.cpp
                        src/pinhole_camera.cpp
                        src/equidistant_camera.cpp
                        src/polynomial_camera.cpp
                        src/homography.cpp
                        src/img_align.cpp)

# Create vikit library
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${Sophus_LIBRARIES}
  fmt::fmt)

IF(USE_ROS)
  TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${catkin_LIBRARIES})
ENDIF()

# Tests (C2-Explorer: non compilati se non richiesti, non servono a FAST-LIVO2)
OPTION(VIKIT_COMMON_TESTS "programmi di prova di vikit_common" OFF)
IF(VIKIT_COMMON_TESTS)
ADD_EXECUTABLE(test_vk_common_camera test/test_camera.cpp)
TARGET_LINK_LIBRARIES(test_vk_common_camera ${PROJECT_NAME} ${OpenCV_LIBS})

ADD_EXECUTABLE(test_vk_common_triangulation test/test_triangulation.cpp)
TARGET_LINK_LIBRARIES(test_vk_common_triangulation ${PROJECT_NAME} ${OpenCV_LIBS})

ADD_EXECUTABLE(test_vk_common_patch_score test/test_patch_score.cpp)
TARGET_LINK_LIBRARIES(test_vk_common_patch_score ${PROJECT_NAME} ${OpenCV_LIBS})
ENDIF()


################################################################################
# C2-Explorer (ottobre 2026): la configurazione originale (vikit_commonConfig.cmake.in)
# puntava alla cartella di compilazione e la registrava con export(PACKAGE) in
# ~/.cmake/packages; il percorso della libreria era sbagliato (la libreria finiva nei
# sorgenti), per cui FAST-LIVO2 non la trovava. Qui si installa una configurazione che
# ricava i percorsi dalla propria posizione: vale dopo aver spostato il progetto.
IF(NOT USE_ROS)
  FILE(WRITE ${CMAKE_CURRENT_BINARY_DIR}/vikit_commonConfig.cmake [=[
get_filename_component(_vikit_common_prefix "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(vikit_common_INCLUDE_DIR  "${_vikit_common_prefix}/include")
set(vikit_common_INCLUDE_DIRS "${_vikit_common_prefix}/include")
set(vikit_common_LIBRARY_DIR  "${_vikit_common_prefix}/lib")
set(vikit_common_LIBRARY_DIRS "${_vikit_common_prefix}/lib")
set(vikit_common_LIBRARY      "${_vikit_common_prefix}/lib/libvikit_common.so")
set(vikit_common_LIBRARIES    "${_vikit_common_prefix}/lib/libvikit_common.so")
set(vikit_common_FOUND TRUE)
]=])
  INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/vikit_commonConfig.cmake
          DESTINATION share/${PROJECT_NAME}/cmake)
  INSTALL(DIRECTORY include/vikit DESTINATION include FILES_MATCHING PATTERN "*.h" )
  INSTALL(TARGETS ${PROJECT_NAME} DESTINATION lib )
ENDIF()
