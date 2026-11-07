cmake_minimum_required(VERSION 3.8)
project(c2x_worlds)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_package()
