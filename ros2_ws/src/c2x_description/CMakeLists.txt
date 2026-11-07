cmake_minimum_required(VERSION 3.8)
project(c2x_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_package()
