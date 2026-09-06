cmake_minimum_required(VERSION 3.10)
project(franka_overhead_camera)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    models
    worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
