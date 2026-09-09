cmake_minimum_required(VERSION 3.10)
project(maze_solver)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    models
    worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/verify_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
