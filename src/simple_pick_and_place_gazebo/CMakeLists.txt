cmake_minimum_required(VERSION 3.8)
project(simple_pick_and_place_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  worlds
  models
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/block_spawner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
