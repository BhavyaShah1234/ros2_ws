cmake_minimum_required(VERSION 3.8)
project(simple_pick_and_place_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/cartesian_moveit_example.py
  scripts/joints_moveit_example.py
  scripts/highlevel_moveit_example.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
