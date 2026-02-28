cmake_minimum_required(VERSION 3.8)
project(simple_pick_and_place_sjc_cartesian)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToCartesian.action"
  DEPENDENCIES action_msgs geometry_msgs
)

# Install directories
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/cartesian_action_server.py
  scripts/test_cartesian_action.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
