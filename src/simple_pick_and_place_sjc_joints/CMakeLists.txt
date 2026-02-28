cmake_minimum_required(VERSION 3.8)
project(simple_pick_and_place_sjc_joints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToJoints.action"
  DEPENDENCIES action_msgs
)

# Install directories
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/joint_action_server.py
  scripts/test_joint_action.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
