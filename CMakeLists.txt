cmake_minimum_required(VERSION 3.8)
project(om_custom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveTurtleBot.action"
  "action/RotateTurtleBot.action"
)

# Install Python package
# ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  om_custom/control_om.py
  om_custom/initial_pose.py
  om_custom/move_turtlebot_action_server.py
  om_custom/rotate_turtlebot_action_server.py
  om_custom/move_turtlebot_action_client.py
  om_custom/rotate_turtlebot_action_client.py
  om_custom/path_planning.py
  om_custom/path_planning2.py
  om_custom/move_turtlebot_node.py
  om_custom/rotate_turtlebot_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
