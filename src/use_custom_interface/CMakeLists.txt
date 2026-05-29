cmake_minimum_required(VERSION 3.8)
project(use_custom_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(hello_world_interface REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
)

add_executable(msg_publisher_node src/msg_publisher_node.cpp)
ament_target_dependencies(msg_publisher_node
  rclcpp
  hello_world_interface
)

add_executable(msg_subscriber_node src/msg_subscriber_node.cpp)
ament_target_dependencies(msg_subscriber_node
  rclcpp
  hello_world_interface
)

add_executable(srv_client_node src/srv_client_node.cpp)
ament_target_dependencies(srv_client_node
  rclcpp
  hello_world_interface
)

add_executable(srv_server_node src/srv_server_node.cpp)
ament_target_dependencies(srv_server_node
  rclcpp
  hello_world_interface
)

add_executable(action_client_node src/action_client_node.cpp)
ament_target_dependencies(action_client_node
  rclcpp
  rclcpp_action
  hello_world_interface
)

add_executable(action_server_node src/action_server_node.cpp)
ament_target_dependencies(action_server_node
  rclcpp
  rclcpp_action
  hello_world_interface
)

install(TARGETS
  msg_publisher_node
  msg_subscriber_node
  srv_server_node
  srv_client_node
  action_server_node
  action_client_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
