cmake_minimum_required(VERSION 3.8)
project(hello_world_parameter_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(parameter_manager src/parameter_manager.cpp)
ament_target_dependencies(parameter_manager rclcpp)

add_executable(business_node src/business_node.cpp)
ament_target_dependencies(business_node rclcpp)

add_executable(single_param_node src/single_param_node.cpp)
ament_target_dependencies(single_param_node rclcpp)

add_executable(dynamic_param_modifier src/dynamic_param_modifier.cpp)
ament_target_dependencies(dynamic_param_modifier rclcpp)

# 安装目标
install(TARGETS
  parameter_manager
  business_node
  single_param_node
  dynamic_param_modifier
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
