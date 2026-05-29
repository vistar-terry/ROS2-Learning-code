cmake_minimum_required(VERSION 3.8)
project(hello_world_service_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
)

# 服务端
add_executable(add_two_ints_server src/server.cpp)
ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)

# 同步客户端
add_executable(sync_add_two_ints_client src/sync_client.cpp)
ament_target_dependencies(sync_add_two_ints_client rclcpp example_interfaces)

# 异步客户端
add_executable(async_add_two_ints_client src/async_client.cpp)
ament_target_dependencies(async_add_two_ints_client rclcpp example_interfaces)

# 安装目标
install(TARGETS
  add_two_ints_server
  sync_add_two_ints_client
  async_add_two_ints_client
  DESTINATION lib/${PROJECT_NAME}
)

# 添加ament依赖导出
ament_export_dependencies(rclcpp)
ament_export_dependencies(example_interfaces)


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
