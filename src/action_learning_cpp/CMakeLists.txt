cmake_minimum_required(VERSION 3.8)
project(action_learning_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── 查找依赖 ──
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ── 生成自定义 Action 消息 ──
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CountUp.action"
  "action/Fibonacci.action"
  "action/MoveArm.action"
  "action/Navigate.action"
)

# ── 获取生成的消息类型支持目标 ──
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# ── 辅助宏：创建可执行目标 ──
macro(add_action_executable name)
  add_executable(${name} src/${name}.cpp)
  ament_target_dependencies(${name} rclcpp rclcpp_action rclcpp_lifecycle lifecycle_msgs std_msgs geometry_msgs)
  target_link_libraries(${name} "${cpp_typesupport_target}")
  install(TARGETS ${name} DESTINATION lib/${PROJECT_NAME})
endmacro()

# ── 10 个演示节点 ──
add_action_executable(action_server_basic)
add_action_executable(action_client_basic)
add_action_executable(03_action_server_feedback)
add_action_executable(04_action_client_feedback)
add_action_executable(05_action_cancel_demo)
add_action_executable(06_action_server_policy)
add_action_executable(07_action_lifecycle_server)
add_action_executable(08_robot_arm_demo)
add_action_executable(09_robot_nav_demo)
add_action_executable(10_task_coordinator)

# ── 安装 launch 文件和文档 ──
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY doc/
  DESTINATION share/${PROJECT_NAME}/doc
)

ament_package()
