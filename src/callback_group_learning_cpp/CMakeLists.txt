cmake_minimum_required(VERSION 3.8)
project(callback_group_learning_cpp)

# 默认使用 C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(example_interfaces REQUIRED)

# ============================================================
# ① 默认行为演示：单线程执行器下的回调阻塞问题
# ============================================================
add_executable(default_behavior src/default_behavior.cpp)
target_include_directories(default_behavior PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(default_behavior)
ament_target_dependencies(default_behavior
  rclcpp std_msgs)
target_compile_options(default_behavior PRIVATE -Wall -Wextra)

# ============================================================
# ② MutuallyExclusive 回调组演示
# ============================================================
add_executable(mutually_exclusive_demo src/mutually_exclusive_demo.cpp)
target_include_directories(mutually_exclusive_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mutually_exclusive_demo)
ament_target_dependencies(mutually_exclusive_demo
  rclcpp std_msgs)
target_compile_options(mutually_exclusive_demo PRIVATE -Wall -Wextra)

# ============================================================
# ③ Reentrant 回调组演示
# ============================================================
add_executable(reentrant_demo src/reentrant_demo.cpp)
target_include_directories(reentrant_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(reentrant_demo)
ament_target_dependencies(reentrant_demo
  rclcpp std_msgs)
target_compile_options(reentrant_demo PRIVATE -Wall -Wextra)

# ============================================================
# ④ 执行器类型对比
# ============================================================
add_executable(executor_types src/executor_types.cpp)
target_include_directories(executor_types PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(executor_types)
ament_target_dependencies(executor_types
  rclcpp std_msgs)
target_compile_options(executor_types PRIVATE -Wall -Wextra)

# ============================================================
# ⑤ 多节点单执行器
# ============================================================
add_executable(multi_node_executor src/multi_node_executor.cpp)
target_include_directories(multi_node_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(multi_node_executor)
ament_target_dependencies(multi_node_executor
  rclcpp std_msgs sensor_msgs)
target_compile_options(multi_node_executor PRIVATE -Wall -Wextra)

# ============================================================
# ⑥ 机器人传感器融合实战
# ============================================================
add_executable(robot_sensor_fusion src/robot_sensor_fusion.cpp)
target_include_directories(robot_sensor_fusion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(robot_sensor_fusion)
ament_target_dependencies(robot_sensor_fusion
  rclcpp std_msgs sensor_msgs)
target_compile_options(robot_sensor_fusion PRIVATE -Wall -Wextra)

# ============================================================
# ⑦ 机器人导航栈实战
# ============================================================
add_executable(robot_nav_stack src/robot_nav_stack.cpp)
target_include_directories(robot_nav_stack PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(robot_nav_stack)
ament_target_dependencies(robot_nav_stack
  rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs)
target_compile_options(robot_nav_stack PRIVATE -Wall -Wextra)

# ============================================================
# ⑧ 死锁演示与解决
# ============================================================
add_executable(deadlock_demo src/deadlock_demo.cpp)
target_include_directories(deadlock_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(deadlock_demo)
ament_target_dependencies(deadlock_demo
  rclcpp std_msgs example_interfaces)
target_compile_options(deadlock_demo PRIVATE -Wall -Wextra)

# 安装可执行文件
install(TARGETS
  default_behavior
  mutually_exclusive_demo
  reentrant_demo
  executor_types
  multi_node_executor
  robot_sensor_fusion
  robot_nav_stack
  deadlock_demo
  DESTINATION lib/${PROJECT_NAME})

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# 安装文档
install(FILES CALLBACK_GROUP_CPP_GUIDE.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
