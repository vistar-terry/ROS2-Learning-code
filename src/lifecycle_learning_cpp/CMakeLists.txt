cmake_minimum_required(VERSION 3.8)
project(lifecycle_learning_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# ============================================================
# 10 个演示节点
# ============================================================

# ① 基础生命周期节点 —— 4个转换回调 + 状态查询
add_executable(lifecycle_basic src/lifecycle_basic.cpp)
target_include_directories(lifecycle_basic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lifecycle_basic PUBLIC cxx_std_17)
ament_target_dependencies(lifecycle_basic rclcpp rclcpp_lifecycle std_msgs lifecycle_msgs)
install(TARGETS lifecycle_basic DESTINATION lib/${PROJECT_NAME})

# ② 状态转换详解 —— 每个转换的含义、触发条件、返回值
add_executable(lifecycle_transitions src/lifecycle_transitions.cpp)
target_include_directories(lifecycle_transitions PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lifecycle_transitions PUBLIC cxx_std_17)
ament_target_dependencies(lifecycle_transitions rclcpp rclcpp_lifecycle std_msgs lifecycle_msgs)
install(TARGETS lifecycle_transitions DESTINATION lib/${PROJECT_NAME})

# ③ 生命周期发布器 —— LifecyclePublisher 按状态自动启用/禁用
add_executable(lifecycle_publisher src/lifecycle_publisher.cpp)
target_include_directories(lifecycle_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lifecycle_publisher PUBLIC cxx_std_17)
ament_target_dependencies(lifecycle_publisher rclcpp rclcpp_lifecycle std_msgs sensor_msgs)
install(TARGETS lifecycle_publisher DESTINATION lib/${PROJECT_NAME})

# ④ 生命周期订阅器 —— 按状态控制订阅行为
add_executable(lifecycle_subscriber src/lifecycle_subscriber.cpp)
target_include_directories(lifecycle_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lifecycle_subscriber PUBLIC cxx_std_17)
ament_target_dependencies(lifecycle_subscriber rclcpp rclcpp_lifecycle std_msgs sensor_msgs)
install(TARGETS lifecycle_subscriber DESTINATION lib/${PROJECT_NAME})

# ⑤ 外部状态管理器 —— 通过服务调用管理生命周期
add_executable(lifecycle_manager_node src/lifecycle_manager_node.cpp)
target_include_directories(lifecycle_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lifecycle_manager_node PUBLIC cxx_std_17)
ament_target_dependencies(lifecycle_manager_node rclcpp rclcpp_lifecycle std_msgs lifecycle_msgs)
install(TARGETS lifecycle_manager_node DESTINATION lib/${PROJECT_NAME})

# ⑥ Launch 编排生命周期 —— LifecycleNode + Launch 系统集成
add_executable(lifecycle_launch_node src/lifecycle_launch_node.cpp)
target_include_directories(lifecycle_launch_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lifecycle_launch_node PUBLIC cxx_std_17)
ament_target_dependencies(lifecycle_launch_node rclcpp rclcpp_lifecycle std_msgs)
install(TARGETS lifecycle_launch_node DESTINATION lib/${PROJECT_NAME})

# ⑦ 生命周期回调组 —— LifecycleNode + CallbackGroup 配合
add_executable(lifecycle_callback_groups src/lifecycle_callback_groups.cpp)
target_include_directories(lifecycle_callback_groups PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lifecycle_callback_groups PUBLIC cxx_std_17)
ament_target_dependencies(lifecycle_callback_groups rclcpp rclcpp_lifecycle std_msgs sensor_msgs)
install(TARGETS lifecycle_callback_groups DESTINATION lib/${PROJECT_NAME})

# ⑧ 机器人传感器驱动 —— 生命周期控制的传感器节点
add_executable(robot_sensor_driver src/robot_sensor_driver.cpp)
target_include_directories(robot_sensor_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_sensor_driver PUBLIC cxx_std_17)
ament_target_dependencies(robot_sensor_driver rclcpp rclcpp_lifecycle std_msgs sensor_msgs)
install(TARGETS robot_sensor_driver DESTINATION lib/${PROJECT_NAME})

# ⑨ 机器人导航子系统 —— 生命周期控制的安全关键系统
add_executable(robot_nav_lifecycle src/robot_nav_lifecycle.cpp)
target_include_directories(robot_nav_lifecycle PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_nav_lifecycle PUBLIC cxx_std_17)
ament_target_dependencies(robot_nav_lifecycle rclcpp rclcpp_lifecycle std_msgs geometry_msgs nav_msgs sensor_msgs)
install(TARGETS robot_nav_lifecycle DESTINATION lib/${PROJECT_NAME})

# ⑩ 机器人多子系统协调 —— 多生命周期节点的有序启停
add_executable(robot_system_coordinator src/robot_system_coordinator.cpp)
target_include_directories(robot_system_coordinator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_system_coordinator PUBLIC cxx_std_17)
ament_target_dependencies(robot_system_coordinator rclcpp rclcpp_lifecycle std_msgs sensor_msgs geometry_msgs nav_msgs)
install(TARGETS robot_system_coordinator DESTINATION lib/${PROJECT_NAME})

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
