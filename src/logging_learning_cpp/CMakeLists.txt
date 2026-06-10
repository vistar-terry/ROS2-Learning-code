cmake_minimum_required(VERSION 3.8)
project(logging_learning_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ============================================================
# 5 个演示节点
# ============================================================

# ① 基础日志使用 —— 五种日志级别与基本宏
add_executable(logging_basic src/logging_basic.cpp)
target_include_directories(logging_basic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(logging_basic PUBLIC cxx_std_17)
ament_target_dependencies(logging_basic rclcpp)
install(TARGETS logging_basic DESTINATION lib/${PROJECT_NAME})

# ② 日志级别控制 —— 运行时动态调整与外部配置
add_executable(logging_levels src/logging_levels.cpp)
target_include_directories(logging_levels PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(logging_levels PUBLIC cxx_std_17)
ament_target_dependencies(logging_levels rclcpp)
install(TARGETS logging_levels DESTINATION lib/${PROJECT_NAME})

# ③ 高级日志 —— 条件日志、限流日志、首次日志
add_executable(logging_advanced src/logging_advanced.cpp)
target_include_directories(logging_advanced PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(logging_advanced PUBLIC cxx_std_17)
ament_target_dependencies(logging_advanced rclcpp)
install(TARGETS logging_advanced DESTINATION lib/${PROJECT_NAME})

# ④ 日志性能与最佳实践
add_executable(logging_performance src/logging_performance.cpp)
target_include_directories(logging_performance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(logging_performance PUBLIC cxx_std_17)
ament_target_dependencies(logging_performance rclcpp)
install(TARGETS logging_performance DESTINATION lib/${PROJECT_NAME})

# ⑤ 机器人实战案例
add_executable(logging_robot_demo src/logging_robot_demo.cpp)
target_include_directories(logging_robot_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(logging_robot_demo PUBLIC cxx_std_17)
ament_target_dependencies(logging_robot_demo rclcpp sensor_msgs std_msgs)
install(TARGETS logging_robot_demo DESTINATION lib/${PROJECT_NAME})

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
