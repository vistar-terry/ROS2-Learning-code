cmake_minimum_required(VERSION 3.12)
project(diff_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_manager REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ============================================================
# 自定义硬件接口（用于真实机器人或 mock 仿真）
# 不使用 Gazebo 时，可以用此硬件接口进行测试
# ============================================================
add_library(diff_drive_hardware SHARED
  src/diff_drive_hardware.cpp
)
target_include_directories(diff_drive_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(diff_drive_hardware
  rclcpp hardware_interface pluginlib rclcpp_lifecycle
)

# ============================================================
# 简易里程计发布节点
# 从 /joint_states 读取轮子速度，计算并发布 odom → base_footprint TF
# ============================================================
add_executable(diff_odometry src/diff_odometry.cpp)
ament_target_dependencies(diff_odometry
  rclcpp sensor_msgs nav_msgs tf2 tf2_ros geometry_msgs
)

# 安装
install(TARGETS diff_drive_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS diff_odometry
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 插件描述文件
pluginlib_export_plugin_description_file(hardware_interface diff_robot_hardware/diff_robot_hardware.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
