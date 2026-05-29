cmake_minimum_required(VERSION 3.12)
project(tf2_learning)

# 如果使用GCC或Clang编译器，启用所有警告
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# ============================================================
# 示例1：静态坐标广播器（C++）
# 发布一个不会随时间变化的固定坐标变换
# ============================================================
add_executable(static_tf_broadcaster src/static_tf_broadcaster.cpp)
ament_target_dependencies(static_tf_broadcaster
  rclcpp tf2_ros geometry_msgs std_msgs
)

# ============================================================
# 示例2：动态坐标广播器（C++）
# 发布一个随时间变化的坐标变换（模拟旋转/平移）
# ============================================================
add_executable(dynamic_tf_broadcaster src/dynamic_tf_broadcaster.cpp)
ament_target_dependencies(dynamic_tf_broadcaster
  rclcpp tf2_ros geometry_msgs std_msgs
)

# ============================================================
# 示例3：坐标监听器（C++）
# 订阅TF变换，查询两个坐标系之间的变换关系
# ============================================================
add_executable(tf_listener src/tf_listener.cpp)
ament_target_dependencies(tf_listener
  rclcpp tf2_ros tf2 geometry_msgs
)

# ============================================================
# 示例4：坐标点变换（C++）
# 将一个坐标系下的点变换到另一个坐标系下
# ============================================================
add_executable(tf_point_transform src/tf_point_transform.cpp)
ament_target_dependencies(tf_point_transform
  rclcpp tf2_ros tf2 tf2_geometry_msgs geometry_msgs
)

# ============================================================
# 示例5：机器人TF树广播器（C++）
# 模拟一个移动机器人的完整TF树：base_link -> sensor/actuator
# ============================================================
add_executable(robot_tf_broadcaster src/robot_tf_broadcaster.cpp)
ament_target_dependencies(robot_tf_broadcaster
  rclcpp tf2_ros geometry_msgs
)

# ============================================================
# 示例6：TF时间旅行（C++）
# 演示如何查询特定时间点的TF变换
# ============================================================
add_executable(tf_time_travel src/tf_time_travel.cpp)
ament_target_dependencies(tf_time_travel
  rclcpp tf2_ros tf2 geometry_msgs
)

# ============================================================
# 安装所有可执行目标
# ============================================================
install(TARGETS
  static_tf_broadcaster
  dynamic_tf_broadcaster
  tf_listener
  tf_point_transform
  robot_tf_broadcaster
  tf_time_travel
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装rviz配置文件
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
