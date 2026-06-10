cmake_minimum_required(VERSION 3.8)
project(component_learning_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===== 查找依赖 =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ===== 组件共享库 =====
# ★ 关键：组件必须编译为共享库（SHARED），不能是静态库
add_library(${PROJECT_NAME}_components SHARED
  src/sensor_source_component.cpp
  src/data_filter_component.cpp
  src/data_display_component.cpp
)

target_include_directories(${PROJECT_NAME}_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME}_components PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}_components
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
)

# ★ 关键：注册组件节点
#    第一个参数是共享库 target 名
#    后续参数是组件类的完整限定名
rclcpp_components_register_nodes(${PROJECT_NAME}_components
  "SensorSourceComponent"
  "DataFilterComponent"
  "DataDisplayComponent"
)

# ★ 关键：安装共享库时，组件描述文件会自动安装到 ament index
#    EXPORT 关键字将 target 注册为可导出目标，供 ament_export_targets 使用
install(
  TARGETS ${PROJECT_NAME}_components
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

# ===== 导出依赖 =====
ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_components sensor_msgs std_msgs)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
