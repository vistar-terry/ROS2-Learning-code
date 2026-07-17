cmake_minimum_required(VERSION 3.8)
project(executor_learning_cpp)

# 默认 C++17 标准
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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(cm_executors REQUIRED)

# ============================================================
# 11 个演示节点
# ============================================================

# ① 单线程执行器基础 —— rclcpp::spin() 背后的原理
add_executable(single_threaded_basic src/single_threaded_basic.cpp)
target_include_directories(single_threaded_basic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(single_threaded_basic PUBLIC cxx_std_17)
ament_target_dependencies(single_threaded_basic rclcpp std_msgs)
install(TARGETS single_threaded_basic DESTINATION lib/${PROJECT_NAME})

# ② 多线程执行器 —— 并发回调与线程安全
add_executable(multi_threaded_demo src/multi_threaded_demo.cpp)
target_include_directories(multi_threaded_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(multi_threaded_demo PUBLIC cxx_std_17)
ament_target_dependencies(multi_threaded_demo rclcpp std_msgs sensor_msgs)
install(TARGETS multi_threaded_demo DESTINATION lib/${PROJECT_NAME})

# ③ 静态单线程执行器 —— 零分配高性能执行
add_executable(static_executor src/static_executor.cpp)
target_include_directories(static_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(static_executor PUBLIC cxx_std_17)
ament_target_dependencies(static_executor rclcpp std_msgs)
install(TARGETS static_executor DESTINATION lib/${PROJECT_NAME})

# ④ 执行器对比基准 —— 三种执行器性能测试
add_executable(executor_benchmark src/executor_benchmark.cpp)
target_include_directories(executor_benchmark PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(executor_benchmark PUBLIC cxx_std_17)
ament_target_dependencies(executor_benchmark rclcpp std_msgs)
install(TARGETS executor_benchmark DESTINATION lib/${PROJECT_NAME})

# ⑤ spin / spin_once / spin_some —— 三种驱动方式
add_executable(spin_methods src/spin_methods.cpp)
target_include_directories(spin_methods PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(spin_methods PUBLIC cxx_std_17)
ament_target_dependencies(spin_methods rclcpp std_msgs)
install(TARGETS spin_methods DESTINATION lib/${PROJECT_NAME})

# ⑥ 多节点单执行器 —— 一个执行器管理多个节点
add_executable(multi_node_single_executor src/multi_node_single_executor.cpp)
target_include_directories(multi_node_single_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(multi_node_single_executor PUBLIC cxx_std_17)
ament_target_dependencies(multi_node_single_executor rclcpp std_msgs sensor_msgs)
install(TARGETS multi_node_single_executor DESTINATION lib/${PROJECT_NAME})

# ⑦ 多执行器架构 —— 每个节点独立执行器 + 多线程
add_executable(multi_executor_arch src/multi_executor_arch.cpp)
target_include_directories(multi_executor_arch PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(multi_executor_arch PUBLIC cxx_std_17)
ament_target_dependencies(multi_executor_arch rclcpp std_msgs sensor_msgs)
install(TARGETS multi_executor_arch DESTINATION lib/${PROJECT_NAME})

# ⑧ 执行器与回调组配合 —— 回调组如何影响执行器调度
add_executable(executor_callback_groups src/executor_callback_groups.cpp)
target_include_directories(executor_callback_groups PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(executor_callback_groups PUBLIC cxx_std_17)
ament_target_dependencies(executor_callback_groups rclcpp std_msgs example_interfaces)
install(TARGETS executor_callback_groups DESTINATION lib/${PROJECT_NAME})

# ⑨ 机器人导航栈 —— 执行器驱动的多优先级控制架构
add_executable(robot_nav_executor src/robot_nav_executor.cpp)
target_include_directories(robot_nav_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_nav_executor PUBLIC cxx_std_17)
ament_target_dependencies(robot_nav_executor rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs)
install(TARGETS robot_nav_executor DESTINATION lib/${PROJECT_NAME})

# ⑩ 机器人多臂协调 —— 多执行器隔离不同子系统的实时性
add_executable(robot_multi_arm src/robot_multi_arm.cpp)
target_include_directories(robot_multi_arm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_multi_arm PUBLIC cxx_std_17)
ament_target_dependencies(robot_multi_arm rclcpp std_msgs sensor_msgs geometry_msgs)
install(TARGETS robot_multi_arm DESTINATION lib/${PROJECT_NAME})

# ⑪ EventsCBGExecutor —— 事件驱动 + 回调组（CBG）执行器
add_executable(events_cbg_executor src/events_cbg_executor.cpp)
target_include_directories(events_cbg_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(events_cbg_executor PUBLIC cxx_std_17)
ament_target_dependencies(events_cbg_executor rclcpp std_msgs cm_executors)

# 把兄弟包 cm_executors 的库目录写进二进制的 install RPATH。
# colcon/ament 默认不会把兄弟包的 lib 目录写进 RPATH，导致不 source 工作区 setup.bash
# 时运行报 "libcm_executors.so: cannot open shared object file"。
# 这里直接取 cm_executors 已安装 .so 的真实路径，无论 merge / 非 merge 布局都能正确定位。
set(_cm_executors_rpath "")
if(TARGET cm_executors::cm_executors)
  get_target_property(_cm_imp_loc cm_executors::cm_executors IMPORTED_LOCATION)
  if(_cm_imp_loc)
    get_filename_component(_cm_executors_rpath "${_cm_imp_loc}" DIRECTORY)
  endif()
endif()
if(NOT _cm_executors_rpath)
  # 按 colcon 非 merge 布局推导
  set(_cm_executors_rpath "${CMAKE_INSTALL_PREFIX}/../cm_executors/lib")
endif()
get_target_property(_existing_rpath events_cbg_executor INSTALL_RPATH)
if(_existing_rpath)
  set_target_properties(events_cbg_executor PROPERTIES
    INSTALL_RPATH "${_existing_rpath};${_cm_executors_rpath}")
else()
  set_target_properties(events_cbg_executor PROPERTIES
    INSTALL_RPATH "${_cm_executors_rpath}")
endif()

install(TARGETS events_cbg_executor DESTINATION lib/${PROJECT_NAME})

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
