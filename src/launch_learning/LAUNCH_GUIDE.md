# ROS2 Launch 系统完全指南

> 本文档是 `launch_learning` 功能包的配套学习文档，全面覆盖 ROS2 Launch 系统的原理、语法、命令行工具、代码实现及实战设计模式。

---

## 目录

1. [Launch 系统概述](#1-launch-系统概述)
2. [Launch 文件三种格式](#2-launch-文件三种格式)
3. [Launch 核心概念](#3-launch-核心概念)
4. [命令行工具](#4-命令行工具)
5. [基础示例详解](#5-基础示例详解)
6. [参数传递](#6-参数传递)
7. [命名空间与重映射](#7-命名空间与重映射)
8. [条件启动](#8-条件启动)
9. [事件与处理器](#9-事件与处理器)
10. [IncludeLaunchDescription](#10-includelaunchdescription)
11. [Substitutions 替换系统](#11-substitutions-替换系统)
12. [综合实战案例](#12-综合实战案例)
13. [实用技巧与设计模式](#13-实用技巧与设计模式)
14. [常见问题与调试](#14-常见问题与调试)
15. [快速参考卡片](#15-快速参考卡片)

---

## 1. Launch 系统概述

### 1.1 什么是 Launch 系统？

ROS2 Launch 系统是一个**节点编排框架**，用于：

- **同时启动多个节点**：一条命令启动整个机器人系统
- **传递参数**：从命令行/YAML 文件向节点传递配置
- **条件执行**：根据条件决定是否启动某些节点
- **事件响应**：节点启动/退出时执行特定动作
- **模块化组织**：通过 include 机制组合多个 launch 文件

**没有 Launch 的世界：**
```bash
# 每个节点需要手动启动一个终端...
ros2 run package_a node_a --ros-args -p param1:=value1
ros2 run package_b node_b --ros-args -p param2:=value2
ros2 run package_c node_c --remap topic1:=topic2
# ... 一个复杂系统可能需要20+个终端！
```

**有 Launch 的世界：**
```bash
# 一条命令搞定！
ros2 launch my_robot bringup.launch.py use_sim:=true
```

### 1.2 Launch 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                  ROS2 Launch 系统                        │
│                                                         │
│  ┌──────────────┐    ┌──────────────┐                  │
│  │  Launch File  │    │  Launch File  │                  │
│  │  (.py/.xml)   │    │  (.yaml)      │                  │
│  └──────┬───────┘    └──────┬───────┘                  │
│         │                    │                           │
│         ▼                    ▼                           │
│  ┌─────────────────────────────────────┐                │
│  │       LaunchDescription             │                │
│  │  ┌─────────┐ ┌─────────┐           │                │
│  │  │ Action1 │ │ Action2 │  ...      │                │
│  │  │ (Node)  │ │ (Node)  │           │                │
│  │  └─────────┘ └─────────┘           │                │
│  │  ┌─────────┐ ┌─────────┐           │                │
│  │  │ Event   │ │ Include │  ...      │                │
│  │  │ Handler │ │ Launch  │           │                │
│  │  └─────────┘ └─────────┘           │                │
│  └─────────────────────────────────────┘                │
│         │                                               │
│         ▼                                               │
│  ┌─────────────────────────────────────┐                │
│  │       Launch Service                │                │
│  │  (解析、调度、执行所有 Action)        │                │
│  └─────────────────────────────────────┘                │
│         │                                               │
│         ▼                                               │
│  ┌────┐ ┌────┐ ┌────┐ ┌────┐                          │
│  │N1  │ │N2  │ │N3  │ │N4  │  ← ROS2 节点进程          │
│  └────┘ └────┘ └────┘ └────┘                          │
└─────────────────────────────────────────────────────────┘
```

### 1.3 Launch vs ros1 launch

| 特性 | ROS1 Launch | ROS2 Launch |
|------|------------|-------------|
| 格式 | 仅 XML | Python / XML / YAML |
| 编程能力 | 无 | 完整 Python |
| 事件处理 | 无 | OnProcessStart/Exit 等 |
| 条件逻辑 | `if`/`unless` 属性 | IfCondition + PythonExpression |
| 参数传递 | `$(arg name)` | LaunchConfiguration |
| 模块化 | `<include>` | IncludeLaunchDescription |
| 重映射 | `<remap>` | Node.remappings |
| 命名空间 | `<group ns="">` | PushRosNamespace |

---

## 2. Launch 文件三种格式

### 2.1 Python 格式（推荐）

**文件后缀：** `.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            output='screen',
        ),
    ])
```

**优点：** 功能最完整，支持所有特性
**缺点：** 语法较复杂

### 2.2 XML 格式

**文件后缀：** `.launch.xml`

```xml
<launch>
  <node pkg="my_package" exec="my_node" name="my_node" output="screen"/>
</launch>
```

**优点：** 简洁直观，类似 ROS1
**缺点：** 不支持事件处理和复杂逻辑

### 2.3 YAML 格式

**文件后缀：** `.launch.yaml`

```yaml
launch:
  - node:
      pkg: my_package
      exec: my_node
      name: my_node
      output: screen
```

**优点：** 最简洁，适合简单场景
**缺点：** 功能最有限

### 2.4 格式选择建议

```
选择决策树：

你的 launch 文件需要...
│
├── 只是简单启动几个节点？
│   └── YES → XML 或 YAML
│
├── 需要条件判断或事件处理？
│   └── YES → Python
│
├── 需要动态生成节点或复杂逻辑？
│   └── YES → Python
│
├── 多机器人系统？
│   └── YES → Python
│
└── 不确定？
    └── Python（功能最全，不会遇到天花板）
```

---

## 3. Launch 核心概念

### 3.1 Action（动作）

Action 是 Launch 系统的基本执行单元：

| Action | 作用 | 示例文件 |
|--------|------|---------|
| `Node` | 启动 ROS2 节点 | 01_basic |
| `ExecuteProcess` | 执行系统命令 | 10_tips |
| `IncludeLaunchDescription` | 包含其他 launch 文件 | 06_include |
| `TimerAction` | 延迟执行 | 05_events |
| `LogInfo` | 打印日志 | 多处 |
| `GroupAction` | 分组执行 | 03_namespace |
| `SetEnvironmentVariable` | 设置环境变量 | 10_tips |
| `DeclareLaunchArgument` | 声明参数 | 02_parameters |
| `RegisterEventHandler` | 注册事件处理器 | 05_events |
| `OpaqueFunction` | 执行 Python 函数 | 05_events |

### 3.2 Substitution（替换）

Substitution 是运行时才确定值的表达式：

| Substitution | 作用 | 示例 |
|---|---|---|
| `LaunchConfiguration('name')` | 获取 launch 参数 | `02_parameters` |
| `EnvironmentVariable('NAME')` | 获取环境变量 | `07_substitutions` |
| `PathJoinSubstitution([...])` | 拼接路径 | `07_substitutions` |
| `PythonExpression([...])` | Python 表达式 | `04_conditions` |
| `TextSubstitution(text='...')` | 纯文本 | `07_substitutions` |
| `FindPackageShare('pkg')` | 定位包路径 | `06_include` |

**关键理解：** Substitution 不是字符串，不能直接用 `+` 拼接。需要放在列表中：
```python
# ❌ 错误
'prefix_' + LaunchConfiguration('name')

# ✅ 正确
['prefix_', LaunchConfiguration('name')]
```

### 3.3 Condition（条件）

| Condition | 作用 |
|-----------|------|
| `IfCondition(substitution)` | 值为 'true' 时执行 |
| `UnlessCondition(substitution)` | 值为 'false' 时执行 |

### 3.4 Event Handler（事件处理器）

| 事件处理器 | 触发时机 |
|-----------|---------|
| `OnProcessStart` | 目标节点启动时 |
| `OnProcessExit` | 目标节点退出时 |
| `OnExecutionComplete` | 目标动作执行完成时 |

---

## 4. 命令行工具

### 4.1 ros2 launch

```bash
# 基本语法
ros2 launch <package_name> <launch_file_name>

# 传递参数
ros2 launch my_pkg my_launch.launch.py arg1:=value1 arg2:=value2

# 查看参数列表（不实际启动）
ros2 launch my_pkg my_launch.launch.py --show-args

# 调试模式
ros2 launch --debug my_pkg my_launch.launch.py

# 直接运行 launch 文件（不需要包安装）
ros2 launch /path/to/my_launch.launch.py

# 使用 XML 格式
ros2 launch my_pkg my_launch.launch.xml

# 使用 YAML 格式
ros2 launch my_pkg my_launch.launch.yaml
```

### 4.2 查看运行中的 launch

```bash
# 列出正在运行的 launch 进程
ros2 launch list

# 查看 launch 进程的详情
ros2 launch info <launch_id>
```

### 4.3 launch 相关的 ROS2 命令

```bash
# 查看节点列表
ros2 node list

# 查看话题列表
ros2 topic list

# 查看参数
ros2 param list
ros2 param get /node_name param_name

# 查看节点信息
ros2 node info /node_name
```

---

## 5. 基础示例详解

### 5.1 示例文件列表

| 文件 | 主题 | 知识点 |
|------|------|--------|
| `01_basic.launch.py` | 多节点启动 | Node、LaunchDescription |
| `02_parameters.launch.py` | 参数传递 | DeclareLaunchArgument、LaunchConfiguration、YAML |
| `03_namespace_remap.launch.py` | 命名空间与重映射 | namespace、remappings、GroupAction |
| `04_conditions.launch.py` | 条件启动 | IfCondition、UnlessCondition、PythonExpression |
| `05_events_handlers.launch.py` | 事件处理 | OnProcessStart/Exit、TimerAction、OpaqueFunction |
| `06_include.launch.py` | 包含子 launch | IncludeLaunchDescription |
| `07_substitutions.launch.py` | 替换系统 | 各种 Substitution 类型 |
| `08_full_robot.launch.py` | 综合实战 | 多机器人、条件、事件组合 |
| `09_xml_yaml.launch.py` | 格式对比 | Python/XML/YAML 对比 |
| `10_tips_patterns.launch.py` | 技巧模式 | ExecuteProcess、自动重启、动态选择 |

### 5.2 功能包目录结构

```
launch_learning/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── launch_learning
├── config/
│   ├── params.yaml                 # 参数配置文件
│   └── nav_params.yaml             # 导航场景参数
├── launch_learning/                # Python 模块（辅助节点）
│   ├── __init__.py
│   ├── param_talker.py             # 带参数的发布者
│   ├── ns_listener.py              # 命名空间感知的订阅者
│   └── worker_node.py              # 可重启的工作节点
├── launch/
│   ├── 01_basic.launch.py
│   ├── 02_parameters.launch.py
│   ├── 03_namespace_remap.launch.py
│   ├── 04_conditions.launch.py
│   ├── 05_events_handlers.launch.py
│   ├── 06_include.launch.py
│   ├── advanced/
│   │   ├── 07_substitutions.launch.py
│   │   ├── 08_full_robot.launch.py
│   │   ├── 09_xml_yaml.launch.py
│   │   └── 10_tips_patterns.launch.py
│   └── sub_launch/
│       └── robot_bringup.launch.py
└── test/
```

---

## 6. 参数传递

### 6.1 三种参数传递方式

#### 方式一：DeclareLaunchArgument + LaunchConfiguration

```python
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# 声明参数
arg = DeclareLaunchArgument('topic_name', default_value='chatter')

# 使用参数
topic = LaunchConfiguration('topic_name')

# 传给节点
Node(
    parameters=[{'topic_name': topic}],
    ...
)

# 命令行传入
# ros2 launch pkg file.launch.py topic_name:=my_topic
```

#### 方式二：YAML 参数文件

```python
# params.yaml
# param_talker:
#   ros__parameters:
#     topic_name: "sensor_data"

# 在 launch 中加载
import os
from launch_ros.substitutions import FindPackageShare

pkg_share = FindPackageShare('launch_learning').find('launch_learning')
params_file = os.path.join(pkg_share, 'config', 'params.yaml')

Node(
    parameters=[params_file],   # 直接传入 YAML 文件路径
    ...
)
```

#### 方式三：混合使用

```python
Node(
    parameters=[
        {'param_a': LaunchConfiguration('arg_a')},  # 字典方式
        params_file,                                  # YAML 文件方式
    ],
    # 后面的参数覆盖前面的（YAML 会覆盖字典中同名的参数）
    ...
)
```

### 6.2 参数覆盖优先级（从高到低）

1. **命令行参数**：`ros2 launch pkg file.launch.py arg:=value`
2. **launch 文件中的字典参数**：`{'param': value}`
3. **YAML 文件中的参数**
4. **节点代码中的默认值**：`self.declare_parameter('param', default)`

### 6.3 参数类型注意

`DeclareLaunchArgument` 的 `default_value` 和命令行传入的值**都是字符串**。需要数值时：
- 在 `Node.parameters` 中传递时，ROS2 会自动转换
- 在 `PythonExpression` 中使用时需要手动转换

```python
# 数值参数示例
freq_arg = DeclareLaunchArgument('freq', default_value='2.0')

# 方法1：传给节点参数（自动转换）
Node(parameters=[{'publish_freq': LaunchConfiguration('freq')}])

# 方法2：在 PythonExpression 中使用
double_freq = PythonExpression([
    'float("', LaunchConfiguration('freq'), '") * 2'
])
```

---

## 7. 命名空间与重映射

### 7.1 命名空间（Namespace）

```python
# 方式1：Node 的 namespace 参数
Node(
    package='my_pkg',
    executable='my_node',
    namespace='robot1',      # 节点变为 /robot1/my_node
)

# 方式2：GroupAction + PushRosNamespace（推荐，一组节点共享）
GroupAction([
    PushRosNamespace('robot1'),
    Node(package='my_pkg', executable='node_a'),
    Node(package='my_pkg', executable='node_b'),
    # node_a 和 node_b 都在 /robot1 命名空间下
])
```

**命名空间的效果：**
- 节点名：`/robot1/my_node`
- 话题名：`/robot1/chatter`
- 服务名：`/robot1/my_service`

### 7.2 重映射（Remappings）

```python
Node(
    package='my_pkg',
    executable='my_node',
    # 将节点内部使用的 'chatter' 映射为外部 'sensor_data'
    remappings=[
        ('chatter', 'sensor_data'),
        ('odom', 'wheel_odom'),    # 可以映射多个
    ],
)
```

**使用场景：**
- 复用节点代码，适配不同话题名
- 将标准话题名映射到自定义话题名
- 在不修改源码的情况下适配系统

---

## 8. 条件启动

### 8.1 IfCondition / UnlessCondition

```python
from launch.conditions import IfCondition, UnlessCondition

# 条件为 True 时启动
Node(
    condition=IfCondition(LaunchConfiguration('use_sim')),
    ...
)

# 条件为 False 时启动（与 IfCondition 相反）
Node(
    condition=UnlessCondition(LaunchConfiguration('use_sim')),
    ...
)
```

**注意：** LaunchConfiguration 返回的是字符串 `'true'` 或 `'false'`，IfCondition 会自动转换。

### 8.2 PythonExpression 复杂条件

```python
from launch.substitutions import PythonExpression

# 比较运算
condition=IfCondition(
    PythonExpression(['"', LaunchConfiguration('mode'), '" == "debug"'])
)

# 逻辑运算
condition=IfCondition(
    PythonExpression([
        LaunchConfiguration('use_sim'), ' and ',
        LaunchConfiguration('use_listener'),
    ])
)

# 数值比较
condition=IfCondition(
    PythonExpression([LaunchConfiguration('num'), ' >= 2'])
)
```

### 8.3 GroupAction 条件

```python
# 对整组动作应用条件
GroupAction(
    actions=[node1, node2, node3],
    condition=IfCondition(LaunchConfiguration('enable_group')),
)
```

---

## 9. 事件与处理器

### 9.1 OnProcessStart

```python
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

my_node = Node(...)

on_start = RegisterEventHandler(
    event_handler=OnProcessStart(
        target_action=my_node,        # 监听哪个动作
        on_start=[                     # 启动时执行的动作列表
            LogInfo(msg='节点已启动！'),
        ],
    )
)
```

### 9.2 OnProcessExit

```python
from launch.event_handlers import OnProcessExit

my_node = Node(...)

on_exit = RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=my_node,
        on_exit=[
            LogInfo(msg='节点已退出！'),
            another_node,               # 退出后启动新节点
        ],
    )
)
```

### 9.3 TimerAction 延迟执行

```python
from launch.actions import TimerAction

delayed_action = TimerAction(
    period=3.0,                      # 延迟3秒
    actions=[
        LogInfo(msg='3秒后执行'),
        some_node,
    ],
)
```

### 9.4 OpaqueFunction 执行任意 Python 代码

```python
from launch.actions import OpaqueFunction

def my_function(context):
    """可以在运行时执行任意 Python 代码"""
    # context.launch_configurations 获取所有参数的解析值
    mode = context.launch_configurations.get('mode', 'run')

    if mode == 'debug':
        return [Node(...)]    # 返回 Action 列表
    else:
        return [Node(...)]

opaque = OpaqueFunction(function=my_function)
```

### 9.5 关闭整个 Launch

```python
from launch.actions import EmitEvent
from launch.events import Shutdown

# 在任何事件处理器中使用
on_exit = RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=critical_node,
        on_exit=[
            LogInfo(msg='关键节点退出，关闭系统'),
            EmitEvent(event=Shutdown(reason='关键节点退出')),
        ],
    )
)
```

---

## 10. IncludeLaunchDescription

### 10.1 基本用法

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch_ros.substitutions import FindPackageShare

# 方式1：通过包名定位（推荐）
pkg_share = FindPackageShare('my_pkg').find('my_pkg')
sub_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_share, 'launch', 'sub.launch.py')
    ),
)

# 方式2：绝对路径（不推荐，不利于移植）
sub_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource('/absolute/path/to/sub.launch.py')
)
```

### 10.2 传递参数给子 launch

```python
sub_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(path),
    launch_arguments={
        'arg1': 'value1',
        'arg2': LaunchConfiguration('parent_arg'),  # 可以传递父 launch 的参数
    },
)
```

### 10.3 Include + GroupAction 实现多机器人

```python
robot1_group = GroupAction([
    PushRosNamespace('robot1'),
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments={'robot_name': 'robot1'},
    ),
])

robot2_group = GroupAction([
    PushRosNamespace('robot2'),
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments={'robot_name': 'robot2'},
    ),
])
```

---

## 11. Substitutions 替换系统

### 11.1 列表拼接规则

Substitution 在列表中自动拼接：
```python
# 等价于 "prefix_" + <robot_name的值> + "_suffix"
['prefix_', LaunchConfiguration('robot_name'), '_suffix']
```

### 11.2 常用 Substitution 速查

| 类型 | 用途 | 示例 |
|------|------|------|
| `LaunchConfiguration('name')` | 获取 launch 参数 | `02_parameters` |
| `EnvironmentVariable('NAME')` | 读取环境变量 | `07_substitutions` |
| `PathJoinSubstitution([...])` | 安全拼接路径 | `07_substitutions` |
| `PythonExpression([...])` | Python 表达式 | `04_conditions` |
| `FindPackageShare('pkg')` | 获取包路径 | `06_include` |
| `ThisLaunchFileDir()` | 当前 launch 文件目录 | 高级场景 |

### 11.3 在不同上下文中使用 Substitution

```python
# LogInfo 中
LogInfo(msg=['值: ', LaunchConfiguration('arg')])

# Node 名称中
Node(name=['my_node_', LaunchConfiguration('id')])

# Node 参数中
Node(parameters=[{'freq': LaunchConfiguration('freq')}])

# 路径中
PathJoinSubstitution([
    FindPackageShare('my_pkg'), 'config', LaunchConfiguration('config_file')
])
```

---

## 12. 综合实战案例

### 12.1 多机器人导航系统

对应文件：`advanced/08_full_robot.launch.py`

**场景描述：** 启动一个包含 1-2 个机器人的导航系统，支持仿真/真实模式切换。

**设计思路：**
1. 全局参数声明（num_robots, use_sim, use_rviz）
2. 每个机器人用 GroupAction + PushRosNamespace 隔离
3. 子 launch 文件负责单个机器人的节点启动
4. 条件启动控制 RViz 和仿真节点
5. 环境变量设置日志格式

### 12.2 节点自动重启

对应文件：`advanced/10_tips_patterns.launch.py`

```python
# 原理：OnProcessExit + 重新创建 Node
worker = Node(...)

on_exit = RegisterEventHandler(
    OnProcessExit(
        target_action=worker,
        on_exit=[
            LogInfo(msg='节点退出，重启中...'),
            # 重新创建相同的 Node
            Node(
                package='launch_learning',
                executable='worker_node',
                name='auto_restart_worker',
                ...
            ),
        ],
    )
)
```

### 12.3 传感器启动 launch（实战模式）

```python
# 传感器 launch 文件设计模式
def generate_launch_description():
    # 1. 声明参数
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    # 2. 从 YAML 加载默认参数
    params = os.path.join(pkg_share, 'config', 'sensor_params.yaml')

    # 3. 启动传感器驱动节点
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        parameters=[params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('scan', 'lidar_scan')],
    )

    # 4. 启动 IMU 驱动
    imu_node = Node(
        package='imu_driver',
        executable='imu_node',
        parameters=[params],
    )

    # 5. 静态 TF（传感器安装位置）
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'laser_link'],
    )

    return LaunchDescription([use_sim_time_arg, lidar_node, imu_node, static_tf])
```

---

## 13. 实用技巧与设计模式

### 13.1 launch 文件设计原则

1. **参数化一切**：硬编码的值都应该提取为 LaunchArgument
2. **模块化组织**：复杂系统拆分为子 launch 文件
3. **命名空间隔离**：多实例场景必须使用命名空间
4. **提供默认值**：每个 LaunchArgument 都应有合理的默认值
5. **添加描述**：为参数和 launch 文件添加 description

### 13.2 OpaqueFunction vs LaunchConfiguration

| 场景 | 选择 |
|------|------|
| 简单参数传递 | LaunchConfiguration |
| 需要类型转换 | OpaqueFunction |
| 需要条件判断生成不同节点 | OpaqueFunction |
| 需要访问文件系统 | OpaqueFunction |
| 需要动态计算 | OpaqueFunction |

### 13.3 参数文件设计模式

```python
# 模式：参数文件 + 命令行覆盖
Node(
    parameters=[
        params_file,                                     # 基础配置
        {'use_sim_time': LaunchConfiguration('sim')},    # 命令行覆盖
    ],
)
# 命令行传入的值优先级高于 YAML 文件
```

### 13.4 launch 文件的 return 模式

```python
# 模式1：直接返回（简单场景）
def generate_launch_description():
    return LaunchDescription([...])

# 模式2：先构建再返回（需要条件生成）
def generate_launch_description():
    actions = []
    actions.append(DeclareLaunchArgument(...))
    actions.append(Node(...))
    if some_condition:  # 注意：这里不能使用 LaunchConfiguration 做条件
        actions.append(Node(...))
    return LaunchDescription(actions)

# 模式3：使用 OpaqueFunction 做运行时条件判断
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(...),
        OpaqueFunction(function=dynamic_generator),
    ])
```

---

## 14. 常见问题与调试

### 14.1 常见错误

| 错误 | 原因 | 解决方法 |
|------|------|---------|
| `AttributeError: 'str' object has no attribute...` | 把 LaunchConfiguration 当字符串用 | 放在列表中拼接 |
| `LaunchConfiguration` 不生效 | 忘记声明 `DeclareLaunchArgument` | 在 LaunchDescription 中添加声明 |
| 节点启动后立即退出 | 参数类型不匹配 | 检查参数值是否为字符串 |
| `FileNotFoundError` for launch file | setup.py 中未安装 launch 文件 | 检查 data_files 配置 |
| YAML 参数不生效 | YAML 格式错误 | 检查缩进和键名 |
| 命名空间下话题不通 | 话题名包含命名空间前缀 | 检查话题名是否一致 |

### 14.2 调试步骤

**1. 查看参数**
```bash
ros2 launch pkg file.launch.py --show-args
```

**2. 检查节点是否启动**
```bash
ros2 node list
ros2 node info /node_name
```

**3. 检查话题**
```bash
ros2 topic list
ros2 topic echo /topic_name
```

**4. 检查参数**
```bash
ros2 param list
ros2 param get /node_name param_name
ros2 param dump /node_name
```

**5. 使用 debug 模式**
```bash
ros2 launch --debug pkg file.launch.py
```

### 14.3 setup.py 中安装 launch 文件的注意事项

```python
from glob import glob

# 必须在 data_files 中包含 launch 文件
data_files=[
    ('share/' + package_name + '/launch',
        glob('launch/*.launch.py')),
    # 子目录需要单独安装
    ('share/' + package_name + '/launch/sub_launch',
        glob('launch/sub_launch/*.launch.py')),
    # 配置文件
    ('share/' + package_name + '/config',
        glob('config/*.yaml')),
],
```

**常见陷阱：** 修改 launch 文件后需要重新 `colcon build` 才能生效，因为 launch 文件是被安装到 share 目录的。

---

## 15. 快速参考卡片

### 15.1 Launch 文件模板

```python
"""描述"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arg1', default_value='value1', description='...'),
        Node(
            package='pkg',
            executable='node',
            name='node',
            output='screen',
            parameters=[{'param1': LaunchConfiguration('arg1')}],
        ),
    ])
```

### 15.2 Action 速查

```python
# Node
Node(package='pkg', executable='exec', name='name',
     namespace='ns', output='screen'|'log',
     parameters=[{...}, 'file.yaml'],
     remappings=[('old', 'new')],
     condition=IfCondition(...),
     arguments=['arg1', 'arg2'])

# IncludeLaunchDescription
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(path),
    launch_arguments={'key': 'value'})

# TimerAction
TimerAction(period=3.0, actions=[...])

# LogInfo
LogInfo(msg=['text', substitution])

# ExecuteProcess
ExecuteProcess(cmd=['cmd', 'arg1'], output='screen')

# GroupAction
GroupAction(actions=[node1, node2])

# RegisterEventHandler
RegisterEventHandler(OnProcessExit(
    target_action=node, on_exit=[...]))
```

### 15.3 条件速查

```python
IfCondition(LaunchConfiguration('flag'))         # flag == 'true'
UnlessCondition(LaunchConfiguration('flag'))     # flag == 'false'
IfCondition(PythonExpression(['expr']))          # 复杂条件
```

### 15.4 命令行速查

```bash
ros2 launch pkg file.launch.py                    # 启动
ros2 launch pkg file.launch.py key:=value         # 传参
ros2 launch pkg file.launch.py --show-args        # 查看参数
ros2 launch --debug pkg file.launch.py            # 调试模式
ros2 node list                                     # 查看节点
ros2 topic list                                    # 查看话题
ros2 param list                                    # 查看参数
```

---

## 附录：编译与运行

### 编译

```bash
cd z:/learning/ROS2-Learning-main/code

# 编译
colcon build --packages-select launch_learning

# 刷新环境
source install/setup.bash    # Linux
# install/setup.ps1          # Windows PowerShell
```

### 运行示例

```bash
# 基础示例
ros2 launch launch_learning 01_basic.launch.py
ros2 launch launch_learning 02_parameters.launch.py
ros2 launch launch_learning 03_namespace_remap.launch.py
ros2 launch launch_learning 04_conditions.launch.py
ros2 launch launch_learning 05_events_handlers.launch.py
ros2 launch launch_learning 06_include.launch.py

# 高级示例
ros2 launch launch_learning 07_substitutions.launch.py
ros2 launch launch_learning 08_full_robot.launch.py
ros2 launch launch_learning 09_xml_yaml.launch.py
ros2 launch launch_learning 10_tips_patterns.launch.py

# 带参数运行
ros2 launch launch_learning 02_parameters.launch.py topic_name:=my_topic freq:=5.0
ros2 launch launch_learning 04_conditions.launch.py use_listener:=true mode:=debug
ros2 launch launch_learning 08_full_robot.launch.py num_robots:=2 use_rviz:=true

# 查看参数
ros2 launch launch_learning 02_parameters.launch.py --show-args
```
