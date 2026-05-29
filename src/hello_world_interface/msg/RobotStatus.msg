# 机器人状态信息
std_msgs/Header header
string robot_name
uint8 battery_level  # 电池电量百分比
bool is_connected    # 连接状态
float32 temperature  # 温度摄氏度
geometry_msgs/Pose2D current_pose  # 使用现有的geometry_msgs