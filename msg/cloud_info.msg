# 点云信息消息定义
Header header # ROS标准消息头，包含时间戳和坐标系信息

# 每条激光线束的起始和结束点索引
int32[] startRingIndex  # 每条线束起始点在点云中的索引
int32[] endRingIndex    # 每条线束结束点在点云中的索引

# 点云在距离图像中的位置和距离信息
int32[]  pointColInd    # 点在距离图像中的列索引
float32[] pointRange    # 点到激光雷达的距离值

# 传感器数据可用性标志
int64 imuAvailable      # IMU数据是否可用（1表示可用，0表示不可用）
int64 odomAvailable     # 里程计数据是否可用（1表示可用，0表示不可用）

# LOAM算法初始化所需的姿态角（从IMU获取）
float32 imuRollInit     # IMU提供的初始横滚角（绕X轴旋转）
float32 imuPitchInit    # IMU提供的初始俯仰角（绕Y轴旋转）
float32 imuYawInit      # IMU提供的初始偏航角（绕Z轴旋转）

# IMU预积分提供的初始位姿估计
float32 initialGuessX   # 初始位置估计 - X坐标
float32 initialGuessY   # 初始位置估计 - Y坐标
float32 initialGuessZ   # 初始位置估计 - Z坐标
float32 initialGuessRoll  # 初始姿态估计 - 横滚角
float32 initialGuessPitch # 初始姿态估计 - 俯仰角
float32 initialGuessYaw   # 初始姿态估计 - 偏航角

# 点云数据
sensor_msgs/PointCloud2 cloud_deskewed  # 经过去畸变处理的原始点云
sensor_msgs/PointCloud2 cloud_corner    # 提取的角点特征点云
sensor_msgs/PointCloud2 cloud_surface   # 提取的平面特征点云