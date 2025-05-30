# 点云信息消息定义
Header header # ROS标准消息头，包含时间戳和坐标系信息

# 每条激光线束的起始和结束点索引
# 在进行点云去畸变时，把range数据展开成一维向量
# ring代表第几条激光线数，比如16线的激光雷达有16个ring
# start_ring_index记录第一个ring在一维向量中的起始索引
# end_ring_index记录第一个ring在一维向量中的结束索引
int32[] startRingIndex  # 每条线束起始点在点云中的索引
int32[] endRingIndex    # 每条线束结束点在点云中的索引

# 点云在距离图像中的位置和距离信息
int32[]  pointColInd    # 点在距离图像中的列索引
float32[] pointRange    # 所有点的range数值，展开为一维数组

# 传感器数据可用性标志
int64 imuAvailable      # IMU数据是否可用（1表示可用，0表示不可用）
int64 odomAvailable     # 里程计数据是否可用（1表示可用，0表示不可用）

# LOAM算法初始化所需的姿态角（从IMU获取）
# imuAvailable为1时，以下字段有效
# Attitude for LOAM initialization
# 从IMU原始数据获取的roll、pitch、yaw角初值
# 当上面的imu_avaliable为true，odom_available为false的时候，会被用来
# 当作这一帧点云的初值
float32 imuRollInit     # IMU提供的初始横滚角（绕X轴旋转）
float32 imuPitchInit    # IMU提供的初始俯仰角（绕Y轴旋转）
float32 imuYawInit      # IMU提供的初始偏航角（绕Z轴旋转）

# IMU预积分提供的初始位姿估计
# odomAvailable为1时，以下字段有效
# Initial guess from imu pre-integration
# 从imuPreintegration，IMU预积分模块获取的与这一帧雷达最接近的初始位姿
# 当上面的odom_available标签被置为true的时候会被使用
# 优先级比imu_xxx_init高（IMU做初始值猜测的只有初值）
float32 initialGuessX   # 初始位置估计 - X坐标
float32 initialGuessY   # 初始位置估计 - Y坐标
float32 initialGuessZ   # 初始位置估计 - Z坐标
float32 initialGuessRoll  # 初始姿态估计 - 横滚角
float32 initialGuessPitch # 初始姿态估计 - 俯仰角
float32 initialGuessYaw   # 初始姿态估计 - 偏航角

# 点云数据
# 在经过imageProjection模块后，这个数据字段被填充。在经过特征提取之后这个字段被清空
sensor_msgs/PointCloud2 cloud_deskewed  # 经过去畸变处理的原始点云
sensor_msgs/PointCloud2 cloud_corner    # 提取的角点特征点云
sensor_msgs/PointCloud2 cloud_surface   # 提取的平面特征点云