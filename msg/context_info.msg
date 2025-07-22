# Robot Name
string robotID

# Robot Name Receive
string robotIDReceive

# Cloud Info
Header header 

# Scan context dimension
# 扫描上下文的环形数量
int64 numRing

# 扫描上下文的扇区数量
int64 numSector

# 当前机器人的位姿信息
# 机器人在X轴上的位置
float32 poseX
# 机器人在Y轴上的位置
float32 poseY
# 机器人在Z轴上的位置
float32 poseZ
# 机器人绕X轴的旋转角度（翻滚角）
float32 poseRoll
# 机器人绕Y轴的旋转角度（俯仰角）
float32 posePitch
# 机器人绕Z轴的旋转角度（偏航角）
float32 poseYaw
# 机器人位姿的强度值
float32 poseIntensity

# 点云消息
# 提取的特征点云
sensor_msgs/PointCloud2 scanCloud  

# 扫描上下文信息
# 扫描上下文的二进制数据数组
float32[] scanContextBin
# 环形键信息数组
float32[] ringKey
