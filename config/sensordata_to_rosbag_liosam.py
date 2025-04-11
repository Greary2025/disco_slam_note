# !/usr/bin/python
#
# 将给定目录中的传感器数据文件转换为单个rosbag文件。
#
# 调用方式:
#
#   python sensordata_to_rosbag.py 2012-01-08/ 2012-01-08.bag
#

import os
# import tf
import math 
import rosbag, rospy
from tqdm import tqdm
from std_msgs.msg import Float64, UInt16, Float64MultiArray, MultiArrayDimension, MultiArrayLayout, Header
from sensor_msgs.msg import CameraInfo, Imu, PointField, NavSatStatus, NavSatFix
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import TransformStamped, TwistStamped, Transform
from cv_bridge import CvBridge

import pandas as pd
import sys
import numpy as np
import struct
# from squaternion import Quaternion
# from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R

num_hits = 1024  # 每个激光雷达扫描的点数

# q_extR = Quaternion.from_euler(0.0, 0.0, 3.1415926/2.0)
# q_extR_T =  Quaternion.from_euler(0.0, 0.0, -3.1415926/2.0)

# def write_groundtruth():
#     odom_gt = np.loadtxt(sys.argv[1] + "odometry_mu_100hz.csv", delimiter = ",")
#     print(len(odom_gt))
#     for i in range(len(odom_gt)):
#         utime = odom_gt[i, 0]


def write_gps(gps, i, bag):
    """
    将GPS数据写入rosbag
    
    参数:
        gps: GPS数据数组
        i: 当前处理的GPS数据索引
        bag: 输出的rosbag对象
    """
    utime = gps[i, 0]  # 微秒时间戳
    mode = gps[i, 1]   # GPS模式

    lat = gps[i, 3]    # 纬度（弧度）
    lng = gps[i, 4]    # 经度（弧度）
    alt = gps[i, 5]    # 高度（米）

    timestamp = rospy.Time.from_sec(utime/1e6)  # 转换为ROS时间戳

    status = NavSatStatus()  # 创建GPS状态消息

    # 根据GPS模式设置状态
    if mode==0 or mode==1:
        status.status = NavSatStatus.STATUS_NO_FIX  # 无定位
    else:
        status.status = NavSatStatus.STATUS_FIX     # 有定位

    status.service = NavSatStatus.SERVICE_GPS  # 服务类型为GPS

    num_sats = UInt16()  # 卫星数量
    num_sats.data = gps[i, 2]

    fix = NavSatFix()  # 创建GPS定位消息
    fix.status = status
    fix.header.stamp = timestamp
    fix.latitude = np.rad2deg(lat)   # 将弧度转换为度
    fix.longitude = np.rad2deg(lng)  # 将弧度转换为度
    fix.altitude = alt

    track = Float64()  # 航向
    track.data = gps[i, 6]

    speed = Float64()  # 速度
    speed.data = gps[i, 7]

    bag.write('gps_fix', fix, t=timestamp)  # 写入rosbag

def write_gps_rtk(gps, i, bag):
    """
    将RTK GPS数据写入rosbag
    
    参数:
        gps: RTK GPS数据数组
        i: 当前处理的RTK GPS数据索引
        bag: 输出的rosbag对象
    """
    utime = gps[i, 0]  # 微秒时间戳
    mode = gps[i, 1]   # GPS模式

    lat = gps[i, 3]    # 纬度（弧度）
    lng = gps[i, 4]    # 经度（弧度）
    alt = gps[i, 5]    # 高度（米）

    timestamp = rospy.Time.from_sec(utime/1e6)  # 转换为ROS时间戳

    status = NavSatStatus()  # 创建GPS状态消息

    # 根据GPS模式设置状态
    if mode==0 or mode==1:
        status.status = NavSatStatus.STATUS_NO_FIX  # 无定位
    else:
        status.status = NavSatStatus.STATUS_FIX     # 有定位

    status.service = NavSatStatus.SERVICE_GPS  # 服务类型为GPS

    num_sats = UInt16()  # 卫星数量
    num_sats.data = gps[i, 2]

    fix = NavSatFix()  # 创建GPS定位消息
    fix.status = status
    fix.header.stamp = timestamp
    fix.latitude = np.rad2deg(lat)   # 将弧度转换为度
    fix.longitude = np.rad2deg(lng)  # 将弧度转换为度
    fix.altitude = alt

    track = Float64()  # 航向
    track.data = gps[i, 6]

    speed = Float64()  # 速度
    speed.data = gps[i, 7]

    bag.write('gps_rtk_fix', fix, t=timestamp)  # 写入rosbag

def write_ms25(ms25, ms25_euler, i, bag):
    """
    将IMU数据写入rosbag
    
    参数:
        ms25: IMU数据数组
        ms25_euler: IMU欧拉角数据数组
        i: 当前处理的IMU数据索引
        bag: 输出的rosbag对象
    """
    # 初始化旋转矩阵
    r_q = R.from_euler('zyx', [0, 0, 0], degrees=0)
    q = R.from_euler('zyx', [0, 0, 0], degrees=0).as_quat()

    # 创建从IMU到激光雷达的外参旋转矩阵
    r_extR = R.from_matrix([[0,-1,0],[-1,0,0],[0,0,-1]])
    q_extR = r_extR.as_quat()
    # R_imu_to_vel = ((0,-1,0),(-1,0,0),(0,0,-1))
    r_extR_T = r_extR.inv()  # 逆矩阵
    q_extR_T = r_extR_T.as_quat()
    # print(r_extR_T.as_matrix())
    print(len(ms25))
    print(len(ms25_euler))
    data_lenth = len(ms25) if len(ms25) <= len(ms25_euler) else len(ms25_euler)

    i = 0

    while i < data_lenth :
        utime = ms25[i, 0]  # 微秒时间戳

        # mag_x = ms25[i, 1]
        # mag_y = ms25[i, 2]
        # mag_z = ms25[i, 3]
        # q = r_q.as_quat()
        # print(q_extR)
        # print(q)
        # print(q_extR_T)

        if i > 0 :
            # 计算当前帧和上一帧的平均值，用于平滑
            accel_x = (ms25[i, 4] + ms25[i-1, 4]) * 0.5  # 加速度x
            accel_y = (ms25[i, 5] + ms25[i-1, 5]) * 0.5  # 加速度y
            accel_z = (ms25[i, 6] + ms25[i-1, 6]) * 0.5  # 加速度z

            rot_r = (ms25[i, 7] + ms25[i-1, 7]) * 0.5  # 角速度roll
            rot_p = (ms25[i, 8] + ms25[i-1, 8]) * 0.5  # 角速度pitch
            rot_h = (ms25[i, 9] + ms25[i-1, 9]) * 0.5  # 角速度heading

            r = (ms25_euler[i, 1] + ms25_euler[i, 1]) * 0.5  # 欧拉角roll
            p = (ms25_euler[i, 2] + ms25_euler[i, 2]) * 0.5  # 欧拉角pitch
            h = (ms25_euler[i, 3] + ms25_euler[i, 3]) * 0.5  # 欧拉角heading

            # 创建旋转矩阵并转换到激光雷达坐标系
            r_q = R.r = R.from_euler('xyz', [h, p, r], degrees=0)
            r_lid = r_extR * r_q * r_extR_T
            q_lid = r_lid.as_quat()
            timestamp = rospy.Time.from_sec((utime + utime_last) / 2e6)  # 计算平均时间戳

            # 创建IMU消息
            imu = Imu()
            imu.header.frame_id = 'imu_link'
            imu.header.stamp = timestamp
            imu.linear_acceleration.x = -float(accel_y)  # 坐标系转换
            imu.linear_acceleration.y = -float(accel_x)  # 坐标系转换
            imu.linear_acceleration.z = -float(accel_z)  # 坐标系转换
            imu.orientation.x = -q_lid[0]  # 四元数
            imu.orientation.y = -q_lid[1]  # 四元数
            imu.orientation.z = -q_lid[2]  # 四元数
            imu.orientation.w = -q_lid[3]  # 四元数
            imu.angular_velocity.x = -float(rot_p)  # 坐标系转换
            imu.angular_velocity.y = -float(rot_r)  # 坐标系转换
            imu.angular_velocity.z = -float(rot_h)  # 坐标系转换
            bag.write('imu_raw', imu, imu.header.stamp)  # 写入rosbag
        
        # 处理当前帧数据
        accel_x = ms25[i, 4]  # 加速度x
        accel_y = ms25[i, 5]  # 加速度y
        accel_z = ms25[i, 6]  # 加速度z

        rot_r = ms25[i, 7]  # 角速度roll
        rot_p = ms25[i, 8]  # 角速度pitch
        rot_h = ms25[i, 9]  # 角速度heading

        r = ms25_euler[i, 1]  # 欧拉角roll
        p = ms25_euler[i, 2]  # 欧拉角pitch
        h = ms25_euler[i, 3]  # 欧拉角heading

        # 创建旋转矩阵并转换到激光雷达坐标系
        r_q = R.r = R.from_euler('xyz', [h, p, r], degrees=0)
        r_lid = r_extR * r_q * r_extR_T
        q_lid = r_lid.as_quat()
        
        timestamp = rospy.Time.from_sec(utime / 1e6)  # 转换为ROS时间戳
        imu = Imu()
        imu.header.frame_id = 'imu_link'
        imu.header.stamp = timestamp
        imu.linear_acceleration.x = -float(accel_y)  # 坐标系转换
        imu.linear_acceleration.y = -float(accel_x)  # 坐标系转换
        imu.linear_acceleration.z = -float(accel_z)  # 坐标系转换
        imu.orientation.x = -q_lid[0]  # 四元数
        imu.orientation.y = -q_lid[1]  # 四元数
        imu.orientation.z = -q_lid[2]  # 四元数
        imu.orientation.w = -q_lid[3]  # 四元数
        imu.angular_velocity.x = -float(rot_p)  # 坐标系转换
        imu.angular_velocity.y = -float(rot_r)  # 坐标系转换
        imu.angular_velocity.z = -float(rot_h)  # 坐标系转换
        bag.write('imu_raw', imu, imu.header.stamp)  # 写入rosbag

        utime_last = utime
        i += 1
    

def write_ms25_euler(ms25_euler, i, bag):
    """
    将IMU欧拉角数据写入rosbag
    
    参数:
        ms25_euler: IMU欧拉角数据数组
        i: 当前处理的IMU欧拉角数据索引
        bag: 输出的rosbag对象
    """
    utime = ms25_euler[i, 0]  # 微秒时间戳

    r = ms25_euler[i, 1]  # 欧拉角roll
    p = ms25_euler[i, 2]  # 欧拉角pitch
    h = ms25_euler[i, 3]  # 欧拉角heading

    timestamp = rospy.Time.from_sec(utime/1e6)  # 转换为ROS时间戳

    # 创建多维数组布局
    layout_rph = MultiArrayLayout()
    layout_rph.dim = [MultiArrayDimension()]
    layout_rph.dim[0].label = "rph"
    layout_rph.dim[0].size = 3
    layout_rph.dim[0].stride = 1

    # 创建欧拉角消息
    euler = Float64MultiArray()
    euler.data = [r, p, h]
    euler.layout = layout_rph
    # bag.write('ms25_euler', euler, t=timestamp)  # 写入rosbag（当前被注释）

def convert_vel(x_s, y_s, z_s):
    """
    转换激光雷达点云坐标
    
    参数:
        x_s, y_s, z_s: 原始坐标
    
    返回:
        转换后的坐标
    """
    scaling = 0.005  # 缩放因子 5 mm
    offset = -100.0  # 偏移量

    x = x_s * scaling + offset
    y = y_s * scaling + offset
    z = z_s * scaling + offset

    return x, -y, -z  # 坐标系转换

def verify_magic(s):
    """
    验证魔数（用于确认数据包的有效性）
    
    参数:
        s: 包含魔数的字节串
    
    返回:
        魔数是否有效
    """
    magic = 44444  # 魔数值

    m = struct.unpack('<HHHH', s)  # 解包4个无符号短整型

    # 验证所有魔数是否匹配
    return len(m)>=3 and m[0] == magic and m[1] == magic and m[2] == magic and m[3] == magic

def read_first_vel_packet(f_vel, bag):
    """
    读取第一个激光雷达数据包
    
    参数:
        f_vel: 激光雷达数据文件
        bag: 输出的rosbag对象
    
    返回:
        数据包的时间戳
    """
    magic = f_vel.read(8)  # 读取魔数

    num_hits = struct.unpack('<I', f_vel.read(4))[0]  # 读取点数
    
    utime = struct.unpack('<Q', f_vel.read(8))[0]  # 读取时间戳

    f_vel.read(4)  # 读取填充字节

    # 读取所有点
    # data = []

    for i in range(num_hits):
        # 读取点的坐标和强度
        x = struct.unpack('<H', f_vel.read(2))[0]
        y = struct.unpack('<H', f_vel.read(2))[0]
        z = struct.unpack('<H', f_vel.read(2))[0]
        i = struct.unpack('B', f_vel.read(1))[0]
        l = struct.unpack('B', f_vel.read(1))[0]

    return utime
    
def write_vel(f_vel,bag):
    """
    读取激光雷达数据并写入rosbag
    
    参数:
        f_vel: 激光雷达数据文件
        bag: 输出的rosbag对象
    """
    size = os.path.getsize(sys.argv[1] + "velodyne_hits.bin")  # 获取文件大小
    print(size/28/32)
    pbar = tqdm(total=size)  # 创建进度条
    num_hits = 384  # 每个数据包的点数
    is_first = True
    last_time = 0
    last_packend_time = 0

    # 读取第一个数据包
    if is_first:
        is_first = False
        magic = f_vel.read(8)  # 读取魔数
        num_hits = struct.unpack('<I', f_vel.read(4))[0]  # 读取点数
        last_packend_time = last_time = struct.unpack('<Q', f_vel.read(8))[0]  # 读取时间戳
        f_vel.read(4)  # 读取填充字节
        for i in range(num_hits):
            # 读取点的坐标和强度
            x = struct.unpack('<H', f_vel.read(2))[0]
            y = struct.unpack('<H', f_vel.read(2))[0]
            z = struct.unpack('<H', f_vel.read(2))[0]
            i = struct.unpack('B', f_vel.read(1))[0]
            l = struct.unpack('B', f_vel.read(1))[0]
    data=[]
    while True:
        # a = f_vel.read(size-3)
        magic = f_vel.read(8)  # 读取魔数
        if len(magic) < 8:
            return

        if magic == '':  # 文件结束
            print("NO MAGIC")
            return

        if not verify_magic(magic):  # 验证魔数
            print("Could not verify magic")
            return 

        num_hits = struct.unpack('<I', f_vel.read(4))[0]  # 读取点数
        utime = struct.unpack('<Q', f_vel.read(8))[0]  # 读取时间戳
        f_vel.read(4)  # 读取填充字节
        pbar.update(24)  # 更新进度条
        
        # if utime > 1357847302646637:
        #     return
        
        # 初始化数组用于存储点云数据
        layer_point_num = np.zeros(32, dtype=np.int16)  # 每层点数
        yaw_ind = np.zeros((32,12), dtype=np.float32)  # 偏航角索引
        offset_time_ind = np.zeros((32,12), dtype=np.float32)  # 时间偏移索引
        offset_time_base = last_packend_time - last_time  # 基础时间偏移
        dt = float(utime - last_packend_time) / 12.0  # 时间步长
        l_last = 0
        N = 1

        # print(utime, num_hits, offset_time_base, dt)

        # 读取所有点
        for i in range(num_hits):
            # 读取点的坐标和强度
            x = struct.unpack('<H', f_vel.read(2))[0]
            y = struct.unpack('<H', f_vel.read(2))[0]
            z = struct.unpack('<H', f_vel.read(2))[0]
            i = struct.unpack('B', f_vel.read(1))[0]
            l = struct.unpack('B', f_vel.read(1))[0]

            # 计算时间偏移
            if l <= l_last:
                N += 1
            
            if N>12:
                N = 12

            l_last = l
            
            # layer_point_num[l] += 1
            # offset_time_ind[l][layer_point_num[l]] = offset_time_base + dt * N
            # if layer_point_num[l] >= 12:
            #     print(l, yaw_ind[l], offset_time_ind[l])
            
            # 转换坐标
            x, y, z = convert_vel(x, y, z)
            offset_time = int(offset_time_base + dt * N)
            if offset_time + last_time >= utime:
                offset_time = utime - last_time
            offset_time = float(offset_time)/1e6  # 转换为秒
            data.append([x, y, z, offset_time, l])  # 添加点到数据列表
            # if l == 31:
            #     print(l,offset_time_base + dt * N, int(offset_time_base + dt * N))
            # print(float(offset_time))
            # print(offset_time)
            pbar.update(8)  # 更新进度条
        
        last_packend_time = utime

        # 当积累了足够的点或时间间隔足够大时，创建点云消息并写入rosbag
        if utime - last_time > 1e5:
            # print(last_time / 1e6)
            # print(utime)
            header = Header()
            header.frame_id = 'velodyne'
            header.stamp = rospy.Time.from_sec(last_time/1e6)  # 转换为ROS时间戳
            
            # 定义点云字段
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),
                    # PointField('intensity', 12, PointField.FLOAT32, 1),
                    PointField('time', 16, PointField.FLOAT32, 1),
                    PointField('ring', 20, PointField.UINT16, 1)]
            
            # 创建点云消息
            pcl_msg = pcl2.create_cloud(header, fields, data)
            pcl_msg.is_dense = True
            bag.write("points_raw", pcl_msg, t=pcl_msg.header.stamp)  # 写入rosbag
            last_time = utime
            data=[]  # 清空数据列表

def main(args):
    """
    主函数
    
    参数:
        args: 命令行参数
    
    返回:
        执行状态码
    """
    if len(sys.argv) < 2:
        print('请指定传感器数据目录文件')
        return 1

    if len(sys.argv) < 3:
        print('请指定输出rosbag文件')
        return 1

    bag = rosbag.Bag(sys.argv[2], 'w')  # 创建输出rosbag文件

    # 加载各种传感器数据
    gps = np.loadtxt(sys.argv[1] + "gps.csv", delimiter = ",")
    gps_rtk = np.loadtxt(sys.argv[1] + "gps_rtk.csv", delimiter = ",")
    ms25 = np.loadtxt(sys.argv[1] + "ms25.csv", delimiter = ",")
    ms25_euler = np.loadtxt(sys.argv[1] + "ms25_euler.csv", delimiter = ",")

    # 如果需要更高频率的数据，可以使用以下代码进行插值
    # ms25_pd = pd.DataFrame(ms25)
    # ms25_median = ms25_pd[0].rolling(2).median()
    # ms25_median = pd.DataFrame(ms25_median.dropna())
    # ms25_median[[1,2,3,4,5,6,7,8,9]] = np.nan
    # ms25_pd = pd.concat([ms25_pd,ms25_median]).sort_values(by=[0]).interpolate()
    # ms25 = ms25_pd.to_numpy()
    #
    # ms25_euler_pd = pd.DataFrame(ms25_euler)
    # ms25_euler_median = ms25_euler_pd[0].rolling(2).median().dropna()
    # ms25_euler_median[[1,2,3]] = np.nan
    # ms25_euler_pd = pd.concat([ms25_euler_pd, ms25_euler_median]).sort_values(by=[0]).interpolate()
    # ms25_euler = ms25_euler_pd.to_numpy()

    # 初始化索引
    i_gps = 0
    i_gps_rtk = 0
    i_ms25 = 0
    i_ms25_euler = 0

    # 打开激光雷达数据文件
    f_vel = open(sys.argv[1] + "velodyne_hits.bin", "rb")

    # time_last = read_first_vel_packet(f_vel, bag)
    # data = []
    # utime_vel = time_last

    # write_groundtruth()
    write_vel(f_vel, bag)  # 处理激光雷达数据
    write_ms25(ms25, ms25_euler, i_ms25, bag)  # 处理IMU数据

    print('数据加载完成，正在写入ROSbag...')

    count = 0

    while 1:
        # 确定时间上的下一个数据包
        next_packet = "done"
        next_utime = -1  # 1357847302646637 
        count = count + 1

        # print(next_utime - utime_vel)

        # 根据时间戳确定下一个要处理的数据包类型
        if i_gps<len(gps) and (gps[i_gps, 0]<next_utime or next_utime<0):
            next_packet = "gps"

        if i_gps_rtk<len(gps_rtk) and (gps_rtk[i_gps_rtk, 0]<next_utime or next_utime<0):
            next_packet = "gps_rtk"

        if i_ms25<len(ms25) and (ms25[i_ms25, 0]<next_utime or next_utime<0):
            next_packet = "ms25"

        if i_ms25_euler<len(ms25_euler) and (ms25_euler[i_ms25_euler, 0]<next_utime or next_utime<0):
            next_packet = "ms25_euler"

        # if utime_vel>0 and (utime_vel<next_utime or next_utime<0):
        #     next_packet = "vel"

        # if utime_hok30>0 and (utime_hok30<next_utime or next_utime<0):
        #     next_packet = "hok30"

        # if utime_hok4>0 and (utime_hok4<next_utime or next_utime<0):
        #     next_packet = "hok4"

        # 处理下一个数据包
        if next_packet == "done":
            break
        elif next_packet == "gps":
            print("进度: {0}% \r".format(i_gps * 100.0 / len(gps[:,1])))
            write_gps(gps, i_gps, bag)
            i_gps = i_gps + 1
        elif next_packet == "gps_rtk":
            write_gps_rtk(gps_rtk, i_gps_rtk, bag)
            i_gps_rtk = i_gps_rtk + 1
        elif next_packet == "ms25":
            # write_ms25(ms25, ms25_euler, i_ms25, bag)
            i_ms25 = i_ms25 + 1
        elif next_packet == "ms25_euler":
            # write_ms25_euler(ms25_euler, i_ms25_euler, bag)
            i_ms25_euler = i_ms25_euler + 1
        # elif next_packet == "vel":
            # time_last, utime_vel, data = read_next_vel_packet(f_vel,time_last,data,bag)
        else:
            print("未知的数据包类型")

    # 关闭所有文件
    f_vel.close()
    # f_hok_30.close()
    # f_hok_4.close()
    bag.close()

    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv))