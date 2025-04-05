#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

// 激光雷达SLAM系统工具头文件
// 包含ROS消息处理、点云处理、坐标转换等实用功能

// ROS基础头文件
#include <ros/ros.h>

// ROS消息类型头文件
#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// OpenCV4计算机视觉库头文件
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// PCL点云处理库头文件
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

// TF坐标变换库头文件
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 
// C++标准库头文件
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;

// 定义点云类型为XYZI(坐标+强度)
typedef pcl::PointXYZI PointType;

// 激光雷达传感器类型枚举
enum class SensorType { VELODYNE, OUSTER };

/*
 * 参数服务器类
 * 用于从ROS参数服务器加载和存储所有配置参数
 */
class ParamServer
{
public:
    // ROS节点句柄
    ros::NodeHandle nh;

    // 机器人标识符
    std::string robot_id;

    // 以下是各类参数分组...

    // ROS话题名称配置
    string pointCloudTopic;  // 激光雷达点云话题名称
    string imuTopic;         // IMU传感器话题名称  
    string odomTopic;        // 里程计话题名称
    string gpsTopic;         // GPS定位话题名称

    // 坐标系名称配置
    string lidarFrame;      // 激光雷达坐标系名称
    string baselinkFrame;   // 机器人基座坐标系名称
    string odometryFrame;   // 里程计坐标系名称  
    string mapFrame;        // 地图坐标系名称

    // GPS相关参数
    bool useImuHeadingInitialization;  // 是否使用IMU初始化航向角
    bool useGpsElevation;              // 是否使用GPS高程数据
    float gpsCovThreshold;             // GPS协方差阈值
    float poseCovThreshold;            // 位姿协方差阈值

    // 点云保存配置
    bool savePCD;           // 是否保存点云到PCD文件
    string savePCDDirectory; // PCD文件保存路径

    // 激光雷达传感器配置
    SensorType sensor;      // 传感器类型(Velodyne/Ouster)
    int N_SCAN;            // 激光雷达垂直线数
    int Horizon_SCAN;       // 激光雷达水平点数
    int downsampleRate;     // 点云降采样率
    float lidarMinRange;    // 最小有效测量距离
    float lidarMaxRange;    // 最大有效测量距离

    // IMU传感器参数
    float imuAccNoise;      // 加速度计噪声
    float imuGyrNoise;      // 陀螺仪噪声
    float imuAccBiasN;      // 加速度计偏置噪声
    float imuGyrBiasN;      // 陀螺仪偏置噪声
    float imuGravity;       // 重力加速度值
    float imuRPYWeight;     // 姿态角权重
    vector<double> extRotV;  // 外参旋转矩阵(向量形式)
    vector<double> extRPYV;  // 外参欧拉角(向量形式)
    vector<double> extTransV; // 外参平移向量
    Eigen::Matrix3d extRot;  // 外参旋转矩阵
    Eigen::Matrix3d extRPY;  // 外参欧拉角矩阵
    Eigen::Vector3d extTrans; // 外参平移向量
    Eigen::Quaterniond extQRPY; // 外参四元数

    // LOAM特征提取参数
    float edgeThreshold;    // 边缘特征阈值
    float surfThreshold;     // 平面特征阈值
    int edgeFeatureMinValidNum;  // 边缘特征最小有效点数
    int surfFeatureMinValidNum;  // 平面特征最小有效点数

    // 体素滤波参数
    float odometrySurfLeafSize;    // 里程计平面特征体素尺寸
    float mappingCornerLeafSize;   // 建图边缘特征体素尺寸
    float mappingSurfLeafSize;    // 建图平面特征体素尺寸

    // 位姿优化容忍度
    float z_tollerance;       // Z轴高度容忍度
    float rotation_tollerance; // 旋转容忍度

    // 多线程处理参数
    int numberOfCores;        // 使用CPU核心数
    double mappingProcessInterval; // 建图处理间隔(秒)

    // 局部地图参数
    float surroundingkeyframeAddingDistThreshold; // 关键帧添加距离阈值
    float surroundingkeyframeAddingAngleThreshold; // 关键帧添加角度阈值
    float surroundingKeyframeDensity;  // 关键帧密度
    float surroundingKeyframeSearchRadius; // 关键帧搜索半径

    // 回环检测参数
    bool loopClosureEnableFlag;  // 是否启用回环检测
    float loopClosureFrequency;  // 回环检测频率
    int surroundingKeyframeSize;  // 局部关键帧数量
    float historyKeyframeSearchRadius; // 历史关键帧搜索半径
    float historyKeyframeSearchTimeDiff; // 历史关键帧时间差
    int historyKeyframeSearchNum;  // 历史关键帧搜索数量
    float historyKeyframeFitnessScore; // 历史关键帧匹配分数

    // 全局地图可视化参数
    float globalMapVisualizationSearchRadius; // 可视化搜索半径
    float globalMapVisualizationPoseDensity;  // 位姿显示密度
    float globalMapVisualizationLeafSize;     // 可视化点云体素尺寸

    int number_print;  // 调试打印频率控制

    ParamServer()
        {
            // 初始化私有节点句柄
            ros::NodeHandle n("~");
            
            // 加载调试打印频率参数
            n.param<int>("no", number_print, 100);
            // 加载机器人ID参数
            n.param<std::string>("robot_id", robot_id, "jackal0");
        
            // 加载ROS话题名称参数
            nh.param<std::string>("disco_slam/pointCloudTopic", pointCloudTopic, "points_raw");
            nh.param<std::string>("disco_slam/imuTopic", imuTopic, "imu_correct");
            nh.param<std::string>("disco_slam/odomTopic", odomTopic, "odometry/imu");
            nh.param<std::string>("disco_slam/gpsTopic", gpsTopic, "odometry/gps");
        
            // 加载坐标系名称参数
            nh.param<std::string>("disco_slam/lidarFrame", lidarFrame, "base_link");
            nh.param<std::string>("disco_slam/baselinkFrame", baselinkFrame, "base_link");
            nh.param<std::string>("disco_slam/odometryFrame", odometryFrame, "odom");
            nh.param<std::string>("disco_slam/mapFrame", mapFrame, "map");
        
            // 加载GPS相关参数
            nh.param<bool>("disco_slam/useImuHeadingInitialization", useImuHeadingInitialization, false);
            nh.param<bool>("disco_slam/useGpsElevation", useGpsElevation, false);
            nh.param<float>("disco_slam/gpsCovThreshold", gpsCovThreshold, 2.0);
            nh.param<float>("disco_slam/poseCovThreshold", poseCovThreshold, 25.0);
        
            // 加载点云保存参数
            nh.param<bool>("disco_slam/savePCD", savePCD, false);
            nh.param<std::string>("disco_slam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");
        
            // 加载激光雷达传感器类型参数
            std::string sensorStr;
            nh.param<std::string>("disco_slam/sensor", sensorStr, "");
            if (sensorStr == "velodyne") {
                sensor = SensorType::VELODYNE;
            } else if (sensorStr == "ouster") {
                sensor = SensorType::OUSTER;
            } else {
                ROS_ERROR_STREAM("Invalid sensor type (must be either 'velodyne' or 'ouster'): " << sensorStr);
                ros::shutdown();
            }
        
            // 加载激光雷达配置参数
            nh.param<int>("disco_slam/N_SCAN", N_SCAN, 16);
            nh.param<int>("disco_slam/Horizon_SCAN", Horizon_SCAN, 1800);
            nh.param<int>("disco_slam/downsampleRate", downsampleRate, 1);
            nh.param<float>("disco_slam/lidarMinRange", lidarMinRange, 1.0);
            nh.param<float>("disco_slam/lidarMaxRange", lidarMaxRange, 1000.0);
        
            // 加载IMU传感器参数
            nh.param<float>("disco_slam/imuAccNoise", imuAccNoise, 0.01);
            nh.param<float>("disco_slam/imuGyrNoise", imuGyrNoise, 0.001);
            nh.param<float>("disco_slam/imuAccBiasN", imuAccBiasN, 0.0002);
            nh.param<float>("disco_slam/imuGyrBiasN", imuGyrBiasN, 0.00003);
            nh.param<float>("disco_slam/imuGravity", imuGravity, 9.80511);
            nh.param<float>("disco_slam/imuRPYWeight", imuRPYWeight, 0.01);
            
            // 加载外参参数并转换为矩阵形式
            nh.param<vector<double>>("disco_slam/extrinsicRot", extRotV, vector<double>());
            nh.param<vector<double>>("disco_slam/extrinsicRPY", extRPYV, vector<double>());
            nh.param<vector<double>>("disco_slam/extrinsicTrans", extTransV, vector<double>());
            extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
            extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
            extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
            extQRPY = Eigen::Quaterniond(extRPY);
        
            // 加载特征提取参数
            nh.param<float>("disco_slam/edgeThreshold", edgeThreshold, 0.1);
            nh.param<float>("disco_slam/surfThreshold", surfThreshold, 0.1);
            nh.param<int>("disco_slam/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
            nh.param<int>("disco_slam/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);
        
            // 加载体素滤波参数
            nh.param<float>("disco_slam/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
            nh.param<float>("disco_slam/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
            nh.param<float>("disco_slam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);
        
            // 加载优化容忍度参数
            nh.param<float>("disco_slam/z_tollerance", z_tollerance, FLT_MAX);
            nh.param<float>("disco_slam/rotation_tollerance", rotation_tollerance, FLT_MAX);
        
            // 加载多线程处理参数
            nh.param<int>("disco_slam/numberOfCores", numberOfCores, 2);
            nh.param<double>("disco_slam/mappingProcessInterval", mappingProcessInterval, 0.15);
        
            // 加载局部地图参数
            nh.param<float>("disco_slam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
            nh.param<float>("disco_slam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
            nh.param<float>("disco_slam/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
            nh.param<float>("disco_slam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);
        
            // 加载回环检测参数
            nh.param<bool>("disco_slam/loopClosureEnableFlag", loopClosureEnableFlag, false);
            nh.param<float>("disco_slam/loopClosureFrequency", loopClosureFrequency, 1.0);
            nh.param<int>("disco_slam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
            nh.param<float>("disco_slam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
            nh.param<float>("disco_slam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
            nh.param<int>("disco_slam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
            nh.param<float>("disco_slam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);
        
            // 加载全局地图可视化参数
            nh.param<float>("disco_slam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
            nh.param<float>("disco_slam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
            nh.param<float>("disco_slam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);
        
            // 短暂延时确保参数加载完成
            usleep(100);
        }

    /*
     * IMU数据转换函数
     * 将原始IMU数据从传感器坐标系转换到机器人基座坐标系
     * @param imu_in 输入IMU消息(传感器坐标系)
     * @return 转换后的IMU消息(机器人基座坐标系)
     * 处理流程：
     * 1. 转换加速度数据
     * 2. 转换角速度数据  
     * 3. 转换姿态四元数
     * 4. 检查四元数有效性
     */
    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
    {
        // 创建输出消息并复制输入消息
        sensor_msgs::Imu imu_out = imu_in;
        
        // 转换加速度数据：将加速度从传感器坐标系转换到基座坐标系
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;  // 应用外参旋转矩阵
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        
        // 转换角速度数据：将角速度从传感器坐标系转换到基座坐标系
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;  // 应用外参旋转矩阵
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        
        // 转换姿态四元数：将姿态从传感器坐标系转换到基座坐标系
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extQRPY;  // 应用外参四元数旋转
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        // 检查四元数有效性：确保转换后的四元数是单位四元数
        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            ros::shutdown();
        }

        return imu_out;
    }
};


/*
 * 发布点云数据到ROS话题
 * @param thisPub ROS发布器指针
 * @param thisCloud 待发布的点云数据
 * @param thisStamp 时间戳
 * @param thisFrame 坐标系ID
 * @return 转换后的ROS点云消息
 */
sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, 
                                    pcl::PointCloud<PointType>::Ptr thisCloud,
                                    ros::Time thisStamp, 
                                    std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

/*
 * 从ROS消息获取时间戳(秒)
 * @param msg ROS消息指针(必须包含header)
 * @return 时间戳(秒)
 */
template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}

/*
 * IMU角速度数据转换到ROS消息格式
 * @param thisImuMsg 输出ROS IMU消息
 * @param angular_x 输出x轴角速度
 * @param angular_y 输出y轴角速度
 * @param angular_z 输出z轴角速度
 */
template<typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}

/*
 * IMU加速度数据转换到ROS消息格式
 * @param thisImuMsg 输入IMU消息指针
 * @param acc_x 输出x轴加速度
 * @param acc_y 输出y轴加速度
 * @param acc_z 输出z轴加速度
 */
template<typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}

/*
 * IMU姿态数据转换到欧拉角(roll/pitch/yaw)
 * @param thisImuMsg 输入IMU消息指针
 * @param rosRoll 输出横滚角(弧度)
 * @param rosPitch 输出俯仰角(弧度) 
 * @param rosYaw 输出偏航角(弧度)
 */
template<typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}

/*
 * 计算点到原点的欧氏距离
 * @param p 输入点坐标
 * @return 点到原点的距离
 */
float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

/*
 * 计算两点之间的欧氏距离
 * @param p1 第一个点坐标
 * @param p2 第二个点坐标
 * @return 两点之间的距离
 */
float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

#endif
