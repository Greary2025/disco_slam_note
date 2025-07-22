// 包含自定义工具头文件，该文件可能包含一些通用的工具函数、类型定义等
#include "utility.h"
// 包含 disco_double 包中的 cloud_info 消息类型头文件，用于处理点云信息
// #include "disco_double/cloud_info.h"
#include "disco_double/ring_cloud_info.h"
// 包含 disco_double 包中的 context_info 消息类型头文件，用于处理上下文信息
#include "disco_double/context_info.h"

// 包含 gtsam 库中旋转矩阵相关的头文件，用于处理三维旋转
#include <gtsam/geometry/Rot3.h>
// 包含 gtsam 库中三维位姿相关的头文件，用于表示和处理三维空间中的位姿
#include <gtsam/geometry/Pose3.h>
// 包含 gtsam 库中先验因子相关的头文件，用于添加先验约束到因子图中
#include <gtsam/slam/PriorFactor.h>
// 包含 gtsam 库中两个位姿之间的因子相关的头文件，用于添加两个位姿之间的约束
#include <gtsam/slam/BetweenFactor.h>
// 包含 gtsam 库中 GPS 因子相关的头文件，用于添加 GPS 测量值到因子图中
#include <gtsam/navigation/GPSFactor.h>
// 包含 gtsam 库中 IMU 因子相关的头文件，用于添加 IMU 测量值到因子图中
#include <gtsam/navigation/ImuFactor.h>
// 包含 gtsam 库中组合 IMU 因子相关的头文件，用于处理更复杂的 IMU 测量信息
#include <gtsam/navigation/CombinedImuFactor.h>
// 包含 gtsam 库中非线性因子图相关的头文件，用于构建和管理非线性因子图
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
// 包含 gtsam 库中 Levenberg-Marquardt 优化器相关的头文件，用于对非线性因子图进行优化
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
// 包含 gtsam 库中边缘概率相关的头文件，用于计算因子图中变量的边缘概率
#include <gtsam/nonlinear/Marginals.h>
// 包含 gtsam 库中值容器相关的头文件，用于存储和管理因子图中的变量值
#include <gtsam/nonlinear/Values.h>
// 包含 gtsam 库中符号相关的头文件，用于在因子图中标识变量
#include <gtsam/inference/Symbol.h>
// 包含 gtsam 库中增量平滑与建图（ISAM2）相关的头文件，用于实时增量式地优化因子图
#include <gtsam/nonlinear/ISAM2.h>

// 包含 gtsam 库中数据集相关的头文件，可能用于加载和处理预定义的数据集
#include <gtsam/slam/dataset.h>

#include <fstream>
#include <iostream>
#include <string>


// 使用 gtsam 命名空间，方便直接使用 gtsam 库中的类和函数
using namespace gtsam;

// 使用符号简写，X 代表三维位姿 (x, y, z, roll, pitch, yaw)
using symbol_shorthand::X; 
// 使用符号简写，V 代表速度 (xdot, ydot, zdot)
using symbol_shorthand::V; 
// 使用符号简写，B 代表偏置 (ax, ay, az, gx, gy, gz)
using symbol_shorthand::B; 
// 使用符号简写，G 代表 GPS 位姿
using symbol_shorthand::G; 

/**
 * @brief 包含 6D 位姿信息的点云类型，强度值存储时间戳
 * 
 * 该结构体扩展了 PCL 点类型，包含三维坐标、强度、欧拉角（roll, pitch, yaw）和时间戳。
 */
struct PointXYZIRPYT
{
    // 添加三维坐标和填充字段
    PCL_ADD_POINT4D
    // 添加强度字段，这是推荐的添加 XYZ + 填充的方式
    PCL_ADD_INTENSITY;                  
    // 绕 X 轴的旋转角度（弧度）
    float roll;
    // 绕 Y 轴的旋转角度（弧度）
    float pitch;
    // 绕 Z 轴的旋转角度（弧度）
    float yaw;
    // 时间戳
    double time;
    // 确保内存分配对齐，以支持 SSE 指令集
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
} EIGEN_ALIGN16;                    // 强制 SSE 填充以保证正确的内存对齐

// 注册自定义点云结构体，方便 PCL 库使用
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

// 定义 PointTypePose 作为 PointXYZIRPYT 的别名
typedef PointXYZIRPYT  PointTypePose;

void saveToFile(const std::string& content, const std::string& filename) {
    string dir = "/home/john/catkin_ws/src/disco_slam_note/results/";
    try {
        std::ofstream file(dir + filename);
        if (file.is_open()) {
            file << content << std::endl;
            file.close();
            // std::cout << "successed" << std::endl;
        } else {
            // std::cerr << "Failed to open file: " << filename << std::endl;
        }
    } catch (const std::exception& e) {
        // std::cerr << "Error occurred: " << e.what() << std::endl;
    }
}

/**
 * @brief 地图优化类，继承自 ParamServer，用于处理地图优化相关任务
 * 
 * 该类包含了与地图优化相关的成员变量和方法，如 GTSAM 因子图、ROS 发布者和订阅者、点云数据等。
 */
class mapOptimization : public ParamServer
{

public:

    // GTSAM 非线性因子图，用于存储各种约束因子
    NonlinearFactorGraph gtSAMgraph;
    // GTSAM 初始估计值，存储变量的初始猜测值（可有多种因子的初始值）
    Values initialEstimate;
    // GTSAM 优化后的估计值
    Values optimizedEstimate;
    // GTSAM 增量平滑与建图（ISAM2）优化器指针
    ISAM2 *isam;
    // GTSAM 当前的估计值
    Values isamCurrentEstimate;
    // 位姿协方差矩阵，用于存储位姿估计的不确定性
    Eigen::MatrixXd poseCovariance;

    // ROS 发布者，发布全局地图点云
    ros::Publisher pubLaserCloudSurround;
    // ROS 发布者，发布全局激光里程计信息
    ros::Publisher pubLaserOdometryGlobal;
    // ROS 发布者，发布增量式激光里程计信息
    ros::Publisher pubLaserOdometryIncremental;
    // ROS 发布者，发布关键位姿点云
    ros::Publisher pubKeyPoses;
    // ROS 发布者，发布全局路径信息
    ros::Publisher pubPath;

    // ROS 发布者，发布历史关键帧点云，用于回环检测
    ros::Publisher pubHistoryKeyFrames;
    // ROS 发布者，发布经过 ICP 校正后的关键帧点云
    ros::Publisher pubIcpKeyFrames;
    // ROS 发布者，发布最近的关键帧点云
    ros::Publisher pubRecentKeyFrames;
    // ROS 发布者，发布最近的单个关键帧点云
    ros::Publisher pubRecentKeyFrame;
    // ROS 发布者，发布原始注册后的点云
    ros::Publisher pubCloudRegisteredRaw;
    // ROS 发布者，发布回环约束边的可视化标记
    ros::Publisher pubLoopConstraintEdge;

    // ROS 发布者，发布激光点云信息
    ros::Publisher pubLaserCloudInfo;

    // 发布到融合节点的 ROS 发布者（注释掉，暂未使用）
    // ros::Publisher pubCloudInfoWithPose;
    // disco_double::cloud_info cloudInfoWithPose;
    // disco_double::ring_cloud_info cloudInfoWithPose;
    // std_msgs::Header cloudHeader;
    // ROS 订阅者，订阅全局回环信息
    ros::Subscriber subGlobalLoop;
    // ROS 发布者，发布特征点云
    ros::Publisher pubFeatureCloud;
    // 存储最后一帧角点特征点云的指针
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastFeature; 
    // 存储最后一帧平面特征点云的指针
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastFeature; 

    // ROS 订阅者，订阅激光点云信息
    ros::Subscriber subCloud;
    // ROS 订阅者，订阅 GPS 信息
    ros::Subscriber subGPS;
    // ROS 订阅者，订阅回环检测信息
    ros::Subscriber subLoop;

    // GPS 消息队列，存储接收到的 GPS 信息
    std::deque<nav_msgs::Odometry> gpsQueue;
    // 激光点云信息，包含点云特征和相关信息
    // disco_double::cloud_info cloudInfo;
    disco_double::ring_cloud_info cloudInfo;

    // 存储角点关键帧点云的向量
    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    // 存储平面关键帧点云的向量
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
    
    // 存储关键位姿的三维点云指针
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    // 存储关键位姿的 6D 点云指针
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    // 存储关键位姿三维点云的副本指针
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    // 存储关键位姿 6D 点云的副本指针
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    // 存储来自里程计优化的角点特征点云指针
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast; 
    // 存储来自里程计优化的平面特征点云指针
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; 
    // 存储来自里程计优化的下采样角点特征点云指针
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; 
    // 存储来自里程计优化的下采样平面特征点云指针
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS; 

    // 存储待优化的点云指针
    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    // 存储优化系数的点云指针
    pcl::PointCloud<PointType>::Ptr coeffSel;

    // 用于并行计算的角点特征点存储向量
    std::vector<PointType> laserCloudOriCornerVec; 
    // 用于并行计算的角点特征优化系数存储向量
    std::vector<PointType> coeffSelCornerVec;
    // 角点特征点是否有效的标志向量
    std::vector<bool> laserCloudOriCornerFlag;
    // 用于并行计算的平面特征点存储向量
    std::vector<PointType> laserCloudOriSurfVec; 
    // 用于并行计算的平面特征优化系数存储向量
    std::vector<PointType> coeffSelSurfVec;
    // 平面特征点是否有效的标志向量
    std::vector<bool> laserCloudOriSurfFlag;

    // 存储地图点云的容器，键为关键帧索引，值为角点和平面点云的配对
    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;
    // 存储来自地图的角点特征点云指针
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    // 存储来自地图的平面特征点云指针
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    // 存储来自地图的下采样角点特征点云指针
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
    // 存储来自地图的下采样平面特征点云指针
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    // 用于地图角点特征点云的 KD 树指针
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    // 用于地图平面特征点云的 KD 树指针
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    // 用于周围关键位姿的 KD 树指针
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    // 用于历史关键位姿的 KD 树指针
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    // 角点特征点云的体素滤波器
    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    // 平面特征点云的体素滤波器
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    // 用于 ICP 匹配的点云体素滤波器
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    // 用于周围关键位姿的体素滤波器，用于扫描到地图的优化
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; 
    
    // 激光点云信息的时间戳
    ros::Time timeLaserInfoStamp;
    // 激光点云信息的当前时间（秒）
    double timeLaserInfoCur;

    // 待映射的变换参数数组，依次为 roll, pitch, yaw, x, y, z
    float transformTobeMapped[6];

    // 互斥锁，用于保护共享数据
    std::mutex mtx;
    // 用于回环信息的互斥锁
    std::mutex mtxLoopInfo;

    // 标志位，指示优化是否退化
    bool isDegenerate = false;
    // 用于处理退化情况的矩阵
    cv::Mat matP;

    // 来自地图的下采样角点特征点云数量
    int laserCloudCornerFromMapDSNum = 0;
    // 来自地图的下采样平面特征点云数量
    int laserCloudSurfFromMapDSNum = 0;
    // 最后一帧下采样角点特征点云数量
    int laserCloudCornerLastDSNum = 0;
    // 最后一帧下采样平面特征点云数量
    int laserCloudSurfLastDSNum = 0;

    // ring问题修正
    double timeStart ;

    // 标志位，指示是否有回环被闭合
    bool aLoopIsClosed = false;
    // 回环索引容器，键为新关键帧索引，值为旧关键帧索引
    map<int, int> loopIndexContainer; 
    // 回环索引队列，存储回环的关键帧索引对
    vector<pair<int, int>> loopIndexQueue;
    // 回环位姿队列，存储回环的位姿信息
    vector<gtsam::Pose3> loopPoseQueue;
    // 回环噪声队列，存储回环约束的噪声模型
    vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    // 回环信息队列，存储接收到的回环检测信息
    deque<std_msgs::Float64MultiArray> loopInfoVec;

    // 全局路径信息
    nav_msgs::Path globalPath;

    // 点关联到地图的变换矩阵
    Eigen::Affine3f transPointAssociateToMap;
    // 增量里程计变换矩阵的前一时刻值
    Eigen::Affine3f incrementalOdometryAffineFront;
    // 增量里程计变换矩阵的后一时刻值
    Eigen::Affine3f incrementalOdometryAffineBack;

    /************************************************************************************************** 
    ****************************          主线程：位姿优化器            **********************************
    **************************************************************************************************/

    /**
     * @brief 地图优化类的构造函数，用于初始化 ISAM2 优化器、ROS 发布者、订阅者以及点云滤波器，并分配内存。
     * 
     * 此构造函数会完成以下操作：
     * 1. 初始化 ISAM2 优化器的参数并创建优化器实例。
     * 2. 初始化多个 ROS 发布者，用于发布不同类型的消息，如点云、里程计、路径等。
     * 3. 初始化多个 ROS 订阅者，用于订阅不同类型的消息，如点云信息、GPS 信息、回环检测信息等。
     * 4. 初始化点云滤波器的参数。
     * 5. 调用 allocateMemory 函数分配内存。
     */
    mapOptimization()
    {
        // 定义 ISAM2 优化器的参数
        ISAM2Params parameters;
        // 设置重新线性化的阈值，当变量的更新超过该阈值时，进行重新线性化
        parameters.relinearizeThreshold = 0.1;
        // 设置重新线性化的跳过次数，每处理 1 次进行一次重新线性化
        parameters.relinearizeSkip = 1;
        // 根据设置的参数创建 ISAM2 优化器实例
        isam = new ISAM2(parameters);
        cout << "robot_id: " << robot_id << endl;
        // 初始化 ROS 发布者，用于发布关键位姿点云
        pubKeyPoses                 = nh.advertise<sensor_msgs::PointCloud2>(robot_id + "/disco_double/mapping/trajectory", 1);
        // 初始化 ROS 发布者，用于发布全局地图点云
        pubLaserCloudSurround       = nh.advertise<sensor_msgs::PointCloud2>(robot_id + "/disco_double/mapping/map_global", 1);
        // 初始化 ROS 发布者，用于发布全局激光里程计信息
        pubLaserOdometryGlobal      = nh.advertise<nav_msgs::Odometry> (robot_id + "/disco_double/mapping/odometry", 1);
        // 初始化 ROS 发布者，用于发布增量式激光里程计信息
        pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry> (robot_id + "/disco_double/mapping/odometry_incremental", 1);
        // 初始化 ROS 发布者，用于发布全局路径信息
        pubPath                     = nh.advertise<nav_msgs::Path>(robot_id + "/disco_double/mapping/path", 1);

        // 注释掉的代码，原本用于发布到融合节点的点云信息
        //for fusion node
        //for
        //        pubCloudInfoWithPose        = nh.advertise<disco_double::cloud_info> (robot_id + "/disco_double/mapping/cloud_info", 1);
        //        pubCloudInfoWithPose        = nh.advertise<disco_double::ring_cloud_info> (robot_id + "/disco_double/mapping/cloud_info", 1);
        
        // 多机器人添加
        // 初始化 ROS 发布者，用于发布全局特征点云
        pubFeatureCloud = nh.advertise<sensor_msgs::PointCloud2>(robot_id + "/disco_double/mapping/feature_cloud_global", 1);
        // 初始化 ROS 订阅者，订阅全局回环信息，并指定回调函数
        subGlobalLoop = nh.subscribe<disco_double::context_info>(robot_id + "/context/loop_info", 100, &mapOptimization::contextLoopInfoHandler, this, ros::TransportHints().tcpNoDelay());
        // 多机器人添加end

        // 初始化 ROS 订阅者，订阅激光点云信息，并指定回调函数
        // subCloud = nh.subscribe<disco_double::cloud_info>(robot_id + "/disco_double/feature/cloud_info", 1, &mapOptimization::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        subCloud = nh.subscribe<disco_double::ring_cloud_info>(robot_id + "/disco_double/feature/cloud_info", 1, &mapOptimization::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        // 初始化 ROS 订阅者，订阅 GPS 信息，并指定回调函数
        subGPS   = nh.subscribe<nav_msgs::Odometry> (gpsTopic, 200, &mapOptimization::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        // 初始化 ROS 订阅者，订阅回环检测信息，并指定回调函数
        subLoop  = nh.subscribe<std_msgs::Float64MultiArray>(robot_id + "/lio_loop/loop_closure_detection", 1, &mapOptimization::loopInfoHandler, this, ros::TransportHints().tcpNoDelay());

        // 初始化 ROS 发布者，用于发布历史关键帧点云，用于回环检测
        pubHistoryKeyFrames   = nh.advertise<sensor_msgs::PointCloud2>(robot_id + "/disco_double/mapping/icp_loop_closure_history_cloud", 1);
        // 初始化 ROS 发布者，用于发布经过 ICP 校正后的关键帧点云
        pubIcpKeyFrames       = nh.advertise<sensor_msgs::PointCloud2>(robot_id + "/disco_double/mapping/icp_loop_closure_corrected_cloud", 1);
        // 初始化 ROS 发布者，用于发布回环约束边的可视化标记
        pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>(robot_id + "/disco_double/mapping/loop_closure_constraints", 1);

        // 初始化 ROS 发布者，用于发布最近的关键帧点云
        pubRecentKeyFrames    = nh.advertise<sensor_msgs::PointCloud2>(robot_id + "/disco_double/mapping/map_local", 1);
        // 初始化 ROS 发布者，用于发布最近的单个关键帧点云
        pubRecentKeyFrame     = nh.advertise<sensor_msgs::PointCloud2>(robot_id + "/disco_double/mapping/cloud_registered", 1);
        // 初始化 ROS 发布者，用于发布原始注册后的点云
        pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>(robot_id + "/disco_double/mapping/cloud_registered_raw", 1);

        // 初始化 ROS 发布者，用于多机器人场景下发布激光点云信息(多机器人修改)
        // pubSLAMInfo           = nh.advertise<lio_sam::cloud_info>("lio_sam/mapping/slam_info", 1);
        pubLaserCloudInfo = nh.advertise<disco_double::ring_cloud_info> (robot_id + "/disco_double/mapping/cloud_info", 1);

        // 设置角点特征点云体素滤波器的叶子大小
        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        // 设置平面特征点云体素滤波器的叶子大小
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        // 设置用于 ICP 匹配的点云体素滤波器的叶子大小
        downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        // 设置用于周围关键位姿的体素滤波器的叶子大小，用于扫描到地图的优化
        downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); 

        // 调用函数分配内存
        allocateMemory();
    }

    /**
     * @brief 为地图优化类中的各种点云和 KD 树分配内存，并初始化相关变量。
     * 
     * 该函数负责为地图优化过程中使用的各种点云数据结构和 KD 树分配内存，
     * 同时对一些数组和矩阵进行初始化操作，确保后续的地图优化计算能够正常进行。
     */
    void allocateMemory()
    {
        // 为存储关键位姿的三维点云分配内存
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        // 为存储关键位姿的 6D 点云分配内存
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        // 为存储关键位姿三维点云的副本分配内存
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        // 为存储关键位姿 6D 点云的副本分配内存
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        // 为用于周围关键位姿的 KD 树分配内存
        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        // 为用于历史关键位姿的 KD 树分配内存
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        // 为来自里程计优化的角点特征点云分配内存
        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>()); 
        // 为来自里程计优化的平面特征点云分配内存
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>()); 
        // 为来自里程计优化的下采样角点特征点云分配内存
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); 
        // 为来自里程计优化的下采样平面特征点云分配内存
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>()); 

        //多机器人修改
        // 为最后一帧角点特征点云分配内存
        laserCloudCornerLastFeature.reset(new pcl::PointCloud<PointType>());
        // 为最后一帧平面特征点云分配内存
        laserCloudSurfLastFeature.reset(new pcl::PointCloud<PointType>());
        //多机器人修改end

        // 为待优化的点云分配内存
        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        // 为优化系数的点云分配内存
        coeffSel.reset(new pcl::PointCloud<PointType>());

        // 为并行计算的角点特征点存储向量分配内存
        laserCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
        // 为并行计算的角点特征优化系数存储向量分配内存
        coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
        // 为角点特征点是否有效的标志向量分配内存
        laserCloudOriCornerFlag.resize(N_SCAN * Horizon_SCAN);
        // 为并行计算的平面特征点存储向量分配内存
        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        // 为并行计算的平面特征优化系数存储向量分配内存
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        // 为平面特征点是否有效的标志向量分配内存
        laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        // 初始化角点特征点是否有效的标志向量，全部置为 false
        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        // 初始化平面特征点是否有效的标志向量，全部置为 false
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        // 为来自地图的角点特征点云分配内存
        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        // 为来自地图的平面特征点云分配内存
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        // 为来自地图的下采样角点特征点云分配内存
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        // 为来自地图的下采样平面特征点云分配内存
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        // 为用于地图角点特征点云的 KD 树分配内存
        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        // 为用于地图平面特征点云的 KD 树分配内存
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        // 初始化待映射的变换参数数组，全部置为 0
        for (int i = 0; i < 6; ++i){
            transformTobeMapped[i] = 0;
        }

        // 初始化用于处理退化情况的矩阵，全部置为 0
        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
    }

    /**
     * @brief 处理全局（机器人间）回环信息的回调函数，根据接收到的上下文信息添加回环约束并更新优化结果。
     * 
     * 该函数会检查接收到的消息中的机器人 ID 是否与当前机器人 ID 匹配，
     * 若匹配则提取回环的起始和结束索引、回环位姿和噪声信息，
     * 并将回环约束添加到 GTSAM 因子图中，然后更新 ISAM2 优化器，
     * 最后调用 `correctPoses` 和 `publishFrames` 函数进行位姿校正和帧发布。
     * 
     * @param msgIn 接收到的全局回环上下文信息的常量指针。(机器人之间添加)
     */
    void contextLoopInfoHandler(const disco_double::context_infoConstPtr& msgIn){
        // 注释掉的代码，原本用于直接返回，不处理全局回环信息
        // close global loop by do nothing
        // return;

        // 检查接收到的消息中的机器人 ID 是否与当前机器人 ID 匹配，若不匹配则直接返回
        if(msgIn->robotID != robot_id)
            return;

        // 从消息中提取回环的起始索引
        int indexFrom = msgIn->numRing;
        // 从消息中提取回环的结束索引
        int indexTo = msgIn->numSector;

        // 根据消息中的旋转和平移信息创建回环的位姿对象
        gtsam::Pose3 poseBetween = gtsam::Pose3( gtsam::Rot3::RzRyRx(msgIn->poseRoll, msgIn->posePitch, msgIn->poseYaw),
                                         gtsam::Point3(msgIn->poseX, msgIn->poseY, msgIn->poseZ) );
        // 从消息中提取噪声分数
        float noiseScore = msgIn->poseIntensity;
        // 创建一个 6 维向量，用于存储噪声信息
        gtsam::Vector Vector6(6);
        // 将噪声分数赋值给向量的每个元素
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore,
            noiseScore;
        // 根据噪声向量创建噪声模型
        auto noiseBetween = gtsam::noiseModel::Diagonal::Variances(Vector6);

        // 记录当前时间，用于后续可能的性能分析
        double start = ros::Time::now().toSec();;
        // 将回环约束添加到 GTSAM 因子图中
        gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
        // 更新 ISAM2 优化器
        isam->update(gtSAMgraph);
        // 多次更新 ISAM2 优化器，以确保优化结果收敛
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        // 计算优化后的估计值
        isamCurrentEstimate = isam->calculateEstimate();

        // 设置回环闭合标志为 true
        aLoopIsClosed = true;

        // 调用位姿校正函数，校正关键帧的位姿
        correctPoses();

        // 调用帧发布函数，发布更新后的帧信息
        publishFrames();

    }

    /**
     * @brief 处理激光点云信息的回调函数，当接收到新的激光点云信息时被调用。
     * 
     * 该函数从接收到的消息中提取时间戳、点云信息和特征点云，
     * 并在满足处理间隔条件时，依次执行初始化猜测更新、提取周围关键帧、
     * 下采样当前扫描点云、进行扫描到地图的优化、保存关键帧和因子、
     * 校正位姿、发布里程计信息和发布帧信息等操作。
     * 
     * @param msgIn 接收到的激光点云信息的常量指针。
     */
    // void laserCloudInfoHandler(const disco_double::cloud_infoConstPtr& msgIn)
    void laserCloudInfoHandler(const disco_double::ring_cloud_infoConstPtr& msgIn)
    {
        // 提取时间戳信息
        // 将消息头中的时间戳赋值给 timeLaserInfoStamp
        timeLaserInfoStamp = msgIn->header.stamp;
        // 将时间戳转换为秒并赋值给 timeLaserInfoCur
        timeLaserInfoCur = msgIn->header.stamp.toSec();

        // 提取激光点云信息和特征点云
        // 将接收到的消息内容赋值给 cloudInfo
        cloudInfo = *msgIn;
        // 将 ROS 消息中的角点特征点云转换为 PCL 点云并存储到 laserCloudCornerLast
        pcl::fromROSMsg(msgIn->cloud_corner,  *laserCloudCornerLast);
        // 将 ROS 消息中的平面特征点云转换为 PCL 点云并存储到 laserCloudSurfLast
        pcl::fromROSMsg(msgIn->cloud_surface, *laserCloudSurfLast);

        // 加锁，确保线程安全，防止多个线程同时访问共享资源
        std::lock_guard<std::mutex> lock(mtx);

        // 静态变量，记录上一次处理的时间，初始值为 -1
        static double timeLastProcessing = -1;
        // 检查是否达到地图处理间隔时间
        if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval)
        {
            // 更新上一次处理的时间
            timeLastProcessing = timeLaserInfoCur;

            // 更新初始猜测值，为后续优化提供初始位姿
            // 从imu，上一个里程计等信息中获取初始位姿估计
            // （其实可以全局位姿优化的时候再做）
            updateInitialGuess();

            // 提取周围的关键帧，用于构建局部地图
            // 上一帧的数据，用于优化当前帧的位姿
            extractSurroundingKeyFrames();

            // 对当前扫描的点云进行下采样，减少数据量
            downsampleCurrentScan();

            // 进行扫描到地图的优化，优化当前帧的位姿
            scan2MapOptimization();

            // 保存关键帧和约束因子，用于后续的图优化
            saveKeyFramesAndFactor();

            // 校正关键帧的位姿，确保位姿的准确性
            correctPoses();

            // 发布里程计信息，供其他节点使用
            publishOdometry();

            // 发布更新后的帧信息，如点云、路径等
            publishFrames();
        }
    }

    /**
     * @brief GPS 消息处理的回调函数，当接收到 GPS 消息时被调用。
     * 
     * 该函数的主要功能是将接收到的 GPS 消息添加到 `gpsQueue` 队列中，
     * 后续程序可以从该队列中获取 GPS 信息，用于地图优化等操作。
     * 
     * @param gpsMsg 接收到的 GPS 消息的常量指针，包含 GPS 的位置、姿态等信息。
     */
    void gpsHandler(const nav_msgs::Odometry::ConstPtr& gpsMsg)
    {
        // 将接收到的 GPS 消息添加到 gpsQueue 队列的尾部
        gpsQueue.push_back(*gpsMsg);
    }

    /**
     * @brief 将一个点从当前坐标系转换到地图坐标系。
     * 
     * 该函数使用 `transPointAssociateToMap` 变换矩阵，将输入点 `pi` 从当前坐标系转换到地图坐标系，
     * 并将转换后的点存储在输出点 `po` 中。强度值保持不变。
     * 
     * @param pi 指向输入点的常量指针，该点位于当前坐标系中。
     * @param po 指向输出点的指针，用于存储转换到地图坐标系后的点。
     */
    void pointAssociateToMap(PointType const * const pi, PointType * const po)
    {
        // 根据变换矩阵将输入点的 x 坐标转换到地图坐标系
        po->x = transPointAssociateToMap(0,0) * pi->x + transPointAssociateToMap(0,1) * pi->y + transPointAssociateToMap(0,2) * pi->z + transPointAssociateToMap(0,3);
        // 根据变换矩阵将输入点的 y 坐标转换到地图坐标系
        po->y = transPointAssociateToMap(1,0) * pi->x + transPointAssociateToMap(1,1) * pi->y + transPointAssociateToMap(1,2) * pi->z + transPointAssociateToMap(1,3);
        // 根据变换矩阵将输入点的 z 坐标转换到地图坐标系
        po->z = transPointAssociateToMap(2,0) * pi->x + transPointAssociateToMap(2,1) * pi->y + transPointAssociateToMap(2,2) * pi->z + transPointAssociateToMap(2,3);
        // 保持输入点的强度值不变
        po->intensity = pi->intensity;
    }

    /**
     * @brief 将输入点云根据给定的变换信息转换到新的坐标系。
     * 
     * 该函数使用 `transformIn` 中的平移和旋转信息创建一个变换矩阵，
     * 然后将输入点云 `cloudIn` 中的所有点根据该变换矩阵转换到新的坐标系，
     * 并返回转换后的点云。
     * 
     * @param cloudIn 输入点云的指针，包含需要转换的所有点。
     * @param transformIn 指向包含变换信息（平移和旋转）的点的指针。
     * @return pcl::PointCloud<PointType>::Ptr 转换后的点云的指针。
     */
    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        // 创建一个新的点云对象，用于存储转换后的点
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        // 获取输入点云的大小
        int cloudSize = cloudIn->size();
        // 调整输出点云的大小，使其与输入点云相同
        cloudOut->resize(cloudSize);

        // 根据 `transformIn` 中的平移和旋转信息创建一个仿射变换矩阵
        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
        
        // 使用 OpenMP 并行计算，加速点云转换过程
        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            // 获取输入点云中的当前点
            const auto &pointFrom = cloudIn->points[i];
            // 根据变换矩阵将当前点的 x 坐标转换到新的坐标系
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            // 根据变换矩阵将当前点的 y 坐标转换到新的坐标系
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            // 根据变换矩阵将当前点的 z 坐标转换到新的坐标系
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
            // 保持当前点的强度值不变
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

    /**
     * @brief 将 `PointTypePose` 类型的点转换为 `gtsam::Pose3` 类型的位姿。
     * 
     * 该函数从 `thisPoint` 中提取平移和旋转信息，并将其转换为 `gtsam::Pose3` 类型的位姿。
     * 
     * @param thisPoint 包含平移和旋转信息的 `PointTypePose` 类型的点。
     * @return gtsam::Pose3 转换后的 `gtsam::Pose3` 类型的位姿。
     */
    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        // 从 `thisPoint` 中提取旋转信息创建旋转矩阵，提取平移信息创建三维点，然后组合成 `gtsam::Pose3` 类型的位姿
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                                  gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
    }

    /**
     * @brief 将一个浮点数数组表示的变换信息转换为 `gtsam::Pose3` 类型的位姿。
     * 
     * 该函数从 `transformIn` 数组中提取旋转和平移信息，并将其转换为 `gtsam::Pose3` 类型的位姿。
     * 
     * @param transformIn 包含旋转和平移信息的浮点数数组，前三个元素为旋转信息，后三个元素为平移信息。
     * @return gtsam::Pose3 转换后的 `gtsam::Pose3` 类型的位姿。
     */
    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        // 从 `transformIn` 数组中提取旋转信息创建旋转矩阵，提取平移信息创建三维点，然后组合成 `gtsam::Pose3` 类型的位姿
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                                  gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    /**
     * @brief 将 `PointTypePose` 类型的点转换为 `Eigen::Affine3f` 类型的仿射变换矩阵。
     * 
     * 该函数从 `thisPoint` 中提取平移和旋转信息，并将其转换为 `Eigen::Affine3f` 类型的仿射变换矩阵。
     * 
     * @param thisPoint 包含平移和旋转信息的 `PointTypePose` 类型的点。
     * @return Eigen::Affine3f 转换后的 `Eigen::Affine3f` 类型的仿射变换矩阵。
     */
    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
    { 
        // 根据 `thisPoint` 中的平移和旋转信息创建一个 `Eigen::Affine3f` 类型的仿射变换矩阵
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    /**
     * @brief 将一个浮点数数组表示的变换信息转换为 `Eigen::Affine3f` 类型的仿射变换矩阵。
     * 
     * 该函数从 `transformIn` 数组中提取旋转和平移信息，并将其转换为 `Eigen::Affine3f` 类型的仿射变换矩阵。
     * 
     * @param transformIn 包含旋转和平移信息的浮点数数组，前三个元素为旋转信息，后三个元素为平移信息。
     * @return Eigen::Affine3f 转换后的 `Eigen::Affine3f` 类型的仿射变换矩阵。
     */
    Eigen::Affine3f trans2Affine3f(float transformIn[])
    {
        // 根据 `transformIn` 数组中的平移和旋转信息创建一个 `Eigen::Affine3f` 类型的仿射变换矩阵
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    /**
     * @brief 将一个浮点数数组表示的变换信息转换为 `PointTypePose` 类型的点。
     * 
     * 该函数从 `transformIn` 数组中提取旋转和平移信息，并将其存储在 `PointTypePose` 类型的点中。
     * 
     * @param transformIn 包含旋转和平移信息的浮点数数组，前三个元素为旋转信息，后三个元素为平移信息。
     * @return PointTypePose 转换后的 `PointTypePose` 类型的点。
     */
    PointTypePose trans2PointTypePose(float transformIn[])
    {
        // 创建一个 `PointTypePose` 类型的点，用于存储转换后的信息
        PointTypePose thisPose6D;
        // 从 `transformIn` 数组中提取平移信息存储到 `thisPose6D` 中
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        // 从 `transformIn` 数组中提取旋转信息存储到 `thisPose6D` 中
        thisPose6D.roll  = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw   = transformIn[2];
        return thisPose6D;
    }

    /************************************************************************************************** 
    ****************************          副线程：全局位姿、地图发布和可视化            *********************
    **************************************************************************************************/

    /**
     * @brief 可视化全局地图（就是保存地图）的线程函数，负责定期可视化全局地图，并在需要时将地图保存为 PCD 文件。
     * 
     * 该线程会以固定频率调用 `publishGlobalMap` 函数来可视化全局地图。当 `savePCD` 标志为 `true` 时，
     * 线程会将全局地图的相关信息保存为 PCD 文件，包括关键帧位姿、变换信息、角点特征点云、平面特征点云和全局点云地图。
     */
    // 重要函数 --- 可视化全局地图的线程函数，负责定期可视化全局地图，并在需要时将地图保存为 PCD 文件。
    void visualizeGlobalMapThread()
    {
        // 创建一个 ROS 速率对象，设置频率为 0.2Hz，即每 5 秒执行一次循环
        ros::Rate rate(0.2);
        // 当 ROS 节点正常运行时，持续执行循环
        while (ros::ok()){
            // 让线程休眠，以保证按照设定的频率执行后续操作
            rate.sleep();
            // 调用 publishGlobalMap 函数发布全局地图进行可视化
            publishGlobalMap();
        }

        // 如果 savePCD 标志为 false，则直接返回，不进行地图保存操作
        if (savePCD == false)
            return;

        // 输出提示信息，表明开始保存地图到 PCD 文件
        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files ..." << endl;
        // 将保存 PCD 文件的目录路径与用户主目录拼接
        savePCDDirectory = std::getenv("HOME") + savePCDDirectory;
        // 删除指定目录下的所有文件
        int unused = system((std::string("exec rm -r ") + savePCDDirectory).c_str());
        // 创建指定目录
        unused = system((std::string("mkdir ") + savePCDDirectory).c_str());
        // 保存关键帧位姿信息到 trajectory.pcd 文件
        pcl::io::savePCDFileASCII(savePCDDirectory + "trajectory.pcd", *cloudKeyPoses3D);
        // 保存关键帧变换信息到 transformations.pcd 文件
        pcl::io::savePCDFileASCII(savePCDDirectory + "transformations.pcd", *cloudKeyPoses6D);
        // 定义用于存储全局角点特征点云的指针
        pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
        // 定义用于存储下采样后的全局角点特征点云的指针
        pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(new pcl::PointCloud<PointType>());
        // 定义用于存储全局平面特征点云的指针
        pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
        // 定义用于存储下采样后的全局平面特征点云的指针
        pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
        // 定义用于存储全局点云地图的指针
        pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
        // 遍历所有关键帧，将角点和平面特征点云进行变换并累加到全局点云中
        for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++) {
            *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i],  &cloudKeyPoses6D->points[i]);
            *globalSurfCloud   += *transformPointCloud(surfCloudKeyFrames[i],    &cloudKeyPoses6D->points[i]);
            // 输出处理进度信息
            cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
        }
        // 对全局角点特征点云进行下采样
        downSizeFilterCorner.setInputCloud(globalCornerCloud);
        downSizeFilterCorner.filter(*globalCornerCloudDS);
        // 将下采样后的全局角点特征点云保存到 cloudCorner.pcd 文件
        pcl::io::savePCDFileASCII(savePCDDirectory + "cloudCorner.pcd", *globalCornerCloudDS);
        // 对全局平面特征点云进行下采样
        downSizeFilterSurf.setInputCloud(globalSurfCloud);
        downSizeFilterSurf.filter(*globalSurfCloudDS);
        // 将下采样后的全局平面特征点云保存到 cloudSurf.pcd 文件
        pcl::io::savePCDFileASCII(savePCDDirectory + "cloudSurf.pcd", *globalSurfCloudDS);
        // 将全局角点和平面特征点云合并到全局点云地图中
        *globalMapCloud += *globalCornerCloud;
        *globalMapCloud += *globalSurfCloud;
        // 将全局点云地图保存到 cloudGlobal.pcd 文件
        pcl::io::savePCDFileASCII(savePCDDirectory + "cloudGlobal.pcd", *globalMapCloud);
        // 输出提示信息，表明地图保存完成
        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files completed" << endl;
    }

    /**
     * @brief 发布全局地图点云用于可视化。
     * 
     * 该函数会检查是否有订阅者订阅全局地图点云，若有则从关键位姿中筛选出
     * 距离当前位姿一定范围内的关键帧，对这些关键帧进行下采样，然后将对应的
     * 角点和平面特征点云进行变换并合并，最后再次下采样并发布全局地图点云。
     */
    void publishGlobalMap()
    {
        // 检查是否有订阅者订阅全局地图点云，如果没有则直接返回，不进行后续操作
        if (pubLaserCloudSurround.getNumSubscribers() == 0)
            return;

        // 检查是否存在关键位姿点云，如果不存在则直接返回，不进行后续操作
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        // 创建用于全局地图的 KD 树，用于查找附近的关键帧
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());;
        // 存储筛选出的全局地图关键位姿点云
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        // 存储下采样后的全局地图关键位姿点云
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
        // 存储筛选出的全局地图关键帧点云
        pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
        // 存储下采样后的全局地图关键帧点云
        pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

        // 用于存储 KD 树搜索结果的索引和平方距离
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;

        // 搜索距离当前关键位姿一定范围内的关键帧
        // 加锁，确保线程安全，防止在搜索过程中关键位姿点云被修改
        mtx.lock();
        // 设置 KD 树的输入点云为全局关键位姿点云
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        /**
         * @brief 使用 KD 树在全局关键位姿点云中进行半径搜索，查找距离当前关键位姿一定范围内的所有关键位姿。
         * 
         * 该函数会在以当前关键位姿为中心，以 `globalMapVisualizationSearchRadius` 为半径的球形区域内，
         * 搜索全局关键位姿点云中的所有关键位姿。搜索结果会存储在 `pointSearchIndGlobalMap` 和 `pointSearchSqDisGlobalMap` 中。
         * 
         * @param cloudKeyPoses3D->back() 作为搜索中心的当前关键位姿点，即全局关键位姿点云的最后一个点。
         * @param globalMapVisualizationSearchRadius 搜索半径，用于定义搜索的球形区域大小。
         * @param pointSearchIndGlobalMap 存储搜索到的关键位姿点在全局关键位姿点云中的索引。
         * @param pointSearchSqDisGlobalMap 存储搜索到的关键位姿点与搜索中心的平方距离。
         * @param 0 表示不限制搜索到的点的数量，即返回所有满足条件的点。
         */
        // 调用 KD 树的半径搜索函数，在全局关键位姿点云中查找距离当前关键位姿一定范围内的所有关键位姿
        // 以当前关键位姿为中心，在指定半径内搜索附近的关键帧
        kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        // 解锁，允许其他线程访问关键位姿点云
        mtx.unlock();

        // 将搜索到的关键位姿添加到全局地图关键位姿点云中
        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);

        // 创建体素滤波器，用于对全局地图关键位姿点云进行下采样
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
        // 设置体素滤波器的叶子大小（x降采样,y降采样,z降采样）
        downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
        // 设置体素滤波器的输入点云为全局地图关键位姿点云
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        // 执行下采样操作，结果存储在 globalMapKeyPosesDS 中
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);

        for(auto& pt : globalMapKeyPosesDS->points)
        {
            kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
            pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
        }

        // 提取可视化并下采样后的关键帧
        for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i){
            // 检查当前关键位姿是否在指定搜索半径内，若不在则跳过
            if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
                continue;
            // 获取当前关键帧的索引
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            // 将当前关键帧的角点特征点云进行变换并添加到全局地图关键帧点云中
            *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
            // 将当前关键帧的平面特征点云进行变换并添加到全局地图关键帧点云中
            *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
        }

        // 创建体素滤波器，用于对全局地图关键帧点云进行下采样
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
        // 设置体素滤波器的叶子大小
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
        // 设置体素滤波器的输入点云为全局地图关键帧点云
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        // 执行下采样操作，结果存储在 globalMapKeyFramesDS 中
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);

        // 发布下采样后的全局地图关键帧点云
        publishCloud(&pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, robot_id + "/" + odometryFrame);
    }

    /************************************************************************************************** 
    *********************************          副线程：回环检测模块            ***************************
    **************************************************************************************************/

    /**
     * @brief 回环检测线程函数，用于定期执行回环检测和可视化操作。
     * 
     * 该线程会在回环检测功能启用的情况下运行，按照设定的频率执行回环检测操作，并将回环检测的结果进行可视化展示。
     * 当 ROS 节点正常运行时，线程会持续工作，直到节点关闭。
     */
    // 重要函数 --- 回环检测线程函数
    void loopClosureThread()
    {
        // 检查回环检测功能是否启用，如果未启用则直接返回，不执行后续操作
        if (loopClosureEnableFlag == false)
            return;

        // 创建一个 ROS 速率对象，用于控制回环检测操作的执行频率
        ros::Rate rate(loopClosureFrequency);
        // 当 ROS 节点正常运行时，持续执行回环检测和可视化操作
        while (ros::ok())
        {
            // 让线程休眠，以保证按照设定的频率执行后续操作
            rate.sleep();
            // 调用 performLoopClosure 函数执行回环检测操作
            performLoopClosure();
            // 调用 visualizeLoopClosure 函数将回环检测的结果进行可视化展示
            visualizeLoopClosure();
        }
    }

    /**
     * @brief 处理回环检测信息的回调函数，当接收到回环检测消息时被调用。
     * 
     * 该函数会将接收到的回环检测消息添加到 `loopInfoVec` 队列中，
     * 同时确保队列的长度不超过 5 条，若超过则移除最早的消息。
     * 
     * @param loopMsg 接收到的回环检测消息的常量指针，消息类型为 `std_msgs::Float64MultiArray`。
     */
    void loopInfoHandler(const std_msgs::Float64MultiArray::ConstPtr& loopMsg)
    {
        // 加锁，确保线程安全，防止多个线程同时访问 loopInfoVec 队列
        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        // 检查接收到的消息数据长度是否为 2，若不是则直接返回，不处理该消息
        if (loopMsg->data.size() != 2)
            return;

        // 将接收到的回环检测消息添加到 loopInfoVec 队列的尾部
        loopInfoVec.push_back(*loopMsg);

        // 确保 loopInfoVec 队列的长度不超过 5 条，若超过则移除最早的消息
        while (loopInfoVec.size() > 5)
            loopInfoVec.pop_front();
    }

    /**
     * @brief 执行回环检测和位姿优化的主要函数
     */
    void performLoopClosure()
    {
        // 检查是否有关键帧位姿数据，如果没有则直接返回
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        // 使用互斥锁保护关键帧位姿数据的复制操作
        mtx.lock();
        // 复制3D关键帧位姿数据到副本中
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        // 复制6D关键帧位姿数据到副本中
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mtx.unlock();

        // 声明用于存储回环检测结果的变量
        int loopKeyCur;  // 当前关键帧索引
        int loopKeyPre;  // 历史关键帧索引
        // 首先尝试使用外部回环检测方法
        // 如果外部检测失败，则使用基于距离的回环检测方法
        // 如果两种方法都失败，则返回
        if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false)
            if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
                return;

        // 创建用于存储回环帧点云的智能指针
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            // 提取当前关键帧的点云数据
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
            // 提取历史关键帧及其周围帧的点云数据
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);
            // 检查点云数据是否足够，如果点云点数太少则返回
            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
            // 如果有订阅者，则发布历史关键帧点云用于可视化
            if (pubHistoryKeyFrames.getNumSubscribers() != 0)
                publishCloud(&pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, robot_id + "/" + odometryFrame);
        }

        // 创建并配置ICP算法对象
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        // 设置最大对应点距离阈值
        icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius*2);
        // 设置最大迭代次数
        icp.setMaximumIterations(100);
        // 设置变换矩阵元素收敛阈值
        icp.setTransformationEpsilon(1e-6);
        // 设置两次变换间隔的欧氏距离收敛阈值
        icp.setEuclideanFitnessEpsilon(1e-6);
        // 禁用RANSAC迭代
        icp.setRANSACIterations(0);

        // 设置ICP算法的输入源点云和目标点云
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        // 创建结果点云（实际未使用）
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        // 执行点云配准
        icp.align(*unused_result);

        // 检查ICP配准结果
        // 如果未收敛或匹配分数过高则认为配准失败，直接返回
        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
            return;

        // 如果有订阅者，发布经过ICP配准后的点云结果
        if (pubIcpKeyFrames.getNumSubscribers() != 0)
        {
            // 创建用于存储配准后点云的智能指针
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            // 使用ICP得到的变换矩阵对当前帧点云进行变换
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
            // 发布变换后的点云
            publishCloud(&pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, robot_id + "/" + odometryFrame);
        }

        // 声明用于存储位姿变换参数的变量
        float x, y, z, roll, pitch, yaw;
        // 获取ICP计算得到的变换矩阵
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();
        // 获取当前帧原始（未校正）的位姿变换矩阵
        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // 计算校正后的位姿变换矩阵
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
        // 从变换矩阵中提取平移和欧拉角
        pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
        // 创建GTSAM位姿对象，用于位姿图优化
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
        // 创建噪声模型向量
        gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore();
        // 使用ICP的匹配分数作为噪声值
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        // 创建对角噪声模型
        noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

        // 加锁保护回环信息的添加操作
        mtx.lock();
        // 将回环帧对的索引添加到队列
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        // 将位姿约束添加到队列
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        // 将噪声模型添加到队列
        loopNoiseQueue.push_back(constraintNoise);
        mtx.unlock();

        // 在回环索引容器中记录当前检测到的回环关系
        loopIndexContainer[loopKeyCur] = loopKeyPre;
    }

    /**
     * @brief 基于距离的回环检测函数
     * 
     * 该函数通过检查当前关键帧与历史关键帧之间的距离来检测可能的回环。
     * 主要步骤包括:
     * 1. 获取最新关键帧的索引
     * 2. 检查该关键帧是否已经添加过回环约束
     * 3. 使用KD树搜索历史关键帧中距离当前关键帧较近的帧
     * 4. 在搜索结果中寻找满足时间差条件的最早关键帧作为回环候选帧
     * 
     * @param latestID 输出参数,存储当前关键帧的索引
     * @param closestID 输出参数,存储找到的回环候选帧的索引
     * @return 如果找到合适的回环候选帧返回true,否则返回false
     */
    bool detectLoopClosureDistance(int *latestID, int *closestID)
    {
        // 获取当前关键帧的索引(最新关键帧)
        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
        // 初始化历史关键帧索引为-1
        int loopKeyPre = -1;

        // 检查当前关键帧是否已经添加过回环约束
        auto it = loopIndexContainer.find(loopKeyCur);
        // 如果已经添加过回环约束,则返回false
        if (it != loopIndexContainer.end())
            return false;

        // 创建用于存储KD树搜索结果的容器
        std::vector<int> pointSearchIndLoop;      // 存储搜索到的关键帧索引
        std::vector<float> pointSearchSqDisLoop;  // 存储对应的平方距离
        
        // 设置KD树的输入点云为历史关键帧位姿
        kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
        // 在历史关键帧中搜索距离当前关键帧小于给定半径的帧
        kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
        
        // 遍历搜索结果,寻找满足时间差条件的最早关键帧
        for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
        {
            int id = pointSearchIndLoop[i];
            // 如果找到的关键帧与当前帧的时间差大于阈值,则选择该帧作为回环候选帧
            if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) > historyKeyframeSearchTimeDiff)
            {
                loopKeyPre = id;
                break;
            }
        }

        // 如果没有找到合适的回环候选帧,或者回环候选帧就是当前帧,则返回false
        if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
            return false;

        // 将找到的回环帧对的索引存储到输出参数中
        *latestID = loopKeyCur;  // 当前关键帧索引
        *closestID = loopKeyPre; // 回环候选帧索引

        // 找到合适的回环候选帧,返回true
        return true;
    }

    /**
    * @brief 基于外部回环检测信息的回环检测函数
    * 
    * 该函数通过处理外部回环检测模块提供的时间戳信息来检测回环。主要步骤包括:
    * 1. 从回环信息队列中获取当前帧和历史帧的时间戳
    * 2. 在关键帧位姿序列中查找对应时间戳的关键帧索引
    * 3. 检查找到的关键帧对是否满足回环条件
    * 
    * @param latestID 输出参数,存储当前关键帧的索引
    * @param closestID 输出参数,存储找到的回环候选帧的索引
    * @return 如果找到合适的回环候选帧返回true,否则返回false
    */
    bool detectLoopClosureExternal(int *latestID, int *closestID)
    {
        // 初始化当前帧和历史帧的索引为-1
        int loopKeyCur = -1;
        int loopKeyPre = -1;

        // 使用互斥锁保护对回环信息队列的访问
        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        // 如果回环信息队列为空,则返回false
        if (loopInfoVec.empty())
            return false;

        // 获取队列头部的回环信息中的时间戳
        double loopTimeCur = loopInfoVec.front().data[0];  // 当前帧时间戳
        double loopTimePre = loopInfoVec.front().data[1];  // 历史帧时间戳
        // 处理完后将该回环信息从队列中移除
        loopInfoVec.pop_front();

        // 如果当前帧和历史帧的时间差小于设定阈值,则返回false
        if (abs(loopTimeCur - loopTimePre) < historyKeyframeSearchTimeDiff)
            return false;

        // 获取关键帧位姿序列的大小
        int cloudSize = copy_cloudKeyPoses6D->size();
        // 如果关键帧数量少于2,则返回false
        if (cloudSize < 2)
            return false;

        // 在关键帧位姿序列中查找当前帧对应的关键帧索引
        loopKeyCur = cloudSize - 1;
        for (int i = cloudSize - 1; i >= 0; --i)
        {
            if (copy_cloudKeyPoses6D->points[i].time >= loopTimeCur)
                loopKeyCur = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        // 在关键帧位姿序列中查找历史帧对应的关键帧索引
        loopKeyPre = 0;
        for (int i = 0; i < cloudSize; ++i)
        {
            if (copy_cloudKeyPoses6D->points[i].time <= loopTimePre)
                loopKeyPre = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        // 如果当前帧和历史帧是同一帧,则返回false
        if (loopKeyCur == loopKeyPre)
            return false;

        // 检查当前帧是否已经添加过回环约束,如果是则返回false
        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;

        // 将找到的回环帧对的索引存储到输出参数中
        *latestID = loopKeyCur;  // 当前关键帧索引
        *closestID = loopKeyPre; // 回环候选帧索引

        // 找到合适的回环候选帧,返回true
        return true;
    }

    /**
     * @brief 查找并提取指定关键帧附近的关键帧点云
     * 
     * @param nearKeyframes 用于存储提取的附近关键帧点云的指针
     * @param key 当前关键帧的索引
     * @param searchNum 搜索范围，表示在当前关键帧前后各搜索多少帧
     */
    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum)
    {
        // 清空用于存储附近关键帧点云的容器
        nearKeyframes->clear();
        // 获取关键帧位姿点云的大小
        int cloudSize = copy_cloudKeyPoses6D->size();
        // 在指定范围内遍历关键帧
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            // 计算待处理的关键帧索引
            int keyNear = key + i;
            // 如果索引超出有效范围则跳过
            if (keyNear < 0 || keyNear >= cloudSize )
                continue;
            // 将对应关键帧的角点和平面点云进行坐标变换后添加到结果点云中
            *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
            *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear],   &copy_cloudKeyPoses6D->points[keyNear]);
        }

        // 如果结果点云为空则直接返回
        if (nearKeyframes->empty())
            return;

        // 对提取的点云进行降采样处理
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }

    /**
     * @brief 可视化回环检测结果
     * 
     * 该函数将回环检测的结果以可视化标记的形式发布，包括：
     * 1. 回环节点：用球体表示
     * 2. 回环边：用线段表示连接配对的回环帧
     */
    void visualizeLoopClosure()
    {
        // 如果没有回环检测结果则直接返回
        if (loopIndexContainer.empty())
            return;
        
        // 创建标记数组用于存储可视化信息
        visualization_msgs::MarkerArray markerArray;
        
        // 配置回环节点的可视化标记
        visualization_msgs::Marker markerNode;
        markerNode.header.frame_id = robot_id + "/" + odometryFrame;  // 设置坐标系
        markerNode.header.stamp = timeLaserInfoStamp;  // 设置时间戳
        markerNode.action = visualization_msgs::Marker::ADD;  // 设置动作类型为添加
        markerNode.type = visualization_msgs::Marker::SPHERE_LIST;  // 设置标记类型为球体列表
        markerNode.ns = "loop_nodes";  // 设置命名空间
        markerNode.id = 0;  // 设置标记ID
        markerNode.pose.orientation.w = 1;  // 设置方向
        markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3;  // 设置球体大小
        markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;  // 设置颜色为青色
        markerNode.color.a = 1;  // 设置不透明度
        
        // 配置回环边的可视化标记
        visualization_msgs::Marker markerEdge;
        markerEdge.header.frame_id = robot_id + "/" + odometryFrame;  // 设置坐标系
        markerEdge.header.stamp = timeLaserInfoStamp;  // 设置时间戳
        markerEdge.action = visualization_msgs::Marker::ADD;  // 设置动作类型为添加
        markerEdge.type = visualization_msgs::Marker::LINE_LIST;  // 设置标记类型为线段列表
        markerEdge.ns = "loop_edges";  // 设置命名空间
        markerEdge.id = 1;  // 设置标记ID
        markerEdge.pose.orientation.w = 1;  // 设置方向
        markerEdge.scale.x = 0.1;  // 设置线段宽度
        markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;  // 设置颜色为黄色
        markerEdge.color.a = 1;  // 设置不透明度

        // 遍历所有回环检测结果，添加可视化标记
        for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
        {
            // 获取回环帧对的索引
            int key_cur = it->first;  // 当前帧索引
            int key_pre = it->second;  // 历史帧索引
            
            // 创建并设置点的坐标
            geometry_msgs::Point p;
            // 设置当前帧的位置
            p.x = copy_cloudKeyPoses6D->points[key_cur].x;
            p.y = copy_cloudKeyPoses6D->points[key_cur].y;
            p.z = copy_cloudKeyPoses6D->points[key_cur].z;
            // 将当前帧位置添加到节点和边的标记中
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            
            // 设置历史帧的位置
            p.x = copy_cloudKeyPoses6D->points[key_pre].x;
            p.y = copy_cloudKeyPoses6D->points[key_pre].y;
            p.z = copy_cloudKeyPoses6D->points[key_pre].z;
            // 将历史帧位置添加到节点和边的标记中
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }

        // 将节点和边的标记添加到标记数组中
        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);
        // 发布标记数组用于可视化
        pubLoopConstraintEdge.publish(markerArray);
    }

    /************************************************************************************************** 
    *******************************          主线程：全局位姿优化、全局地图            *********************
    **************************************************************************************************/

    /**
     * @brief 更新初始猜测值，为后续的地图优化提供初始位姿。
     * 
     * 在进行任何处理之前保存当前的变换信息，根据不同的传感器数据（IMU、里程计）
     * 来更新初始猜测的位姿信息。如果没有关键位姿点云，则使用 IMU 数据初始化位姿；
     * 如果有里程计数据，则使用里程计的预积分估计来更新位姿；
     * 如果只有 IMU 数据，则使用 IMU 的增量估计来更新位姿的旋转部分。
     */
    void updateInitialGuess()
    {
        // 在进行任何处理之前保存当前的变换信息
        // 从自建类型（R,Y,P,x,y,z）转换为Eigen类型
        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

        static Eigen::Affine3f lastImuTransformation;
        // 初始化
        if (cloudKeyPoses3D->points.empty())
        {
            // 使用 IMU 数据初始化位姿的旋转部分
            transformTobeMapped[0] = cloudInfo.imuRollInit;
            transformTobeMapped[1] = cloudInfo.imuPitchInit;
            transformTobeMapped[2] = cloudInfo.imuYawInit;

            // 如果不使用 IMU 航向初始化，则将偏航角设为 0
            if (!useImuHeadingInitialization)
                transformTobeMapped[2] = 0;

            // 保存 IMU 变换信息
            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); 
            return;
        }

        // 使用 IMU 预积分估计进行位姿猜测
        static bool lastImuPreTransAvailable = false;
        static Eigen::Affine3f lastImuPreTransformation;
        if (cloudInfo.odomAvailable == true)
        {
            // 根据里程计的初始猜测值创建变换矩阵
            Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initialGuessX,    cloudInfo.initialGuessY,     cloudInfo.initialGuessZ, 
                                                               cloudInfo.initialGuessRoll, cloudInfo.initialGuessPitch, cloudInfo.initialGuessYaw);
            if (lastImuPreTransAvailable == false)
            {
                // 保存初始的 IMU 预积分变换矩阵
                lastImuPreTransformation = transBack;
                lastImuPreTransAvailable = true;
            } else {
                // 计算增量变换矩阵（其实是IMU坐标系旋转至点云坐标系的旋转矩阵）
                // 注意：求逆
                Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
                // 获取当前待映射的变换矩阵（其实是点云坐标系转换到全局坐标系的旋转矩阵）
                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
                // 计算最终的变换矩阵（IMU转全局坐标 = 点云转全局 * IMU转点云）
                Eigen::Affine3f transFinal = transTobe * transIncre;
                // 从最终的变换矩阵中提取平移和旋转信息
                // 将IMU增量作为位姿估计的初始值，不过要转到全局坐标系
                // 需要准备上一个循环中的位姿（雷达转全局）、IMU预积分（初始猜测值）、
                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

                // 更新上一次的 IMU 预积分变换矩阵
                lastImuPreTransformation = transBack;

                // 保存 IMU 变换信息
                lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); 
                return;
            }
        }

        // 使用 IMU 增量估计进行位姿猜测（仅旋转部分）
        if (cloudInfo.imuAvailable == true)
        {
            // 根据 IMU 数据创建变换矩阵
            Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
            // 计算增量变换矩阵
            Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

            // 获取当前待映射的变换矩阵
            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            // 计算最终的变换矩阵
            Eigen::Affine3f transFinal = transTobe * transIncre;
            // 从最终的变换矩阵中提取平移和旋转信息
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            // 保存 IMU 变换信息
            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); 
            return;
        }
    }

    /**
     * @brief 提取用于回环检测的关键帧点云
     * 
     * 该函数从最新的关键帧开始，提取指定数量的关键帧位姿，用于后续的回环检测。
     * 主要步骤包括:
     * 1. 创建一个点云对象用于存储提取的关键帧位姿
     * 2. 从最新的关键帧开始，向前提取指定数量的关键帧位姿
     * 3. 调用 extractCloud 函数处理提取的关键帧
     */
    void extractForLoopClosure()
    {
        // 创建一个点云指针，用于存储待提取的关键帧位姿
        pcl::PointCloud<PointType>::Ptr cloudToExtract(new pcl::PointCloud<PointType>());
        // 获取当前关键帧位姿的总数
        int numPoses = cloudKeyPoses3D->size();
        // 从最新的关键帧开始，向前遍历所有关键帧位姿
        for (int i = numPoses-1; i >= 0; --i)
        {
            // 如果已提取的关键帧数量小于等于设定的周围关键帧数量阈值
            if ((int)cloudToExtract->size() <= surroundingKeyframeSize)
                // 将当前关键帧位姿添加到待提取点云中
                cloudToExtract->push_back(cloudKeyPoses3D->points[i]);
            else
                // 如果已达到指定数量，则退出循环
                break;
        }

        // 调用 extractCloud 函数处理提取的关键帧位姿
        extractCloud(cloudToExtract);
    }

    void extractNearby()
    {
        // 创建用于存储周围关键帧位姿的点云指针
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        // 创建用于存储下采样后的周围关键帧位姿的点云指针
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
        // 创建用于存储KD树搜索结果的容器
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // extract all the nearby key poses and downsample them
        // 设置KD树的输入点云为所有关键帧位姿
        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
        // 在KD树中搜索距离最新关键帧位姿指定半径范围内的关键帧
        kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
        // 将搜索到的关键帧位姿添加到surroundingKeyPoses中
        for (int i = 0; i < (int)pointSearchInd.size(); ++i)
        {
            int id = pointSearchInd[i];
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
        }

        // 对周围关键帧位姿进行下采样
        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);

        // 对下采样后的周围关键帧位姿进行再次搜索，确保每个关键帧都有最近的匹配
        for(auto& pt : surroundingKeyPosesDS->points)
        {
            kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
            pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
        }

        // also extract some latest key frames in case the robot rotates in one position
        // 获取关键帧位姿的总数
        int numPoses = cloudKeyPoses3D->size();
        // 从最新的关键帧开始向前遍历，提取最近10秒内的关键帧
        for (int i = numPoses-1; i >= 0; --i)
        {
            // 如果关键帧的时间戳与当前时间的差值小于10秒，则添加该关键帧位姿
            if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
                surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        // 调用extractCloud函数处理提取的关键帧位姿
        extractCloud(surroundingKeyPosesDS);
    }

    /**
     * @brief 从指定的关键帧集合中提取并融合局部地图点云
     * 
     * 该函数负责从给定的关键帧集合中提取角点和平面特征点云，并将它们转换到全局坐标系下，
     * 形成局部地图。主要步骤包括:
     * 1. 清空并重建局部地图
     * 2. 提取并转换特征点云
     * 3. 对特征点云进行降采样
     * 4. 管理点云缓存
     * 
     * @param cloudToExtract 包含待处理关键帧位姿的点云指针
     */
    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
    {
        // 清空用于存储局部地图的角点和平面特征点云
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear(); 

        // 遍历所有待处理的关键帧
        for (int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            // 如果当前关键帧与最新关键帧之间的距离超过设定阈值，则跳过该帧
            if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
                continue;

            // 获取当前关键帧的索引（存储在点云的intensity字段中）
            int thisKeyInd = (int)cloudToExtract->points[i].intensity;

            // 检查该关键帧的特征点云是否已经被转换并缓存
            if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end()) 
            {
                // 如果已经缓存，直接从缓存中获取转换后的特征点云
                *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;   // 添加角点特征
                *laserCloudSurfFromMap   += laserCloudMapContainer[thisKeyInd].second;  // 添加平面特征
            } else {
                // 如果未缓存，需要进行坐标转换
                // 转换角点特征点云到全局坐标系
                pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
                // 转换平面特征点云到全局坐标系
                pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
                // 将转换后的特征点云添加到局部地图中
                *laserCloudCornerFromMap += laserCloudCornerTemp;
                *laserCloudSurfFromMap   += laserCloudSurfTemp;
                // 将转换后的特征点云缓存起来，避免重复计算
                laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
        }

        // 对局部地图中的角点特征进行降采样
        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        // 记录降采样后的角点特征数量
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();

        // 对局部地图中的平面特征进行降采样
        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        // 记录降采样后的平面特征数量
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

        // 如果缓存的特征点云数量过多（超过1000），清空缓存以防止内存占用过大
        if (laserCloudMapContainer.size() > 1000)
            laserCloudMapContainer.clear();
    }

    void extractSurroundingKeyFrames()
    {
        // 如果关键帧位姿点云为空,则直接返回
        if (cloudKeyPoses3D->points.empty() == true)
            return; 
        
        // 注释掉的代码用于根据是否启用回环检测来选择不同的特征提取方式
        // if (loopClosureEnableFlag == true)
        // {
        //     extractForLoopClosure();    
        // } else {
        //     extractNearby();
        // }

        // 直接调用extractNearby()函数提取周围关键帧
        extractNearby();
    }

    /**
     * @brief 对当前扫描的点云进行下采样处理
     * 
     * 该函数分别对当前帧的角点和平面点云进行下采样处理，主要步骤包括:
     * 1. 清空下采样后的点云容器
     * 2. 使用体素滤波器进行下采样
     * 3. 保存原始特征点云的副本
     * 4. 记录下采样后的点云大小
     */
    void downsampleCurrentScan()
    {
        // 处理角点特征
        laserCloudCornerLastDS->clear();      // 清空下采样后的角点点云容器
        laserCloudCornerLastFeature->clear(); // 清空角点特征点云容器（多）
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);  // 设置体素滤波器的输入为原始角点点云
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);      // 执行下采样操作
        pcl::copyPointCloud(*laserCloudCornerLast,  *laserCloudCornerLastFeature);  // 保存原始角点点云的副本（多）
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();  // 记录下采样后的角点点云大小

        // 处理平面点特征
        laserCloudSurfLastDS->clear();      // 清空下采样后的平面点点云容器
        laserCloudSurfLastFeature->clear(); // 清空平面点特征点云容器（多）
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);  // 设置体素滤波器的输入为原始平面点点云
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);      // 执行下采样操作
        pcl::copyPointCloud(*laserCloudSurfLast,  *laserCloudSurfLastFeature);  // 保存原始平面点点云的副本（多）
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();  // 记录下采样后的平面点点云大小
    }

    /**
     * @brief 更新点云到地图坐标系的变换矩阵
     * 
     * 该函数用于更新将当前帧点云变换到地图坐标系下所需的变换矩阵。
     * 具体功能：
     * 1. 将当前估计的位姿变换(transformTobeMapped)转换为齐次变换矩阵(Affine3f)
     * 2. 存储在transPointAssociateToMap中供后续使用
     * 
     * @note transformTobeMapped是一个包含6个元素的数组:
     *       - transformTobeMapped[0,1,2]: 分别表示roll, pitch, yaw角度
     *       - transformTobeMapped[3,4,5]: 分别表示x, y, z平移
     * 
     * @note trans2Affine3f函数将位姿参数转换为4x4的齐次变换矩阵
     * 
     * @note 这个变换矩阵主要用在特征匹配过程中，用于将当前帧的点云变换到地图坐标系下
     */
    void updatePointAssociateToMap()
    {
        transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
    }

    /**
     * @brief 角点特征的优化函数
     * 
     * 该函数对当前帧中的角点特征进行优化，主要步骤包括:
     * 1. 更新点云到地图的变换矩阵
     * 2. 对每个角点特征进行处理
     * 3. 在地图中搜索最近邻点
     * 4. 计算特征点的协方差矩阵
     * 5. 进行特征匹配和优化
     */
    void cornerOptimization()
    {
        // 更新当前帧点云到地图坐标系的变换矩阵
        updatePointAssociateToMap();

        // 使用OpenMP进行并行计算，加速处理过程
        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudCornerLastDSNum; i++)
        {
            // 定义点云数据结构和搜索结果存储容器
            PointType pointOri, pointSel, coeff;  // 原始点、转换后的点、系数
            std::vector<int> pointSearchInd;      // 搜索结果点的索引
            std::vector<float> pointSearchSqDis;  // 搜索结果点的距离平方

            // 获取当前处理的角点
            // （feature提取角点-->feature降采样-->optmization降采样-->laserCloudCornerLastDS）
            pointOri = laserCloudCornerLastDS->points[i];
            // 将角点转换到地图坐标系
            pointAssociateToMap(&pointOri, &pointSel);
            // 在地图中搜索距离该点最近的5个点（没有特征要求）
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            // 创建用于特征提取的矩阵
            cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));  // 协方差矩阵
            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));  // 特征值
            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));  // 特征向量
                    
            // 如果最远点的距离小于1.0，则进行特征提取
            // 进行平面，直线的判定，选取附近5个点（历史点）做直线，求协方差矩阵的特征值，就是直线的方向
            // 
            if (pointSearchSqDis[4] < 1.0) {
                // 计算5个最近邻点的质心
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++) {
                    cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                }
                cx /= 5; cy /= 5;  cz /= 5;  // 计算平均值得到质心坐标

                // 计算协方差矩阵的元素
                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++) {
                    // 计算点到质心的距离
                    float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                    float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                    float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                    // 计算协方差矩阵的上三角元素（对称矩阵）
                    a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
                                    a22 += ay * ay; a23 += ay * az;
                                                    a33 += az * az;
                }
                // 计算协方差矩阵元素的平均值
                a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

                // 将计算得到的协方差矩阵元素填入matA1
                matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;

                // 对协方差矩阵进行特征值分解
                // 对协方差矩阵做特征值分解，最大特征值对应的特征向量是这5个点的主方向
                cv::eigen(matA1, matD1, matV1);

                // 如果最大特征值显著大于第二大特征值，说明构成了一个良好的线特征
                // 以下部分是在计算当前点pointSel到检索出的直线的距离和方向，如果距离够近，则认为匹配成功，否则认为匹配失败
                // x0,y0,z0是直线外一点
                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {
                    // 获取当前点的坐标
                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;
                    // 根据特征向量构建线特征的两个端点
                    // matV1的第一行就是5个点形成的直线的方向，cx,cy,cz是5个点的中心点
                    // 因此，x1,y1,z1和x2,y2,z2是经过中心点的直线上的另外两个点（不是点云的点），两点之间的距离是0.2米
                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                    // 计算点到直线的距离相关参数
                    // a012是三个点构成的平行四边形的面积的两倍
                    // 这边是在求[(x0-x1),(y0-y1),(z0-z1)]与[(x0-x2),(y0-y2),(z0-z2)]叉乘得到的向量的模长
                    // 这个模长是由0.2*V1[0]和点[x0,y0,z0]构成的平行四边形的面积
                    // 垂直于0,1,2三点构成的平面的向量
                    // [XXX,YYY,ZZZ] = [(y0-y1)(z0-z2)-(y0-y2)(z0-z1),
                                        // -(x0-x1)(z0-z2)+(x0-x2)(z0-z1),
                                        // (x0-x1)(y0-y2)-(x0-x2)(y0-y1)]
                    float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                                    + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                                    + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)) * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                    // 计算线特征两个端点之间的距离
                    // l12表示的是0.2*(||V1[0]||)
                    // 点x1,y1,z1到点x2,y2,z2的距离
                    float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                    // 计算点到直线的投影系数
                    // 求叉乘结果[la',lb',lc']=[(x1-x2),(y1-y2),(z1-z2)]x[XXX,YYY,ZZZ]
                    // [la,lb,lc]=[la',lb',lc']/a012/l12
                    // LLL=[la,lb,lc]是0.2*V1[0]这条高上的单位法向量。||LLL||=1；
                    float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                              + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                    float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                               - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                               + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    // 计算点到直线的距离
                    // ld2就是点pointSel(x0,y0,z0)到直线的距离
                    float ld2 = a012 / l12;

                    // 计算权重系数
                    float s = 1 - 0.9 * fabs(ld2);

                    // 保存特征匹配的结果
                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;

                    // 如果权重系数足够大，说明是一个好的特征匹配
                    if (s > 0.1) {
                        laserCloudOriCornerVec[i] = pointOri;      // 保存原始角点
                        coeffSelCornerVec[i] = coeff;              // 保存特征系数
                        laserCloudOriCornerFlag[i] = true;         // 标记该点为有效的特征点
                    }
                }
            }
        }
    }

    /**
     * @brief 平面特征的优化函数
     * 
     * 该函数对当前帧中的平面特征点进行优化，主要步骤包括:
     * 1. 更新点云到地图的变换矩阵
     * 2. 对每个平面特征点进行处理
     * 3. 在地图中搜索最近邻点
     * 4. 拟合平面并计算点到平面的距离
     * 5. 进行特征匹配和优化
     */
    void surfOptimization()
    {
        // 更新当前帧点云到地图坐标系的变换矩阵
        updatePointAssociateToMap();

        // 使用OpenMP进行并行计算，加速处理过程
        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudSurfLastDSNum; i++)
        {
            // 定义点云数据结构和搜索结果存储容器
            PointType pointOri, pointSel, coeff;  // 原始点、转换后的点、系数
            std::vector<int> pointSearchInd;      // 搜索结果点的索引
            std::vector<float> pointSearchSqDis;  // 搜索结果点的距离平方

            // 获取当前处理的平面点（feature提取平面点-->feature降采样-->optmization降采样-->laserCloudSurfLastDS）
            // 目标点少之又少
            pointOri = laserCloudSurfLastDS->points[i];
            // 将平面点转换到地图坐标系
            pointAssociateToMap(&pointOri, &pointSel); 
            // 在地图中搜索距离该点最近的5个点（没有特征要求）
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            // 下面的过程要求解Ax+By+Cz+1=0的平面方程
            // 由于有5个点，因此是求解超定方程
            // 假设5个点都在平面上，则matA0是系数矩阵，matB0是等号右边的值（都是-1）；matX0是求出来的A，B，C
            // 创建用于平面拟合的矩阵
            Eigen::Matrix<float, 5, 3> matA0;  // 系数矩阵
            Eigen::Matrix<float, 5, 1> matB0;  // 常数项
            Eigen::Vector3f matX0;             // 解向量

            // 初始化矩阵
            matA0.setZero();     // 将系数矩阵置零
            matB0.fill(-1);      // 将常数项全部设为-1
            matX0.setZero();     // 将解向量置零

            // 如果最远点的距离小于1.0，则进行平面拟合
            if (pointSearchSqDis[4] < 1.0) {
                // 构建平面方程的系数矩阵
                for (int j = 0; j < 5; j++) {
                    matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }

                // 这里是求解matA0XmatX0 = matB0方程
                // 使用QR分解求解平面方程 Ax=b
                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                // 获取平面方程的参数 ax + by + cz + d = 0
                float pa = matX0(0, 0);  // a
                float pb = matX0(1, 0);  // b
                float pc = matX0(2, 0);  // c
                float pd = 1;            // d

                // （pa,pb,pc)是平面的法向量，这里是对法向量规一化，变成单位法向量
                // 对平面方程进行归一化
                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps; pb /= ps; pc /= ps; pd /= ps;

                // 验证平面的有效性
                bool planeValid = true;
                // 检查所有点到拟合平面的距离是否都在阈值范围内
                for (int j = 0; j < 5; j++) {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }

                // 如果平面有效，计算特征值
                if (planeValid) {
                    // 计算当前点到平面的距离
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    // 计算权重系数
                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x
                            + pointOri.y * pointOri.y + pointOri.z * pointOri.z));

                    // 保存特征匹配的结果
                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    // 如果权重系数足够大，说明是一个好的特征匹配
                    if (s > 0.1) {
                        laserCloudOriSurfVec[i] = pointOri;      // 保存原始平面点
                        coeffSelSurfVec[i] = coeff;              // 保存特征系数
                        laserCloudOriSurfFlag[i] = true;         // 标记该点为有效的特征点
                    }
                }
            }
        }
    }

    /**
     * @brief 合并角点和平面点的优化系数
     * 
     * 该函数将角点和平面点的优化结果合并到统一的数据结构中，为后续的优化准备数据。
     * 主要步骤包括:
     * 1. 合并有效的角点特征及其系数
     * 2. 合并有效的平面点特征及其系数  
     * 3. 重置标志位为下一次迭代做准备
     */
    void combineOptimizationCoeffs()
    {
        // 遍历所有角点特征，将有效的角点及其优化系数添加到统一的数据结构中
        for (int i = 0; i < laserCloudCornerLastDSNum; ++i){
            // 检查当前角点是否为有效特征点
            if (laserCloudOriCornerFlag[i] == true){
                // 将有效的角点特征添加到 laserCloudOri
                laserCloudOri->push_back(laserCloudOriCornerVec[i]);
                // 将对应的优化系数添加到 coeffSel
                coeffSel->push_back(coeffSelCornerVec[i]);
            }
        }
        // 遍历所有平面点特征，将有效的平面点及其优化系数添加到统一的数据结构中
        for (int i = 0; i < laserCloudSurfLastDSNum; ++i){
            // 检查当前平面点是否为有效特征点
            if (laserCloudOriSurfFlag[i] == true){
                // 将有效的平面点特征添加到 laserCloudOri
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                // 将对应的优化系数添加到 coeffSel
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }
        // 重置角点和平面点的标志位数组，为下一次迭代做准备
        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    }

    /**
     * @brief Levenberg-Marquardt优化算法实现
     * 
     * 该函数实现了基于LM算法的位姿优化,主要步骤包括:
     * 1. 坐标系转换(从激光雷达坐标系到相机坐标系)
     * 2. 构建优化问题的雅可比矩阵和误差向量
     * 3. 求解优化问题
     * 4. 处理退化情况
     * 5. 更新位姿估计
     * 
     * @param iterCount 当前迭代次数
     * @return true 如果优化收敛, false 如果需要继续优化
     */
    bool LMOptimization(int iterCount)
    {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll

        // 计算三轴欧拉角的sin、cos，后面使用旋转矩阵对欧拉角求导中会使用到
        // lidar -> camera
        // 计算旋转矩阵的三角函数值
        float srx = sin(transformTobeMapped[1]);  // sin(pitch)
        float crx = cos(transformTobeMapped[1]);  // cos(pitch)
        float sry = sin(transformTobeMapped[2]);  // sin(yaw)
        float cry = cos(transformTobeMapped[2]);  // cos(yaw)
        float srz = sin(transformTobeMapped[0]);  // sin(roll)
        float crz = cos(transformTobeMapped[0]);  // cos(roll)

        // laserCloudOri是在cornerOptimization、surfOptimization两个函数中找到的有匹配关系的
        // 角点和平面点，如果找到的可供优化的点数太少，则跳过此次优化
        // 获取特征点的数量,如果少于50个点则返回false
        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < 50) {
            return false;
        }

        // matA是Jacobians矩阵J
        // 创建优化问题相关的矩阵
        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));    // 雅可比矩阵
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));   // 雅可比矩阵的转置
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));                 // J^T * J
        // matB是目标函数，也就是距离
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));    // 误差向量
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));                 // J^T * err
        // matX是高斯-牛顿法计算出的更新向量
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));                   // 优化增量
        // cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

        PointType pointOri, coeff;  // 用于存储原始点和对应的系数

        // 构建雅可比矩阵和误差向量
        for (int i = 0; i < laserCloudSelNum; i++) {
            // 将激光雷达坐标系下的点转换到相机坐标系
            pointOri.x = laserCloudOri->points[i].y;
            pointOri.y = laserCloudOri->points[i].z;
            pointOri.z = laserCloudOri->points[i].x;
            
            // 将系数也转换到相机坐标系
            coeff.x = coeffSel->points[i].y;
            coeff.y = coeffSel->points[i].z;
            coeff.z = coeffSel->points[i].x;
            coeff.intensity = coeffSel->points[i].intensity;

            // 计算雅可比矩阵中的元素
            // arx, ary, arz分别对应roll, pitch, yaw的偏导数
            // in camera
            // 求雅克比矩阵的值，也就是求目标函数（点到线、平面的距离）相对于tx,ty,tz,rx,ry,rz的导数
            // 具体的公式推导看仓库README中本项目博客，高斯牛顿法方程：J^{T}J\Delta{x} = -Jf(x)，\Delta{x}就是要求解的更新向量matX
            // arx是目标函数相对于roll的导数
            float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                      + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                      + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;
            // ary是目标函数相对于pitch的导数
            float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                      + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                      + ((-cry*crz - srx*sry*srz)*pointOri.x 
                      + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;
            // arz是目标函数相对于yaw的导数
            float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                      + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                      + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;

            /*
            在求点到直线的距离时，coeff表示的是如下内容
            [la,lb,lc]表示的是点到直线的垂直连线方向，s是长度
            coeff.x = s * la;
            coeff.y = s * lb;
            coeff.z = s * lc;
            coeff.intensity = s * ld2;

            在求点到平面的距离时，coeff表示的是
            [pa,pb,pc]表示过外点的平面的法向量，s是线的长度
            coeff.x = s * pa;
            coeff.y = s * pb;
            coeff.z = s * pc;
            coeff.intensity = s * pd2;
            */

            // 填充雅可比矩阵和误差向量
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = arx;
            matA.at<float>(i, 2) = ary;
            // 目标函数相对于tx的导数等于法向量的x
            matA.at<float>(i, 3) = coeff.z;
            // 目标函数相对于ty的导数等于法向量的y
            matA.at<float>(i, 4) = coeff.x;
            // 目标函数相对于tz的导数等于法向量的z
            matA.at<float>(i, 5) = coeff.y;
            // matB存储的是目标函数（距离）的负值，因为：J^{T}J\Delta{x} = -Jf(x)
            matB.at<float>(i, 0) = -coeff.intensity;
        }

        // 计算 J^T * J 和 J^T * err
        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        // 使用QR分解求解线性方程组
        // 求解高斯-牛顿法中的增量方程：J^{T}J\Delta{x} = -Jf(x)，这里解出来的matX就是更新向量
        // matA是雅克比矩阵J
        // matAtB是上面等式中等号的右边，负号在matB赋值的时候已经加入
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        // 在第一次迭代时检查是否存在退化
        // 如果是第一次迭代，判断求解出来的近似Hessian矩阵，也就是J^{T}J:=matAtA是否退化
        /**
            * 这部分的计算说实话没有找到很好的理论出处，这里只能大概说一下这段代码想要做的事情
            * 这里用matAtA也就是高斯-牛顿中的近似海瑟（Hessian）矩阵H。求解增量方程：J^{T}J\Delta{x} = -Jf(x)
            * 要求H:=J^{T}J可逆，但H不一定可逆。下面的代码通过H的特征值判断H是否退化，并将退化的方向清零matV2。而后又根据
            * matV.inv()*matV2作为更新向量的权重系数，matV是H的特征向量矩阵。
        */
        if (iterCount == 0) {
            // 创建用于特征值分解的矩阵
            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));  // 特征值
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));  // 特征向量
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0)); // 处理后的特征向量

            // 对 J^T * J 进行特征值分解
            // 对近似Hessian矩阵做特征值分解，matE是特征值，matV是特征向量。opencv的matV中每一行是一个特征向量
            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            // 检查是否存在退化
            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};  // 特征值阈值
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            // 计算投影矩阵用于处理退化情况
            matP = matV.inv() * matV2;
        }

        // 如果存在退化,使用投影矩阵处理优化增量
        // 当第一次迭代判断到海瑟矩阵退化，后面会使用计算出来的权重matP对增量matX做加权组合
        if (isDegenerate)
        {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        // 更新位姿估计
        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        // 计算旋转和平移的增量大小
        // 计算roll、pitch、yaw的迭代步长
        float deltaR = sqrt(
                            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        // 计算平移的迭代步长
        float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

        // 检查是否收敛
        // 如果迭代的步长达到设定阈值，则认为已经收敛
        if (deltaR < 0.05 && deltaT < 0.05) {
            return true; // 收敛
        }
        return false; // 继续优化
    }

    /**
     * @brief 执行扫描匹配优化的主要函数
     * 
     * 该函数通过迭代优化当前帧与局部地图之间的位姿变换关系，主要步骤包括:
     * 1. 检查是否有足够的特征点用于匹配
     * 2. 设置KD树用于特征点搜索
     * 3. 迭代执行位姿优化
     * 4. 更新最终的变换矩阵
     */
    void scan2MapOptimization()
    {
        // 如果没有关键帧位姿，则直接返回
        if (cloudKeyPoses3D->points.empty())
            return;

        // 检查当前帧中的特征点数量是否满足最小要求
        if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum)
        {
            // 设置用于特征匹配的KD树输入点云
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);  // 角点特征KD树
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);      // 平面特征KD树

            // 最多迭代30次进行位姿优化
            for (int iterCount = 0; iterCount < 30; iterCount++)
            {
                // 清空用于存储特征对应关系的容器
                laserCloudOri->clear();  // 清空原始点云
                coeffSel->clear();       // 清空特征系数

                // 执行角点特征的优化
                cornerOptimization();
                // 执行平面特征的优化
                surfOptimization();

                // 合并角点和平面特征的优化系数
                combineOptimizationCoeffs();

                // 执行LM优化算法，如果收敛则退出迭代
                // 相比于NG优化算法，LM优化算法收敛更快，但是可能会出现局部最优解
                // 两阶段优化建议：先用LM优化算法进行初步优化，然后再用NG优化算法进行全局优化
                // 3DGS-LM通过GPU并行化PCG，Dog-Leg
                // 全局最优：遗传算法、模拟退火
                if (LMOptimization(iterCount) == true)
                    break;              
            }

            // 更新位姿变换
            transformUpdate();
        } else {
            // 如果特征点数量不足，输出警告信息
            ROS_WARN("Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
        }
    }

    /**
     * @brief 更新变换矩阵，融合IMU数据以提高姿态估计的准确性
     * 
     * 该函数主要完成以下工作:
     * 1. 如果有IMU数据可用，使用IMU数据对roll和pitch进行修正
     * 2. 对姿态角和z轴平移进行约束，防止估计值偏离合理范围
     * 3. 更新增量里程计的后端变换矩阵
     */
    void transformUpdate()
    {
        // 检查是否有IMU数据可用
        if (cloudInfo.imuAvailable == true)
        {
            // 当IMU的pitch角在合理范围内时(-1.4~1.4弧度，约±80度)才使用IMU数据
            if (std::abs(cloudInfo.imuPitchInit) < 1.4)
            {
                // 设置IMU数据的权重
                double imuWeight = imuRPYWeight;
                // 创建四元数对象，用于存储IMU和激光雷达的姿态
                tf::Quaternion imuQuaternion;
                tf::Quaternion transformQuaternion;
                // 用于存储球面线性插值(SLERP)后的姿态角
                double rollMid, pitchMid, yawMid;

                // 对roll角进行SLERP插值
                // 将激光雷达估计的roll角转换为四元数
                transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
                // 将IMU测量的roll角转换为四元数
                imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                // 使用SLERP进行插值，并提取结果中的roll角
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[0] = rollMid;

                // 对pitch角进行SLERP插值，过程类似
                transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
                imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[1] = pitchMid;
            }
        }

        // 对姿态角和z轴平移进行约束
        // 限制roll角在允许范围内
        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        // 限制pitch角在允许范围内
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        // 限制z轴平移在允许范围内
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);

        // 将最新的位姿变换更新到增量里程计的后端变换矩阵
        incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
    }

    /**
     * @brief 约束变换值在指定范围内
     * 
     * 该函数用于限制变换参数(如旋转角度、平移距离等)的大小,防止其超出合理范围。
     * 具体实现方式是:
     * 1. 如果输入值小于负限制值,则将其设为负限制值
     * 2. 如果输入值大于正限制值,则将其设为正限制值
     * 3. 如果输入值在限制范围内,则保持不变
     * 
     * @param value 需要被约束的变换值
     * @param limit 约束范围的上限值(对应的下限为-limit)
     * @return float 约束后的变换值
     */
    float constraintTransformation(float value, float limit)
    {
        // 如果输入值小于负限制值,将其设置为负限制值
        if (value < -limit)
            value = -limit;
        // 如果输入值大于正限制值,将其设置为正限制值  
        if (value > limit)
            value = limit;

        // 返回约束后的值
        return value;
    }

    /**
     * @brief 判断是否需要保存当前帧作为关键帧
     * 
     * 该函数通过计算当前帧与上一关键帧之间的相对变换来决定是否需要保存当前帧。
     * 主要通过比较两帧之间的旋转角度和平移距离与预设阈值的关系来做出判断。
     * 
     * @return bool 如果需要保存当前帧则返回true，否则返回false
     */
    bool saveFrame()
    {
        // 如果关键帧位姿点云为空，说明是第一帧，需要保存
        if (cloudKeyPoses3D->points.empty())
            return true;

        // 获取上一关键帧的位姿变换矩阵
        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        // 获取当前帧的位姿变换矩阵
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        // 计算两帧之间的相对变换
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        // 用于存储相对变换的平移和旋转值
        float x, y, z, roll, pitch, yaw;
        // 从变换矩阵中提取平移和欧拉角
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        // 判断相对变换是否小于阈值
        // 如果roll、pitch、yaw的绝对值都小于角度阈值
        // 且平移距离小于距离阈值，则认为当前帧与上一关键帧相似
        // 不需要保存为新的关键帧
        if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
            abs(pitch) < surroundingkeyframeAddingAngleThreshold && 
            abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
            sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
            return false;
        
        // 如果相对变换超过阈值，需要保存为新的关键帧
        return true;
    }

    /**
     * @brief 添加里程计因子到因子图中
     * 
     * 该函数负责向GTSAM因子图中添加里程计相关的因子,包括:
     * 1. 如果是第一帧,添加先验因子作为位姿图的起点
     * 2. 如果不是第一帧,添加两个相邻位姿之间的里程计约束因子
     */
    void addOdomFactor()
    {
        // 如果是第一帧数据(位姿点云为空)
        if (cloudKeyPoses3D->points.empty())
        {
            // 创建一个对角噪声模型作为先验噪声
            // 旋转部分噪声为1e-2 rad^2
            // 平移部分噪声为1e8 m^2(较大值表示对平移先验的低信任度)
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
            
            // 添加先验因子到因子图
            // 节点ID为0,使用当前位姿作为先验值
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
            
            // 将第一帧位姿添加到初始估计中
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));

        }else{
            // 如果不是第一帧,创建里程计噪声模型
            // 旋转部分噪声为1e-6 rad^2
            // 平移部分噪声为1e-4 m^2
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            
            // 获取前一帧的位姿
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            // 获取当前帧的位姿
            gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
            
            // 添加两帧之间的里程计因子到因子图
            // 使用相对位姿作为约束
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
            
            // 将当前帧位姿添加到初始估计中
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        }
    }

    /**
     * @brief 添加GPS因子到因子图中
     * 
     * 该函数负责处理GPS数据并将其作为约束添加到GTSAM因子图中。主要步骤包括:
     * 1. 检查GPS数据的有效性和时效性
     * 2. 根据GPS数据的质量和系统状态决定是否添加GPS因子
     * 3. 将符合条件的GPS数据转换为因子图约束
     */
    void addGPSFactor()
    {
        // 如果GPS数据队列为空,直接返回
        if (gpsQueue.empty())
            return;

        // wait for system initialized and settles down
        // 等待系统初始化完成并稳定下来
        // 如果没有关键帧位姿或系统运动距离太短(<5m),则返回
        if (cloudKeyPoses3D->points.empty())
            return;
        else
        {
            if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
                return;
        }

        // pose covariance small, no need to correct
        // 如果位姿协方差较小,说明当前定位精度较高,不需要GPS校正
        if (poseCovariance(3,3) < poseCovThreshold && poseCovariance(4,4) < poseCovThreshold)
            return;

        // 用于记录上一次使用的GPS位置
        static PointType lastGPSPoint;

        // 处理GPS数据队列
        while (!gpsQueue.empty())
        {
            // 如果GPS数据太旧(早于当前时间0.2秒),则丢弃
            if (gpsQueue.front().header.stamp.toSec() < timeLaserInfoCur - 0.2)
            {
                // message too old
                gpsQueue.pop_front();
            }
            // 如果GPS数据太新(晚于当前时间0.2秒),则等待下次处理
            else if (gpsQueue.front().header.stamp.toSec() > timeLaserInfoCur + 0.2)
            {
                // message too new
                break;
            }
            else
            {
                // 获取当前GPS数据并从队列中移除
                nav_msgs::Odometry thisGPS = gpsQueue.front();
                gpsQueue.pop_front();

                // GPS too noisy, skip
                // 获取GPS数据的协方差
                float noise_x = thisGPS.pose.covariance[0];
                float noise_y = thisGPS.pose.covariance[7];
                float noise_z = thisGPS.pose.covariance[14];
                // 如果GPS噪声太大,则跳过该数据
                if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                    continue;

                // 获取GPS位置数据
                float gps_x = thisGPS.pose.pose.position.x;
                float gps_y = thisGPS.pose.pose.position.y;
                float gps_z = thisGPS.pose.pose.position.z;
                // 如果不使用GPS高程数据,则使用激光SLAM的高程估计
                if (!useGpsElevation)
                {
                    gps_z = transformTobeMapped[5];
                    noise_z = 0.01;
                }

                // GPS not properly initialized (0,0,0)
                // 如果GPS未正确初始化(位置接近原点),则跳过
                if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                    continue;

                // Add GPS every a few meters
                // 每隔几米添加一次GPS约束
                PointType curGPSPoint;
                curGPSPoint.x = gps_x;
                curGPSPoint.y = gps_y;
                curGPSPoint.z = gps_z;
                // 如果与上次添加的GPS点距离太近(<5m),则跳过
                if (pointDistance(curGPSPoint, lastGPSPoint) < 5.0)
                    continue;
                else
                    lastGPSPoint = curGPSPoint;

                // 创建GPS噪声模型,确保噪声值不小于1.0
                gtsam::Vector Vector3(3);
                Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
                noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
                // 创建GPS因子并添加到因子图中
                gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
                gtSAMgraph.add(gps_factor);

                // 标记已添加回环,触发图优化
                aLoopIsClosed = true;
                break;
            }
        }
    }

    /**
     * @brief 添加回环因子到因子图中
     * 
     * 该函数负责将检测到的回环约束添加到GTSAM因子图中。主要步骤包括:
     * 1. 检查回环队列是否为空
     * 2. 遍历所有待处理的回环约束
     * 3. 将回环约束转换为因子图中的边
     * 4. 清理相关队列并标记回环状态
     */
    void addLoopFactor()
    {
        // 如果回环索引队列为空,则直接返回
        if (loopIndexQueue.empty())
            return;

        // 遍历所有待处理的回环约束
        for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
        {
            // 获取回环帧对的起始索引
            int indexFrom = loopIndexQueue[i].first;
            // 获取回环帧对的目标索引
            int indexTo = loopIndexQueue[i].second;
            // 获取两帧之间的相对位姿变换
            gtsam::Pose3 poseBetween = loopPoseQueue[i];
            // 获取对应的噪声模型
            gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
            // 向因子图中添加回环约束边
            gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
        }

        // 清空回环索引队列
        loopIndexQueue.clear();
        // 清空回环位姿队列
        loopPoseQueue.clear();
        // 清空回环噪声队列
        loopNoiseQueue.clear();
        // 标记已添加回环,触发图优化
        aLoopIsClosed = true;
    }

    /**
     * @brief 保存关键帧和因子图相关信息的函数
     * 
     * 该函数负责:
     * 1. 添加各类因子(里程计、GPS、回环)到因子图
     * 2. 执行iSAM2优化更新位姿估计
     * 3. 保存关键帧位姿和特征点云
     * 4. 更新可视化路径
     * 5. 发布相关信息
     */
    void saveKeyFramesAndFactor()
    {
        // 检查当前帧是否需要保存为关键帧,如果不需要则直接返回
        if (saveFrame() == false)
            return;

        // 向因子图中添加里程计因子
        addOdomFactor();

        // 向因子图中添加GPS因子(如果有)
        addGPSFactor();

        // 向因子图中添加回环因子(如果有)
        addLoopFactor();

        // 使用iSAM2优化器更新因子图
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        // 如果检测到回环,则多次更新优化器以确保收敛
        if (aLoopIsClosed == true)
        {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }

        // 清空因子图和初始估计,为下一次优化做准备
        gtSAMgraph.resize(0);
        initialEstimate.clear();

        // 定义用于存储位姿的变量
        PointType thisPose3D;        // 3D位姿(x,y,z)
        PointTypePose thisPose6D;    // 6D位姿(x,y,z,roll,pitch,yaw)
        Pose3 latestEstimate;        // GTSAM位姿估计结果

        // 计算优化后的位姿估计
        isamCurrentEstimate = isam->calculateEstimate();
        // 获取最新的位姿估计结果（一连串结果）
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);

        // 保存3D位姿信息
        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); // 使用点云大小作为索引
        cloudKeyPoses3D->push_back(thisPose3D);

        // 保存6D位姿信息
        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity; // 使用点云大小作为索引
        thisPose6D.roll  = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw   = latestEstimate.rotation().yaw();
        thisPose6D.time = timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);

        // 保存位姿协方差矩阵
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);

        // 更新待映射的变换矩阵
        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();

        // 创建并保存当前关键帧的角点和平面点云
        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*laserCloudCornerLastDS,  *thisCornerKeyFrame);
        pcl::copyPointCloud(*laserCloudSurfLastDS,    *thisSurfKeyFrame);

        // 将当前帧的特征点云添加到关键帧序列中
        cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

        // 更新用于可视化的路径信息
        updatePath(thisPose6D);

        // 更新cloudInfo中的初始猜测值
        cloudInfo.initialGuessX = thisPose6D.x;
        cloudInfo.initialGuessY = thisPose6D.y;
        cloudInfo.initialGuessZ = thisPose6D.z;
        cloudInfo.initialGuessRoll  = thisPose6D.roll;
        cloudInfo.initialGuessPitch = thisPose6D.pitch;
        cloudInfo.initialGuessYaw   = thisPose6D.yaw;
        cloudInfo.imuAvailable = cloudKeyPoses6D->size() - 1;

        // 发布特征点云信息
        cloudInfo.cloud_corner = publishCloud(&pubFeatureCloud,  laserCloudCornerLastFeature,  cloudInfo.header.stamp, robot_id + "/" + lidarFrame);
        cloudInfo.cloud_surface = publishCloud(&pubFeatureCloud,  laserCloudSurfLastFeature,  cloudInfo.header.stamp, robot_id + "/" + lidarFrame);
        // 发布激光雷达点云信息
        pubLaserCloudInfo.publish(cloudInfo);
        // cout << robot_id + " cloudInfo " << cloudInfo.header << endl;
        // saveToFile(cloudInfo, robot_id + "cloudInfo.txt");
    }

    /**
     * @brief 在检测到回环时更正所有历史关键帧位姿
     * 
     * 该函数在检测到回环后执行以下操作:
     * 1. 清除地图缓存和路径信息
     * 2. 使用iSAM2优化器的最新估计更新所有历史关键帧的位姿
     * 3. 重新生成全局路径
     */
    void correctPoses()
    {
        // 如果没有关键帧位姿数据，直接返回
        if (cloudKeyPoses3D->points.empty())
            return;

        // 只有在检测到回环时才执行位姿校正
        if (aLoopIsClosed == true)
        {
            // 清除地图缓存，因为所有历史关键帧的位姿都将被更新
            laserCloudMapContainer.clear();
            // 清除全局路径，准备重新生成
            globalPath.poses.clear();
            
            // 获取当前位姿图中的节点数量
            int numPoses = isamCurrentEstimate.size();
            // 遍历所有历史关键帧，使用优化后的位姿进行更新
            for (int i = 0; i < numPoses; ++i)
            {
                // 更新3D位姿点云中的位置信息(x,y,z)
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

                // 更新6D位姿点云中的位置信息，与3D位姿保持一致
                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                // 更新6D位姿点云中的姿态信息(roll,pitch,yaw)
                cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

                // 使用更新后的位姿重新生成全局路径
                updatePath(cloudKeyPoses6D->points[i]);
            }

            // 重置回环检测标志，表示位姿校正已完成
            aLoopIsClosed = false;
        }
    }

    /**
     * @brief 更新全局路径信息
     * 
     * 该函数将新的位姿信息转换为ROS消息格式并添加到全局路径中。主要步骤包括:
     * 1. 创建带时间戳的位姿消息
     * 2. 设置位姿的位置和方向信息
     * 3. 将位姿添加到全局路径中
     * 
     * @param pose_in 输入的位姿信息，包含位置和姿态数据
     */
    void updatePath(const PointTypePose& pose_in)
    {
        // 创建一个带时间戳的位姿消息
        geometry_msgs::PoseStamped pose_stamped;
        // 将位姿的时间转换为ROS时间戳
        pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
        // 设置消息的坐标系
        pose_stamped.header.frame_id = robot_id + "/" + odometryFrame;
        
        // 设置位姿的位置信息(x,y,z)
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        
        // 将欧拉角(roll,pitch,yaw)转换为四元数
        tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        // 设置位姿的方向信息(四元数)
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        // 将新的位姿添加到全局路径中
        globalPath.poses.push_back(pose_stamped);
    }

    /**
     * @brief 发布里程计信息的函数
     * 
     * 该函数负责发布全局和增量里程计信息，以及相关的TF变换。主要包括:
     * 1. 发布全局里程计信息
     * 2. 发布里程计TF变换
     * 3. 发布增量里程计信息(包含IMU融合)
     */
    void publishOdometry()
    {
        // 创建并发布全局里程计消息
        nav_msgs::Odometry laserOdometryROS;
        // 设置消息头的时间戳和坐标系信息
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = robot_id + "/" + odometryFrame;
        laserOdometryROS.child_frame_id = robot_id + "/" + lidarFrame + "/odom_mapping";
        // 设置位置信息(x,y,z)
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        // 设置姿态信息(将欧拉角转换为四元数)
        laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        // 发布全局里程计消息
        pubLaserOdometryGlobal.publish(laserOdometryROS);

        // 发布到融合节点(预留)
        // pubCloudInfoWithPose.publish();

        // 发布TF变换
        // 创建TF广播器(静态变量，避免重复创建)
        static tf::TransformBroadcaster br;
        // 创建从里程计坐标系（全局）到激光雷达坐标系（雷达坐标系）的变换
        tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
                                                      tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        // 创建带时间戳的TF变换消息
        tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, timeLaserInfoStamp, robot_id + "/" + odometryFrame, robot_id + "/" + lidarFrame + "/lidar_link");
        // 发布TF变换
        br.sendTransform(trans_odom_to_lidar);

        // 发布增量里程计信息
        // 使用静态变量记录上一次发布状态和数据
        static bool lastIncreOdomPubFlag = false;
        static nav_msgs::Odometry laserOdomIncremental; // 增量里程计消息
        static Eigen::Affine3f increOdomAffine; // 仿射变换形式的增量里程计
        
        // 第一次发布时的处理
        if (lastIncreOdomPubFlag == false)
        {
            // 标记已经发布过
            lastIncreOdomPubFlag = true;
            // 使用全局里程计初始化增量里程计
            // 位置
            laserOdomIncremental = laserOdometryROS;
            // 旋转
            increOdomAffine = trans2Affine3f(transformTobeMapped);
        } else {
            // 计算相对于上一帧的增量变换
            Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            // 更新累积的增量变换
            increOdomAffine = increOdomAffine * affineIncre;
            // 从仿射变换矩阵中提取位置和姿态
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles (increOdomAffine, x, y, z, roll, pitch, yaw);

            // 如果有IMU数据可用，进行姿态融合
            if (cloudInfo.imuAvailable == true)
            {
                // 当IMU的pitch角在合理范围内时才进行融合
                if (std::abs(cloudInfo.imuPitchInit) < 1.4)
                {
                    // IMU数据的权重
                    double imuWeight = 0.1;
                    tf::Quaternion imuQuaternion;
                    tf::Quaternion transformQuaternion;
                    double rollMid, pitchMid, yawMid;

                    // 使用SLERP对roll角进行融合
                    transformQuaternion.setRPY(roll, 0, 0);
                    imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    roll = rollMid;

                    // 使用SLERP对pitch角进行融合
                    transformQuaternion.setRPY(0, pitch, 0);
                    imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    pitch = pitchMid;
                }
            }

            // 设置增量里程计消息的各项参数
            laserOdomIncremental.header.stamp = timeLaserInfoStamp;
            laserOdomIncremental.header.frame_id = robot_id + "/" + odometryFrame;
            laserOdomIncremental.child_frame_id = robot_id + "/" + lidarFrame + "/odom_mapping";
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            laserOdomIncremental.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
            
            // 根据是否处于退化状态设置协方差
            if (isDegenerate)
                laserOdomIncremental.pose.covariance[0] = 1;  // 退化状态，设置较大协方差
            else
                laserOdomIncremental.pose.covariance[0] = 0;  // 正常状态，设置较小协方差
        }
        // 发布增量里程计消息
        pubLaserOdometryIncremental.publish(laserOdomIncremental);
    }

    /**
     * @brief 发布各种可视化信息的函数
     * 
     * 该函数负责发布:
     * 1. 关键帧位姿点云
     * 2. 周围关键帧的特征点云
     * 3. 当前关键帧的配准结果
     * 4. 原始点云的配准结果
     * 5. 全局路径信息
     */
    void publishFrames()
    {
        // 如果没有关键帧位姿数据，直接返回
        if (cloudKeyPoses3D->points.empty())
            return;

        // 发布关键帧位姿点云，用于可视化轨迹
        publishCloud(&pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, robot_id + "/" + odometryFrame);

        // 发布周围关键帧的平面特征点云，用于可视化局部地图
        publishCloud(&pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, robot_id + "/" + odometryFrame);

        // 发布当前关键帧的配准结果
        // 只有在有订阅者时才进行处理，避免不必要的计算
        if (pubRecentKeyFrame.getNumSubscribers() != 0)
        {
            // 创建用于存储配准结果的点云
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            // 将当前的变换矩阵转换为位姿数据结构
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            // 将当前帧的角点和平面点特征转换到全局坐标系下
            *cloudOut += *transformPointCloud(laserCloudCornerLastDS,  &thisPose6D);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS,    &thisPose6D);
            // 发布转换后的点云
            publishCloud(&pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, robot_id + "/" + odometryFrame);
        }

        // 发布配准后的高分辨率原始点云
        // 同样只在有订阅者时处理
        if (pubCloudRegisteredRaw.getNumSubscribers() != 0)
        {
            // 创建用于存储配准结果的点云
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            // 将ROS消息格式的点云转换为PCL格式
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
            // 将当前的变换矩阵转换为位姿数据结构
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            // 将原始点云转换到全局坐标系下
            *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
            // 发布转换后的点云
            publishCloud(&pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, robot_id + "/" + odometryFrame);
        }

        // 发布全局路径
        // 同样只在有订阅者时处理
        if (pubPath.getNumSubscribers() != 0)
        {
            // 更新路径消息的时间戳和坐标系信息
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = robot_id + "/" + odometryFrame;
            // 发布全局路径消息
            pubPath.publish(globalPath);
        }
    }
};


/**
 * @brief 主函数，程序的入口点，负责初始化 ROS 节点，启动地图优化相关线程并保持 ROS 节点运行。
 * 
 * @param argc 命令行参数的数量。
 * @param argv 命令行参数的字符串数组。
 * @return int 程序退出状态码，0 表示正常退出。
 */
int main(int argc, char** argv)
{
    // 初始化 ROS 节点，节点名为 "disco_double"
    ros::init(argc, argv, "disco_double");

    // 创建 mapOptimization 类的一个实例 MO
    mapOptimization MO;

    // 在终端输出绿色的提示信息，表明地图优化已经启动
    ROS_INFO("\033[1;32m----> Map Optimization Started.\033[0m");
    
    // 此处的多线程与ROS多线程不同
    // C++ 标准库提供的多线程工具，可用于创建任意功能的线程。
    // 可以创建执行任意函数的线程，不仅限于 ROS 相关任务。
    // 需要手动管理线程的生命周期，包括创建、启动、等待线程结束（使用 join 或 detach 方法）。
    // std::thread 创建的线程和 ROS 的消息处理机制没有直接关联，需要用户自己协调线程间的通信和同步。

    // 创建一个新线程，调用 mapOptimization 类的 loopClosureThread 方法，传入 MO 实例的指针
    std::thread loopthread(&mapOptimization::loopClosureThread, &MO);
    // 创建另一个新线程，调用 mapOptimization 类的 visualizeGlobalMapThread 方法，传入 MO 实例的指针
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);

    // 进入 ROS 的主循环，处理订阅的消息和回调函数，直到节点被关闭
    ros::spin();

    // 等待 loopthread 线程结束
    loopthread.join();
    // 等待 visualizeMapThread 线程结束
    visualizeMapThread.join();

    /*
    | 比较项 | ros::MultiThreadedSpinner | std::thread | 
    | ----- | ------------------------- | ----------- | 
    |  用途  |    专门处理 ROS 回调函数    | 可执行任意函数 | 
    | 集成度 |      与 ROS 深度集成        |  与 ROS 解耦 | 
    | 线程管理 |        自动管理           |   手动管理   | 
    | 应用场景 | 适用于提高 ROS 消息处理的并发性能 | 适用于实现自定义的多线程任务 |
    */
    
    // 程序正常退出，返回 0
    return 0;
}
