// 包含自定义工具头文件，该文件可能包含一些通用的工具函数、类型定义等
#include "utility.h"
// 包含 disco_slam 包中的 cloud_info 消息类型头文件，用于处理点云信息
#include "disco_slam/cloud_info.h"
// 包含 disco_slam 包中的 context_info 消息类型头文件，用于处理上下文信息
#include "disco_slam/context_info.h"

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
    // GTSAM 初始估计值，存储变量的初始猜测值
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
//    ros::Publisher pubCloudInfoWithPose;
//    disco_slam::cloud_info cloudInfoWithPose;
//    std_msgs::Header cloudHeader;
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
    disco_slam::cloud_info cloudInfo;

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

        // 初始化 ROS 发布者，用于发布关键位姿点云
        pubKeyPoses                 = nh.advertise<sensor_msgs::PointCloud2>(robot_id + "/disco_slam/mapping/trajectory", 1);
        // 初始化 ROS 发布者，用于发布全局地图点云
        pubLaserCloudSurround       = nh.advertise<sensor_msgs::PointCloud2>(robot_id + "/disco_slam/mapping/map_global", 1);
        // 初始化 ROS 发布者，用于发布全局激光里程计信息
        pubLaserOdometryGlobal      = nh.advertise<nav_msgs::Odometry> (robot_id + "/disco_slam/mapping/odometry", 1);
        // 初始化 ROS 发布者，用于发布增量式激光里程计信息
        pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry> (robot_id + "/disco_slam/mapping/odometry_incremental", 1);
        // 初始化 ROS 发布者，用于发布全局路径信息
        pubPath                     = nh.advertise<nav_msgs::Path>(robot_id + "/disco_slam/mapping/path", 1);

        // 注释掉的代码，原本用于发布到融合节点的点云信息
        //for fusion node
        //for
//        pubCloudInfoWithPose        = nh.advertise<disco_slam::cloud_info> (robot_id + "/disco_slam/mapping/cloud_info", 1);
        // 初始化 ROS 发布者，用于发布全局特征点云
        pubFeatureCloud = nh.advertise<sensor_msgs::PointCloud2>(robot_id + "/disco_slam/mapping/feature_cloud_global", 1);
        // 初始化 ROS 订阅者，订阅全局回环信息，并指定回调函数
        subGlobalLoop = nh.subscribe<disco_slam::context_info>(robot_id + "/context/loop_info", 100, &mapOptimization::contextLoopInfoHandler, this, ros::TransportHints().tcpNoDelay());

        // 初始化 ROS 订阅者，订阅激光点云信息，并指定回调函数
        subCloud = nh.subscribe<disco_slam::cloud_info>(robot_id + "/disco_slam/feature/cloud_info", 1, &mapOptimization::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        // 初始化 ROS 订阅者，订阅 GPS 信息，并指定回调函数
        subGPS   = nh.subscribe<nav_msgs::Odometry> (gpsTopic, 200, &mapOptimization::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        // 初始化 ROS 订阅者，订阅回环检测信息，并指定回调函数
        subLoop  = nh.subscribe<std_msgs::Float64MultiArray>(robot_id + "/lio_loop/loop_closure_detection", 1, &mapOptimization::loopInfoHandler, this, ros::TransportHints().tcpNoDelay());

        // 初始化 ROS 发布者，用于发布历史关键帧点云，用于回环检测
        pubHistoryKeyFrames   = nh.advertise<sensor_msgs::PointCloud2>(robot_id + "/disco_slam/mapping/icp_loop_closure_history_cloud", 1);
        // 初始化 ROS 发布者，用于发布经过 ICP 校正后的关键帧点云
        pubIcpKeyFrames       = nh.advertise<sensor_msgs::PointCloud2>(robot_id + "/disco_slam/mapping/icp_loop_closure_corrected_cloud", 1);
        // 初始化 ROS 发布者，用于发布回环约束边的可视化标记
        pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>(robot_id + "/disco_slam/mapping/loop_closure_constraints", 1);

        // 初始化 ROS 发布者，用于发布最近的关键帧点云
        pubRecentKeyFrames    = nh.advertise<sensor_msgs::PointCloud2>(robot_id + "/disco_slam/mapping/map_local", 1);
        // 初始化 ROS 发布者，用于发布最近的单个关键帧点云
        pubRecentKeyFrame     = nh.advertise<sensor_msgs::PointCloud2>(robot_id + "/disco_slam/mapping/cloud_registered", 1);
        // 初始化 ROS 发布者，用于发布原始注册后的点云
        pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>(robot_id + "/disco_slam/mapping/cloud_registered_raw", 1);

        // 初始化 ROS 发布者，用于多机器人场景下发布激光点云信息
        pubLaserCloudInfo = nh.advertise<disco_slam::cloud_info> (robot_id + "/disco_slam/mapping/cloud_info", 1);

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

        // 为最后一帧角点特征点云分配内存
        laserCloudCornerLastFeature.reset(new pcl::PointCloud<PointType>());
        // 为最后一帧平面特征点云分配内存
        laserCloudSurfLastFeature.reset(new pcl::PointCloud<PointType>());

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
     * @brief 处理全局回环信息的回调函数，根据接收到的上下文信息添加回环约束并更新优化结果。
     * 
     * 该函数会检查接收到的消息中的机器人 ID 是否与当前机器人 ID 匹配，
     * 若匹配则提取回环的起始和结束索引、回环位姿和噪声信息，
     * 并将回环约束添加到 GTSAM 因子图中，然后更新 ISAM2 优化器，
     * 最后调用 `correctPoses` 和 `publishFrames` 函数进行位姿校正和帧发布。
     * 
     * @param msgIn 接收到的全局回环上下文信息的常量指针。
     */
    void contextLoopInfoHandler(const disco_slam::context_infoConstPtr& msgIn){
        // 注释掉的代码，原本用于直接返回，不处理全局回环信息
        //close global loop by do nothing
//        return;

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
    void laserCloudInfoHandler(const disco_slam::cloud_infoConstPtr& msgIn)
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
            updateInitialGuess();

            // 提取周围的关键帧，用于构建局部地图
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

    /**
     * @brief 可视化全局地图的线程函数，负责定期可视化全局地图，并在需要时将地图保存为 PCD 文件。
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

    void performLoopClosure()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        mtx.lock();
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mtx.unlock();

        // find keys
        int loopKeyCur;
        int loopKeyPre;
        if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false)
            if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
                return;

        // extract cloud
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);
            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
            if (pubHistoryKeyFrames.getNumSubscribers() != 0)
                publishCloud(&pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, robot_id + "/" + odometryFrame);
        }

        // ICP Settings
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius*2);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align clouds
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
            return;

        // publish corrected cloud
        if (pubIcpKeyFrames.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
            publishCloud(&pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, robot_id + "/" + odometryFrame);
        }

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();
        // transform from world origin to wrong pose
        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // transform from world origin to corrected pose
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
        pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
        gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore();
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

        // Add pose constraint
        mtx.lock();
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(constraintNoise);
        mtx.unlock();

        // add loop constriant
        loopIndexContainer[loopKeyCur] = loopKeyPre;
    }

    bool detectLoopClosureDistance(int *latestID, int *closestID)
    {
        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
        int loopKeyPre = -1;

        // check loop constraint added before
        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;

        // find the closest history key frame
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;
        kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
        kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
        
        for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
        {
            int id = pointSearchIndLoop[i];
            if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) > historyKeyframeSearchTimeDiff)
            {
                loopKeyPre = id;
                break;
            }
        }

        if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
            return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    bool detectLoopClosureExternal(int *latestID, int *closestID)
    {
        // this function is not used yet, please ignore it
        int loopKeyCur = -1;
        int loopKeyPre = -1;

        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if (loopInfoVec.empty())
            return false;

        double loopTimeCur = loopInfoVec.front().data[0];
        double loopTimePre = loopInfoVec.front().data[1];
        loopInfoVec.pop_front();

        if (abs(loopTimeCur - loopTimePre) < historyKeyframeSearchTimeDiff)
            return false;

        int cloudSize = copy_cloudKeyPoses6D->size();
        if (cloudSize < 2)
            return false;

        // latest key
        loopKeyCur = cloudSize - 1;
        for (int i = cloudSize - 1; i >= 0; --i)
        {
            if (copy_cloudKeyPoses6D->points[i].time >= loopTimeCur)
                loopKeyCur = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        // previous key
        loopKeyPre = 0;
        for (int i = 0; i < cloudSize; ++i)
        {
            if (copy_cloudKeyPoses6D->points[i].time <= loopTimePre)
                loopKeyPre = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        if (loopKeyCur == loopKeyPre)
            return false;

        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum)
    {
        // extract near keyframes
        nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize )
                continue;
            *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
            *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear],   &copy_cloudKeyPoses6D->points[keyNear]);
        }

        if (nearKeyframes->empty())
            return;

        // downsample near keyframes
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }

    void visualizeLoopClosure()
    {
        if (loopIndexContainer.empty())
            return;
        
        visualization_msgs::MarkerArray markerArray;
        // loop nodes
        visualization_msgs::Marker markerNode;
        markerNode.header.frame_id = robot_id + "/" + odometryFrame;
        markerNode.header.stamp = timeLaserInfoStamp;
        markerNode.action = visualization_msgs::Marker::ADD;
        markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
        markerNode.ns = "loop_nodes";
        markerNode.id = 0;
        markerNode.pose.orientation.w = 1;
        markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3; 
        markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
        markerNode.color.a = 1;
        // loop edges
        visualization_msgs::Marker markerEdge;
        markerEdge.header.frame_id = robot_id + "/" + odometryFrame;
        markerEdge.header.stamp = timeLaserInfoStamp;
        markerEdge.action = visualization_msgs::Marker::ADD;
        markerEdge.type = visualization_msgs::Marker::LINE_LIST;
        markerEdge.ns = "loop_edges";
        markerEdge.id = 1;
        markerEdge.pose.orientation.w = 1;
        markerEdge.scale.x = 0.1;
        markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
        markerEdge.color.a = 1;

        for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
        {
            int key_cur = it->first;
            int key_pre = it->second;
            geometry_msgs::Point p;
            p.x = copy_cloudKeyPoses6D->points[key_cur].x;
            p.y = copy_cloudKeyPoses6D->points[key_cur].y;
            p.z = copy_cloudKeyPoses6D->points[key_cur].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            p.x = copy_cloudKeyPoses6D->points[key_pre].x;
            p.y = copy_cloudKeyPoses6D->points[key_pre].y;
            p.z = copy_cloudKeyPoses6D->points[key_pre].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }

        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);
        pubLoopConstraintEdge.publish(markerArray);
    }

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
                // 计算增量变换矩阵
                Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
                // 获取当前待映射的变换矩阵
                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
                // 计算最终的变换矩阵
                Eigen::Affine3f transFinal = transTobe * transIncre;
                // 从最终的变换矩阵中提取平移和旋转信息
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

    void extractForLoopClosure()
    {
        pcl::PointCloud<PointType>::Ptr cloudToExtract(new pcl::PointCloud<PointType>());
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            if ((int)cloudToExtract->size() <= surroundingKeyframeSize)
                cloudToExtract->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(cloudToExtract);
    }

    void extractNearby()
    {
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // extract all the nearby key poses and downsample them
        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
        kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
        for (int i = 0; i < (int)pointSearchInd.size(); ++i)
        {
            int id = pointSearchInd[i];
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
        }

        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);

        // also extract some latest key frames in case the robot rotates in one position
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
                surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(surroundingKeyPosesDS);
    }

    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
    {
        // fuse the map
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear(); 
        for (int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
                continue;

            int thisKeyInd = (int)cloudToExtract->points[i].intensity;
            if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end()) 
            {
                // transformed cloud available
                *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
                *laserCloudSurfFromMap   += laserCloudMapContainer[thisKeyInd].second;
            } else {
                // transformed cloud not available
                pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
                pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
                *laserCloudCornerFromMap += laserCloudCornerTemp;
                *laserCloudSurfFromMap   += laserCloudSurfTemp;
                laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
            
        }

        // Downsample the surrounding corner key frames (or map)
        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
        // Downsample the surrounding surf key frames (or map)
        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

        // clear map cache if too large
        if (laserCloudMapContainer.size() > 1000)
            laserCloudMapContainer.clear();
    }

    void extractSurroundingKeyFrames()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return; 
        
        // if (loopClosureEnableFlag == true)
        // {
        //     extractForLoopClosure();    
        // } else {
        //     extractNearby();
        // }

        extractNearby();
    }

    void downsampleCurrentScan()
    {
        // Downsample cloud from current scan
        laserCloudCornerLastDS->clear();
        laserCloudCornerLastFeature->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);
        pcl::copyPointCloud(*laserCloudCornerLast,  *laserCloudCornerLastFeature);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();


        laserCloudSurfLastDS->clear();
        laserCloudSurfLastFeature->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        pcl::copyPointCloud(*laserCloudSurfLast,  *laserCloudSurfLastFeature);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
    }

    void updatePointAssociateToMap()
    {
        transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
    }

    void cornerOptimization()
    {
        updatePointAssociateToMap();

        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudCornerLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudCornerLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));
                    
            if (pointSearchSqDis[4] < 1.0) {
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++) {
                    cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                }
                cx /= 5; cy /= 5;  cz /= 5;

                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++) {
                    float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                    float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                    float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                    a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
                    a22 += ay * ay; a23 += ay * az;
                    a33 += az * az;
                }
                a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

                matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;

                cv::eigen(matA1, matD1, matV1);

                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;
                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                    float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                                    + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                                    + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)) * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                    float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                    float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                              + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                    float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                               - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                               + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float ld2 = a012 / l12;

                    float s = 1 - 0.9 * fabs(ld2);

                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;

                    if (s > 0.1) {
                        laserCloudOriCornerVec[i] = pointOri;
                        coeffSelCornerVec[i] = coeff;
                        laserCloudOriCornerFlag[i] = true;
                    }
                }
            }
        }
    }

    void surfOptimization()
    {
        updatePointAssociateToMap();

        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudSurfLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudSurfLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel); 
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;

            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            if (pointSearchSqDis[4] < 1.0) {
                for (int j = 0; j < 5; j++) {
                    matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }

                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps; pb /= ps; pc /= ps; pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid) {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                            + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.1) {
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
    }

    void combineOptimizationCoeffs()
    {
        // combine corner coeffs
        for (int i = 0; i < laserCloudCornerLastDSNum; ++i){
            if (laserCloudOriCornerFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriCornerVec[i]);
                coeffSel->push_back(coeffSelCornerVec[i]);
            }
        }
        // combine surf coeffs
        for (int i = 0; i < laserCloudSurfLastDSNum; ++i){
            if (laserCloudOriSurfFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }
        // reset flag for next iteration
        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    }

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

        // lidar -> camera
        float srx = sin(transformTobeMapped[1]);
        float crx = cos(transformTobeMapped[1]);
        float sry = sin(transformTobeMapped[2]);
        float cry = cos(transformTobeMapped[2]);
        float srz = sin(transformTobeMapped[0]);
        float crz = cos(transformTobeMapped[0]);

        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < 50) {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++) {
            // lidar -> camera
            pointOri.x = laserCloudOri->points[i].y;
            pointOri.y = laserCloudOri->points[i].z;
            pointOri.z = laserCloudOri->points[i].x;
            // lidar -> camera
            coeff.x = coeffSel->points[i].y;
            coeff.y = coeffSel->points[i].z;
            coeff.z = coeffSel->points[i].x;
            coeff.intensity = coeffSel->points[i].intensity;
            // in camera
            float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                      + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                      + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

            float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                      + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                      + ((-cry*crz - srx*sry*srz)*pointOri.x 
                      + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

            float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                      + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                      + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
            // lidar -> camera
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = arx;
            matA.at<float>(i, 2) = ary;
            matA.at<float>(i, 3) = coeff.z;
            matA.at<float>(i, 4) = coeff.x;
            matA.at<float>(i, 5) = coeff.y;
            matB.at<float>(i, 0) = -coeff.intensity;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {

            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
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
            matP = matV.inv() * matV2;
        }

        if (isDegenerate)
        {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR = sqrt(
                            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.05 && deltaT < 0.05) {
            return true; // converged
        }
        return false; // keep optimizing
    }

    void scan2MapOptimization()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum)
        {
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            for (int iterCount = 0; iterCount < 30; iterCount++)
            {
                laserCloudOri->clear();
                coeffSel->clear();

                cornerOptimization();
                surfOptimization();

                combineOptimizationCoeffs();

                if (LMOptimization(iterCount) == true)
                    break;              
            }

            transformUpdate();
        } else {
            ROS_WARN("Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
        }
    }

    void transformUpdate()
    {
        if (cloudInfo.imuAvailable == true)
        {
            if (std::abs(cloudInfo.imuPitchInit) < 1.4)
            {
                double imuWeight = imuRPYWeight;
                tf::Quaternion imuQuaternion;
                tf::Quaternion transformQuaternion;
                double rollMid, pitchMid, yawMid;

                // slerp roll
                transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
                imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[0] = rollMid;

                // slerp pitch
                transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
                imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[1] = pitchMid;
            }
        }

        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);

        incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
    }

    float constraintTransformation(float value, float limit)
    {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }

    bool saveFrame()
    {
        if (cloudKeyPoses3D->points.empty())
            return true;

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
            abs(pitch) < surroundingkeyframeAddingAngleThreshold && 
            abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
            sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
            return false;

        return true;
    }

    void addOdomFactor()
    {
        if (cloudKeyPoses3D->points.empty())
        {
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));

        }else{
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);

        }
    }

    void addGPSFactor()
    {
        if (gpsQueue.empty())
            return;

        // wait for system initialized and settles down
        if (cloudKeyPoses3D->points.empty())
            return;
        else
        {
            if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
                return;
        }

        // pose covariance small, no need to correct
        if (poseCovariance(3,3) < poseCovThreshold && poseCovariance(4,4) < poseCovThreshold)
            return;

        // last gps position
        static PointType lastGPSPoint;

        while (!gpsQueue.empty())
        {
            if (gpsQueue.front().header.stamp.toSec() < timeLaserInfoCur - 0.2)
            {
                // message too old
                gpsQueue.pop_front();
            }
            else if (gpsQueue.front().header.stamp.toSec() > timeLaserInfoCur + 0.2)
            {
                // message too new
                break;
            }
            else
            {
                nav_msgs::Odometry thisGPS = gpsQueue.front();
                gpsQueue.pop_front();

                // GPS too noisy, skip
                float noise_x = thisGPS.pose.covariance[0];
                float noise_y = thisGPS.pose.covariance[7];
                float noise_z = thisGPS.pose.covariance[14];
                if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                    continue;

                float gps_x = thisGPS.pose.pose.position.x;
                float gps_y = thisGPS.pose.pose.position.y;
                float gps_z = thisGPS.pose.pose.position.z;
                if (!useGpsElevation)
                {
                    gps_z = transformTobeMapped[5];
                    noise_z = 0.01;
                }

                // GPS not properly initialized (0,0,0)
                if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                    continue;

                // Add GPS every a few meters
                PointType curGPSPoint;
                curGPSPoint.x = gps_x;
                curGPSPoint.y = gps_y;
                curGPSPoint.z = gps_z;
                if (pointDistance(curGPSPoint, lastGPSPoint) < 5.0)
                    continue;
                else
                    lastGPSPoint = curGPSPoint;

                gtsam::Vector Vector3(3);
                Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
                noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
                gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
                gtSAMgraph.add(gps_factor);

                aLoopIsClosed = true;
                break;
            }
        }
    }

    void addLoopFactor()
    {
        if (loopIndexQueue.empty())
            return;

        for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
        {
            int indexFrom = loopIndexQueue[i].first;
            int indexTo = loopIndexQueue[i].second;
            gtsam::Pose3 poseBetween = loopPoseQueue[i];
            gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
            gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
        }

        loopIndexQueue.clear();
        loopPoseQueue.clear();
        loopNoiseQueue.clear();
        aLoopIsClosed = true;
    }

    void saveKeyFramesAndFactor()
    {
        if (saveFrame() == false)
            return;

        // odom factor
        addOdomFactor();

        // gps factor
        addGPSFactor();

        // loop factor
        addLoopFactor();

        // cout << "****************************************************" << endl;
        // gtSAMgraph.print("GTSAM Graph:\n");

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        if (aLoopIsClosed == true)
        {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }


        gtSAMgraph.resize(0);
        initialEstimate.clear();

        //save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();

        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
        // cout << "****************************************************" << endl;
        // isamCurrentEstimate.print("Current estimate: ");

        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
        thisPose6D.roll  = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw   = latestEstimate.rotation().yaw();
        thisPose6D.time = timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);

        // cout << "****************************************************" << endl;
        // cout << "Pose covariance:" << endl;
        // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);

        // save updated transform
        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();

        // save all the received edge and surf points
        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*laserCloudCornerLastDS,  *thisCornerKeyFrame);
        pcl::copyPointCloud(*laserCloudSurfLastDS,    *thisSurfKeyFrame);

        // save key frame cloud
        cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

        // save path for visualization
        updatePath(thisPose6D);

        cloudInfo.initialGuessX = thisPose6D.x;
        cloudInfo.initialGuessY = thisPose6D.y;
        cloudInfo.initialGuessZ = thisPose6D.z;
        cloudInfo.initialGuessRoll  = thisPose6D.roll;
        cloudInfo.initialGuessPitch = thisPose6D.pitch;
        cloudInfo.initialGuessYaw   = thisPose6D.yaw;
        cloudInfo.imuAvailable = cloudKeyPoses6D->size() - 1;

        cloudInfo.cloud_corner = publishCloud(&pubFeatureCloud,  laserCloudCornerLastFeature,  cloudInfo.header.stamp, robot_id + "/" + lidarFrame);
        cloudInfo.cloud_surface = publishCloud(&pubFeatureCloud,  laserCloudSurfLastFeature,  cloudInfo.header.stamp, robot_id + "/" + lidarFrame);;
        pubLaserCloudInfo.publish(cloudInfo);




    }

    void correctPoses()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (aLoopIsClosed == true)
        {
            // clear map cache
            laserCloudMapContainer.clear();
            // clear path
            globalPath.poses.clear();
            // update key poses
            int numPoses = isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i)
            {
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

                updatePath(cloudKeyPoses6D->points[i]);
            }

            aLoopIsClosed = false;
        }
    }

    void updatePath(const PointTypePose& pose_in)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
        pose_stamped.header.frame_id = robot_id + "/" + odometryFrame;
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }

    void publishOdometry()
    {
        // Publish odometry for ROS (global)
        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = robot_id + "/" + odometryFrame;
        laserOdometryROS.child_frame_id = robot_id + "/" + lidarFrame + "/odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        pubLaserOdometryGlobal.publish(laserOdometryROS);

        //publish to fusion node

//        pubCloudInfoWithPose.publish();

        // Publish TF
        static tf::TransformBroadcaster br;
        tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
                                                      tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, timeLaserInfoStamp, robot_id + "/" + odometryFrame, robot_id + "/" + lidarFrame + "/lidar_link");
        br.sendTransform(trans_odom_to_lidar);

        // Publish odometry for ROS (incremental)
        static bool lastIncreOdomPubFlag = false;
        static nav_msgs::Odometry laserOdomIncremental; // incremental odometry msg
        static Eigen::Affine3f increOdomAffine; // incremental odometry in affine
        if (lastIncreOdomPubFlag == false)
        {
            lastIncreOdomPubFlag = true;
            laserOdomIncremental = laserOdometryROS;
            increOdomAffine = trans2Affine3f(transformTobeMapped);
        } else {
            Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            increOdomAffine = increOdomAffine * affineIncre;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles (increOdomAffine, x, y, z, roll, pitch, yaw);
            if (cloudInfo.imuAvailable == true)
            {
                if (std::abs(cloudInfo.imuPitchInit) < 1.4)
                {
                    double imuWeight = 0.1;
                    tf::Quaternion imuQuaternion;
                    tf::Quaternion transformQuaternion;
                    double rollMid, pitchMid, yawMid;

                    // slerp roll
                    transformQuaternion.setRPY(roll, 0, 0);
                    imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    roll = rollMid;

                    // slerp pitch
                    transformQuaternion.setRPY(0, pitch, 0);
                    imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    pitch = pitchMid;
                }
            }
            laserOdomIncremental.header.stamp = timeLaserInfoStamp;
            laserOdomIncremental.header.frame_id = robot_id + "/" + odometryFrame;
            laserOdomIncremental.child_frame_id = robot_id + "/" + lidarFrame + "/odom_mapping";
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            laserOdomIncremental.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
            if (isDegenerate)
                laserOdomIncremental.pose.covariance[0] = 1;
            else
                laserOdomIncremental.pose.covariance[0] = 0;
        }
        pubLaserOdometryIncremental.publish(laserOdomIncremental);
    }

    void publishFrames()
    {
        if (cloudKeyPoses3D->points.empty())
            return;
        // publish key poses
        publishCloud(&pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, robot_id + "/" + odometryFrame);
        // Publish surrounding key frames
        publishCloud(&pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, robot_id + "/" + odometryFrame);
        // publish registered key frame
        if (pubRecentKeyFrame.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut += *transformPointCloud(laserCloudCornerLastDS,  &thisPose6D);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS,    &thisPose6D);
            publishCloud(&pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, robot_id + "/" + odometryFrame);
        }
        // publish registered high-res raw cloud
        if (pubCloudRegisteredRaw.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
            publishCloud(&pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, robot_id + "/" + odometryFrame);
        }
        // publish path
        if (pubPath.getNumSubscribers() != 0)
        {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = robot_id + "/" + odometryFrame;
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
    // 初始化 ROS 节点，节点名为 "disco_slam"
    ros::init(argc, argv, "disco_slam");

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
