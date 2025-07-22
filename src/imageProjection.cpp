// 数据预处理模块
#include "utility.h"
// #include "disco_double/cloud_info.h"
#include "disco_double/ring_cloud_info.h"
// Velodyne激光雷达点结构定义，包含XYZI(坐标和强度)、ring(线号)和time(时间戳)
struct VelodynePointXYZIRT
{
    // 添加 x,y,z 坐标和一个填充字段
    PCL_ADD_POINT4D;
    // 添加反射强度
    PCL_ADD_INTENSITY;
    uint16_t ring;  // 激光雷达线束编号
    float time;     // 点时间戳(相对于扫描起始时间)
    // 是 Eigen 库中的一个宏定义，用于确保类的内存对齐。
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// 注册点云结构体到PCL中，使PCL能够识别和处理该类型的点云
// 参数1: VelodynePointXYZIRT - 要注册的点云结构体名称
// 参数2: 结构体中各字段的定义，格式为(字段类型, 字段名, 字段名)
//   - (float, x, x): x坐标，类型为float
//   - (float, y, y): y坐标，类型为float
//   - (float, z, z): z坐标，类型为float
//   - (float, intensity, intensity): 点云强度值
//   - (uint16_t, ring, ring): 激光雷达线束编号
//   - (float, time, time): 点的时间戳
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)
// Ouster激光雷达点结构定义，包含更多属性
struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;    // 点反射强度
    uint32_t t;         // 时间戳(ns)
    uint16_t reflectivity; // 反射率
    uint8_t ring;       // 线束编号
    uint16_t noise;     // 噪声水平
    uint32_t range;     // 测距值(mm)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)

// 使用Velodyne点格式作为通用表示
using PointXYZIRT = VelodynePointXYZIRT;
// 数据队列最大长度
const int queueLength = 2000;
// 图像投影处理类
class ImageProjection : public ParamServer
{
private:
    // 互斥锁保护IMU和里程计数据
    std::mutex imuLock;
    std::mutex odoLock;
    // ROS订阅器和发布器
    ros::Subscriber subLaserCloud;  // 激光雷达点云订阅
    ros::Publisher  pubLaserCloud;  // 激光雷达点云发布
    
    ros::Publisher pubExtractedCloud;  // 提取点云发布
    ros::Publisher pubLaserCloudInfo;  // 点云信息发布
    // IMU数据队列
    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    // 里程计数据队列
    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    // 点云数据队列
    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    sensor_msgs::PointCloud2 currentCloudMsg;
    // IMU数据缓存数组
    double *imuTime = new double[queueLength];  // 时间戳
    double *imuRotX = new double[queueLength];  // X轴旋转
    double *imuRotY = new double[queueLength];  // Y轴旋转
    double *imuRotZ = new double[queueLength];  // Z轴旋转

    // 当前IMU数据指针
    int imuPointerCur;
    bool firstPointFlag;  // 是否是第一个点标志
    Eigen::Affine3f transStartInverse;  // 起始变换的逆矩阵
    // 点云容器
    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;  // 输入点云
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;  // Ouster格式临时点云
    pcl::PointCloud<PointType>::Ptr   fullCloud;     // 完整点云
    pcl::PointCloud<PointType>::Ptr   extractedCloud; // 提取的点云

    // 去畸变标志
    int deskewFlag;
    cv::Mat rangeMat;  // 距离图像矩阵

    // 里程计去畸变相关
    bool odomDeskewFlag;
    float odomIncreX;  // X轴增量
    float odomIncreY;  // Y轴增量
    float odomIncreZ;  // Z轴增量
    // 点云信息
    // disco_double::cloud_info cloudInfo;
    disco_double::ring_cloud_info cloudInfo;
    double timeScanCur;  // 当前扫描起始时间
    double timeScanEnd;  // 当前扫描结束时间
    std_msgs::Header cloudHeader;  // 点云头信息

    vector<int> columnIdnCountVec;  // 列索引和点数量向量

public:
    // 构造函数，初始化ROS订阅器和发布器
    ImageProjection():
    deskewFlag(0)
    {
        // 初始化订阅器和发布器
        // 订阅IMU话题
        // 参数1: robot_id + "/" + imuTopic - 话题名称，由机器人ID和IMU话题名组成
        // 参数2: 2000 - 消息队列长度，可缓存2000条消息
        // 参数3: &ImageProjection::imuHandler - 回调函数指针，处理接收到的IMU消息
        // 参数4: this - 当前对象指针，用于在回调函数中访问类成员
        // 参数5: ros::TransportHints().tcpNoDelay() - 使用TCP NODELAY选项，减少通信延迟
        subImu = nh.subscribe<sensor_msgs::Imu>(robot_id + "/" + imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
        
        // 订阅里程计增量话题
        // 注意话题名后缀"_incremental"表示增量数据，用于里程计去畸变
        // 其他参数含义同上
        subOdom = nh.subscribe<nav_msgs::Odometry>(robot_id + "/" + odomTopic+"_incremental", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        // subOdom = nh.subscribe<nav_msgs::Odometry>(robot_id + "/" + "disco_double/mapping/odometry"+"_incremental", 5, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        
        // 订阅激光雷达点云话题
        // 参数2: 5 - 队列长度较小，因为点云数据量大，避免占用过多内存
        // 消息类型为sensor_msgs::PointCloud2，包含原始点云数据
        // 其他参数含义同上
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(robot_id + "/" + pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());

        // 创建点云发布器
        // 参数1: robot_id + "/disco_double/deskew/cloud_deskewed" - 发布话题名称，用于发布去畸变后的点云
        // 参数2: 1 - 消息队列长度，只保留最新的1条消息
        // 消息类型为sensor_msgs::PointCloud2，标准ROS点云消息格式
        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> (robot_id + "/disco_double/deskew/cloud_deskewed", 1);
        
        // 创建点云信息发布器，将进入下一个文件和流程
        // 参数1: robot_id + "/disco_double/deskew/cloud_info" - 发布话题名称，用于发布点云的附加信息
        // 参数2: 1 - 消息队列长度，只保留最新的1条消息
        // 消息类型为自定义的disco_double::cloud_info，包含点云的结构化信息
        // pubLaserCloudInfo = nh.advertise<disco_double::cloud_info> (robot_id + "/disco_double/deskew/cloud_info", 1);
        pubLaserCloudInfo = nh.advertise<disco_double::ring_cloud_info> (robot_id + "/disco_double/deskew/cloud_info", 1);

        // 分配内存并初始化参数
        allocateMemory();
        resetParameters();
        // 设置PCL日志级别为错误，减少不必要的输出
        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }
    /*
        * 内存分配函数：为点云处理分配和初始化必要的内存空间
        */
    void allocateMemory()
    {
        // 为各种点云容器分配内存并初始化智能指针
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());         // 输入激光点云
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>()); // Ouster临时点云
        fullCloud.reset(new pcl::PointCloud<PointType>());              // 完整点云（用于图像投影）
        extractedCloud.reset(new pcl::PointCloud<PointType>());         // 提取的有效点云
    
        // 预分配完整点云的空间
        // N_SCAN: 激光雷达垂直线束数量
        // Horizon_SCAN: 水平方向的点数
        fullCloud->points.resize(N_SCAN*Horizon_SCAN);
    
        // 初始化点云信息结构体中的数组
        cloudInfo.startRingIndex.assign(N_SCAN, 0);        // 每个线束起始点的索引
        cloudInfo.endRingIndex.assign(N_SCAN, 0);          // 每个线束结束点的索引
    
        // 为点云的列索引和距离信息分配空间
        cloudInfo.pointColInd.assign(N_SCAN*Horizon_SCAN, 0);  // 点的列索引
        cloudInfo.pointRange.assign(N_SCAN*Horizon_SCAN, 0);   // 点到激光雷达的距离
    
        // 重置其他参数
        resetParameters();
    }
    /*
     * 重置参数函数：在每次点云处理完成后重置所有状态变量
     */
    void resetParameters()
    {
        // 清空点云容器
        laserCloudIn->clear();      // 清空输入点云
        extractedCloud->clear();    // 清空提取的点云
        
        // 重置距离图像矩阵，初始化为最大浮点值
        // N_SCAN: 激光雷达垂直线束数量
        // Horizon_SCAN: 水平方向的点数
        // CV_32F: 32位浮点类型
        // FLT_MAX: 表示无效或未投影的点
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        // 重置IMU数据处理相关变量
        imuPointerCur = 0;          // 当前IMU数据指针归零
        firstPointFlag = true;      // 重置第一个点标志
        odomDeskewFlag = false;     // 重置里程计去畸变标志

        // 清空IMU数据缓存数组
        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;         // 清空时间戳
            imuRotX[i] = 0;         // 清空X轴旋转
            imuRotY[i] = 0;         // 清空Y轴旋转
            imuRotZ[i] = 0;         // 清空Z轴旋转
        }

        columnIdnCountVec.assign(N_SCAN, 0);
    }

    ~ImageProjection(){}
    // IMU数据处理回调函数
    // 重点三大线程之一
    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {
        sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);

        // debug IMU data
        // cout << std::setprecision(6);
        // cout << "IMU acc: " << endl;
        // cout << "x: " << thisImu.linear_acceleration.x << 
        //       ", y: " << thisImu.linear_acceleration.y << 
        //       ", z: " << thisImu.linear_acceleration.z << endl;
        // cout << "IMU gyro: " << endl;
        // cout << "x: " << thisImu.angular_velocity.x << 
        //       ", y: " << thisImu.angular_velocity.y << 
        //       ", z: " << thisImu.angular_velocity.z << endl;
        // double imuRoll, imuPitch, imuYaw;
        // tf::Quaternion orientation;
        // tf::quaternionMsgToTF(thisImu.orientation, orientation);
        // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
        // cout << "IMU roll pitch yaw: " << endl;
        // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
    }
    // 里程计数据处理回调函数
    // 重点三大线程之一
    // 从上一次里程计位置开始，计算当前里程计位置与上一次里程计位置之间的增量
    // 来自mapOptmization.cpp
    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }
    /*
     * 点云数据处理主函数：处理每一帧激光雷达点云数据
     * 主要完成点云的缓存、去畸变、投影、提取和发布等处理流程
     * @param laserCloudMsg 输入的原始点云消息
     */
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        // 缓存点云数据并进行格式转换
        // 如果点云队列不足2帧（最后一帧）或格式转换失败则返回
        if (!cachePointCloud(laserCloudMsg))
            return;

        // 获取点云去畸变所需的IMU和里程计信息
        // 如果IMU或里程计数据不足以覆盖当前点云的时间段则返回
        if (!deskewInfo())
            return;

        // 将点云投影到距离图像上
        // 包括点云去畸变和球面投影
        // 去畸变，将点云从返回值的点 和IMU旋转向量相乘 转换到发出时的点
        projectPointCloud();

        // 从距离图像中提取有效点云
        // 去除无效点并记录点云结构信息
        cloudExtraction();

        // 发布处理后的点云和相关信息
        // 包括去畸变后的点云和点云信息
        publishClouds();

        // 重置处理参数，为下一帧做准备
        resetParameters();
    }
    // 缓存点云数据
    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        // 将点云加入队列
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2)
            return false;
        // 从队列中取出最早的点云（滑动窗口的处理方法）
        // 一帧点云的数据非常多，所以只取最早的一帧
        currentCloudMsg = std::move(cloudQueue.front());
        // 从队列中移除最早的点云
        cloudQueue.pop_front();
        // 根据传感器类型转换点云格式
        if (sensor == SensorType::VELODYNE)
        {
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
            // ring问题的修正
            // indices 是一个索引向量，用于存储保留的点的索引
            std::vector<int> indices;
            // 通过去除无效点获取索引值，（原始点云，目标点云，索引值）cloud_in->is_dense = false;才能使用
            // pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
            // ring问题的修正end
        }
        else if (sensor == SensorType::OUSTER)
        {
            // 将Ouster格式转换为Velodyne格式
            pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
            laserCloudIn->points.resize(tmpOusterCloudIn->size());
            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
            {
                auto &src = tmpOusterCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.t * 1e-9f;
            }
        }
        else
        {
            ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
            ros::shutdown();
        }

        // 获取时间戳
        cloudHeader = currentCloudMsg.header;
        timeScanCur = cloudHeader.stamp.toSec();
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

        // 检查点云是否是dense格式
        if (laserCloudIn->is_dense == false)
        {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }

        // ring问题的修正
        if (!has_ring)
            return true;
        // ring问题的修正end

        // 检查点云是否包含ring通道
        static int ringFlag = 0;
        if (ringFlag == 0)
        {
            ringFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == "ring")
                {
                    ringFlag = 1;
                    break;
                }
            }
            if (ringFlag == -1)
            {
                ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                ros::shutdown();
            }
        }

        // 检查点云是否包含时间戳
        if (deskewFlag == 0)
        {
            deskewFlag = -1;
            for (auto &field : currentCloudMsg.fields)
            {
                if (field.name == "time" || field.name == "t")
                {
                    deskewFlag = 1;
                    break;
                }
            }
            if (deskewFlag == -1)
                ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        }

        return true;
    }
    /*
     * 获取点云去畸变所需的IMU和里程计信息
     * @return 如果获取信息成功返回true，否则返回false
     */
    bool deskewInfo()
    {
        // 使用互斥锁保护IMU和里程计数据的访问
        // 防止在处理过程中数据被其他线程修改
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        // 检查IMU数据是否完整覆盖当前点云的扫描时间段
        // 条件1: IMU队列不为空
        // 条件2: IMU起始时间早于点云扫描起始时间
        // 条件3: IMU结束时间晚于点云扫描结束时间
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanEnd)
        {
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }
        // 处理IMU数据，计算点云去畸变所需的旋转信息
        imuDeskewInfo();
        // 处理里程计数据，计算点云去畸变所需的位置信息
        odomDeskewInfo();

        return true;
    }
    // 从IMU数据获取去畸变信息
    void imuDeskewInfo()
    {
        cloudInfo.imuAvailable = false;
        // 移除过期的IMU数据
        while (!imuQueue.empty())
        {
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;
        // 处理当前扫描时间段内的IMU数据
        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = thisImuMsg.header.stamp.toSec();

            // get roll, pitch, and yaw estimation for this scan
            if (currentImuTime <= timeScanCur)
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);

            if (currentImuTime > timeScanEnd + 0.01)
                break;

            if (imuPointerCur == 0){
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // 获取角速度
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // 积分计算旋转量
            double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;

        if (imuPointerCur <= 0)
            return;

        cloudInfo.imuAvailable = true;
    }
    // 从里程计数据获取去畸变信息
    void odomDeskewInfo()
    {
        cloudInfo.odomAvailable = false;
        // 移除过期的里程计数据
        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return;

        // 获取扫描起始时刻的里程计数据
        nav_msgs::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }
        // 获取起始姿态
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // 设置初始猜测值(用于后续优化)
        cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
        cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
        cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.initialGuessRoll  = roll;
        cloudInfo.initialGuessPitch = pitch;
        cloudInfo.initialGuessYaw   = yaw;

        cloudInfo.odomAvailable = true;

        // 获取扫描结束时刻的里程计数据
        odomDeskewFlag = false;

        if (odomQueue.back().header.stamp.toSec() < timeScanEnd)
            return;

        nav_msgs::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (ROS_TIME(&endOdomMsg) < timeScanEnd)
                continue;
            else
                break;
        }

        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;
        // 计算起始和结束时刻的变换矩阵
        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);
        // 计算相对变换
        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;
        // 提取变换中的平移和旋转增量
        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }
    /*
     * 根据时间戳查找对应的旋转量(插值计算)
     * @param pointTime 当前点的时间戳
     * @param rotXCur 输出X轴旋转量
     * @param rotYCur 输出Y轴旋转量 
     * @param rotZCur 输出Z轴旋转量
     */
    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

        // 查找时间戳对应的IMU数据索引
        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        // 插值计算旋转量
        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            // 直接使用最近的IMU数据
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        } else {
            // 线性插值计算旋转量
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    /*
     * 根据时间戳查找对应的位置(暂未实现)
     * @param relTime 相对时间
     * @param posXCur 输出X轴位置
     * @param posYCur 输出Y轴位置
     * @param posZCur 输出Z轴位置
     */
    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0; *posYCur = 0; *posZCur = 0;

        // 注释说明：对于低速运动(如步行速度)，位置去畸变效果不明显，因此以下代码被注释
        // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
        //     return;

        // float ratio = relTime / (timeScanEnd - timeScanCur);

        // *posXCur = ratio * odomIncreX;
        // *posYCur = ratio * odomIncreY;
        // *posZCur = ratio * odomIncreZ;
    }

    /*
     * 点云去畸变处理
     * @param point 输入点指针
     * @param relTime 相对时间
     * @return 去畸变后的点
     */
    PointType deskewPoint(PointType *point, double relTime)
    {
        // ROS_INFO("IMU_deskewPoint init");
        // 检查去畸变是否可用
        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
            return *point;

        // 计算当前点的绝对时间
        double pointTime = timeScanCur + relTime;

        // 获取当前点的旋转量
        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        // 获取当前点的位置(暂未实现)
        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        // 如果是第一个点，计算初始变换的逆矩阵
        if (firstPointFlag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        // 计算当前点的变换矩阵
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        // 计算相对于起始点的变换
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        // 应用变换到当前点
        PointType newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        newPoint.intensity = point->intensity;

        // ROS_INFO("Processing scan %d, position: x=%.2f, y=%.2f", scan_id, x, y);
        // ROS_INFO("IMU_deskewPoint success");
        return newPoint;
    }

    /*
     * 点云投影到距离图像
     */
    void projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size();

        // ring问题的修正
        // 用于标记扫描是否过半的标志
        bool halfPassed = false;
        // 计算扫描起始点的水平方向角度，使用反正切函数计算y/x的角度，取负号调整方向
        cloudInfo.startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
        // 计算扫描结束点的水平方向角度，同样使用反正切函数计算y/x的角度，取负号调整方向
        // 加上2 * M_PI确保角度在合理范围内
        cloudInfo.endOrientation   = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,
                                                     laserCloudIn->points[laserCloudIn->points.size() - 1].x) + 2 * M_PI;
        // 检查结束角度与起始角度的差值是否过大（超过3 * M_PI）
        // 如果过大，说明角度计算可能绕了一圈，减去2 * M_PI进行修正
        if (cloudInfo.endOrientation - cloudInfo.startOrientation > 3 * M_PI) {
            cloudInfo.endOrientation -= 2 * M_PI;
        } 
        // 检查结束角度与起始角度的差值是否过小（小于M_PI）
        // 如果过小，说明角度计算可能绕了一圈，加上2 * M_PI进行修正
        else if (cloudInfo.endOrientation - cloudInfo.startOrientation < M_PI)
            cloudInfo.endOrientation += 2 * M_PI;
        // 计算扫描结束角度与起始角度的差值，得到整个扫描过程的角度变化量
        cloudInfo.orientationDiff = cloudInfo.endOrientation - cloudInfo.startOrientation;
        // ring问题的修正end
        
        // 遍历所有点进行投影
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            // 计算点到原点的距离并检查范围
            float range = pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;

            // ring问题的修正（注释）
            // 获取点所在的线束ID
            // int rowIdn = laserCloudIn->points[i].ring;
            // ring问题的修正（注释）end

            // ring问题的修正
            // 初始化线束编号为 -1，表示尚未确定
            int rowIdn = -1;
            // 检查点云数据中是否包含 ring 通道
            if (has_ring == true){
                // 如果包含 ring 通道，直接从点云数据中获取线束编号
                rowIdn = laserCloudIn->points[i].ring;
            }
            else{
                // 若不包含 ring 通道，需要手动计算线束编号
                // 定义垂直角度和水平角度变量
                float verticalAngle, horizonAngle;
                // 计算当前点的垂直角度，使用反正切函数计算 z 坐标与 xy 平面投影距离的比值
                // 并将结果转换为角度值
                verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x 
                                    + thisPoint.y * thisPoint.y)) * 180 / M_PI;
                // 根据垂直角度、雷达数据顶部的垂直角度和垂直角度分辨率计算线束编号
                rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
            }
            // ring问题的修正end
            
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            // 降采样处理
            if (rowIdn % downsampleRate != 0)
                continue;

            int columnIdn = -1;
            // 计算水平角度
            float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            // 计算列ID
            static float ang_res_x = 360.0/float(Horizon_SCAN);
            columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            // 检查该位置是否已有投影点
            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            // 去畸变处理(ring问题可注释)
            // thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

            // ring问题修正
            // 检查点云数据是否包含 ring 通道
            if (has_ring == true)
                // 若包含 ring 通道，直接使用点云数据中的时间戳进行点云去畸变处理
                thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);
            else {
                // 若不包含 ring 通道，需要手动计算相对时间
                // 计算当前点的水平方向角度，使用反正切函数计算 y/x 的角度并取负号
                float ori = -atan2(thisPoint.y, thisPoint.x);
                // 检查扫描是否过半
                if (!halfPassed) {
                    // 若扫描未过半，对角度进行修正，确保角度在合理范围内
                    if (ori < cloudInfo.startOrientation - M_PI / 2) {
                        // 若角度过小，加上 2 * M_PI 进行修正
                        ori += 2 * M_PI;
                    } else if (ori > cloudInfo.startOrientation + M_PI * 3 / 2) {
                        // 若角度过大，减去 2 * M_PI 进行修正
                        ori -= 2 * M_PI;
                    }
                    // 检查当前角度与起始角度的差值是否超过 π，若超过则标记扫描过半
                    if (ori - cloudInfo.startOrientation > M_PI) {
                        halfPassed = true;
                    }
                } else {
                    // 若扫描已过半，先给角度加上 2 * M_PI
                    ori += 2 * M_PI;
                    // 对角度进行修正，确保角度在合理范围内
                    if (ori < cloudInfo.endOrientation - M_PI * 3 / 2) {
                        // 若角度过小，加上 2 * M_PI 进行修正
                        ori += 2 * M_PI;
                    } else if (ori > cloudInfo.endOrientation + M_PI / 2) {
                        // 若角度过大，减去 2 * M_PI 进行修正
                        ori -= 2 * M_PI;
                    }
                }
                // 计算当前点相对于扫描起始点的相对时间比例
                float relTime = (ori - cloudInfo.startOrientation) / cloudInfo.orientationDiff;
                // 激光雷达频率为 10Hz，周期为 0.1 秒，根据相对时间比例计算当前点的时间戳
                laserCloudIn->points[i].time = 0.1 * relTime;
                // 使用计算得到的时间戳进行点云去畸变处理
                thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);
            }
            // ring问题修正end

            // 更新距离图像
            rangeMat.at<float>(rowIdn, columnIdn) = range;

            // 保存到完整点云，(当前行 - 1) * 列数 + 当前列
            int index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
        }
    }

    /*
     * 从距离图像提取有效点云
     */
    void cloudExtraction()
    {
        int count = 0;
        // 遍历所有线束
        for (int i = 0; i < N_SCAN; ++i)
        {
            // 记录每线束的起始和结束索引(前后各留5个点缓冲)
            cloudInfo.startRingIndex[i] = count - 1 + 5;

            // 遍历该线束的所有列
            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i,j) != FLT_MAX)
                {
                    // 保存列索引
                    cloudInfo.pointColInd[count] = j;
                    // 保存距离信息
                    cloudInfo.pointRange[count] = rangeMat.at<float>(i,j);
                    // 保存提取的点
                    extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    // 更新计数器
                    ++count;
                }
            }
            cloudInfo.endRingIndex[i] = count -1 - 5;
        }
    }
    
    /*
     * 发布处理后的点云和信息
     */
    void publishClouds()
    {
        // 设置消息头
        cloudInfo.header = cloudHeader;
        // 发布去畸变后的点云
        cloudInfo.cloud_deskewed  = publishCloud(&pubExtractedCloud, extractedCloud, cloudHeader.stamp, robot_id + "/" + lidarFrame);
        // 发布点云信息
        pubLaserCloudInfo.publish(cloudInfo);
    }
};

/*
 * 主函数
 */
int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "disco_double");

    // 创建图像投影处理对象
    ImageProjection IP;
    
    // 打印启动信息
    ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

    // 启动多线程spinner
    // 和C++多线程的区别：
    // 用途专一：主要用于并行处理 ROS 订阅话题的回调函数。
    // 与 ROS 系统深度集成，能自动管理 ROS 节点的消息队列，确保消息按顺序处理。
    // 用户只需指定线程数量，ROS 会自动管理这些线程的生命周期和任务分配。
    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    // 主线程
    // ├── 线程1 -> 处理IMU数据
    // ├── 线程2 -> 处理点云数据
    // └── 线程3 -> 处理里程计数据
    // - 每个线程都可以处理任何类型的回调
    // - 当有新消息到达时，空闲的线程会立即处理对应的回调函数
    // - 多线程处理可以避免某个耗时回调阻塞其他消息的处理
    // - 需要使用互斥锁（如代码中的 imuLock 和 odoLock ）保护共享数据
    // - 回调函数需要考虑线程安全性
    
    return 0;
}