#include "utility.h"

// GTSAM库头文件
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

// 使用GTSAM符号简写
using gtsam::symbol_shorthand::X; // 位姿 Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // 速度 Vel (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // 偏差 Bias (ax,ay,az,gx,gy,gz)

// 变换融合类，用于融合IMU和激光雷达里程计
class TransformFusion : public ParamServer
{
public:
    std::mutex mtx; // 互斥锁

    // 融合话题（多机器人）
    std::string _fusion_topic;
    ros::Subscriber subFusionTrans;
    double fusionTrans[6]; // x,y,z,roll,pitch,yaw

    // 订阅器
    ros::Subscriber subImuOdometry;
    ros::Subscriber subLaserOdometry;

    // 发布器
    ros::Publisher pubImuOdometry;
    ros::Publisher pubImuPath;

    // 变换矩阵
    Eigen::Affine3f lidarOdomAffine;
    Eigen::Affine3f imuOdomAffineFront;
    Eigen::Affine3f imuOdomAffineBack;

    // TF监听器
    tf::TransformListener tfListener;
    tf::StampedTransform lidar2Baselink;

    double lidarOdomTime = -1;
    deque<nav_msgs::Odometry> imuOdomQueue;
    deque<nav_msgs::Odometry> lidarOdomQueue;

    // 构造函数
    TransformFusion()
    {
        // 如果激光雷达坐标系与基座坐标系不同，获取它们之间的变换
        if(lidarFrame != baselinkFrame)
        {
            try
            {
                tfListener.waitForTransform(robot_id + "/" + lidarFrame, robot_id + "/" + baselinkFrame, ros::Time(0), ros::Duration(3.0));
                tfListener.lookupTransform(robot_id + "/" + lidarFrame, robot_id + "/" + baselinkFrame, ros::Time(0), lidar2Baselink);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
            }
        }

        // 初始化融合变换（多机器人）
        fusionTrans[0] = 0; fusionTrans[1] = 0; fusionTrans[2] = 0; fusionTrans[3] = 0; fusionTrans[4] = 0; fusionTrans[5] = 0;
        nh.getParam("/mapfusion/interRobot/sc_topic", _fusion_topic);
        
        // 订阅融合变换（多机器人）、激光雷达里程计和IMU里程计
        subFusionTrans   = nh.subscribe<nav_msgs::Odometry>(robot_id + "/" + _fusion_topic + "/trans_map", 2000,&TransformFusion::FusionTransHandler, this, ros::TransportHints().tcpNoDelay());
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>(robot_id + "/disco_double/mapping/odometry", 5, &TransformFusion::lidarOdometryHandler, this, ros::TransportHints().tcpNoDelay());
        subImuOdometry   = nh.subscribe<nav_msgs::Odometry>(robot_id + "/" + odomTopic+"_incremental",   2000, &TransformFusion::imuOdometryHandler,   this, ros::TransportHints().tcpNoDelay());

        // 发布IMU里程计和路径
        pubImuOdometry   = nh.advertise<nav_msgs::Odometry>(robot_id + "/" + odomTopic, 2000);
        pubImuPath       = nh.advertise<nav_msgs::Path>    (robot_id + "/disco_double/imu/path", 1);
    }

    // 将里程计消息转换为仿射变换
    Eigen::Affine3f odom2affine(nav_msgs::Odometry odom)
    {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

    // 融合变换处理函数（多机器人）
    void FusionTransHandler(const nav_msgs::Odometry::ConstPtr& odomMsg) {
        // 接收里程计
        fusionTrans[0] = odomMsg->pose.pose.position.x;
        fusionTrans[1] = odomMsg->pose.pose.position.y;
        fusionTrans[2] = odomMsg->pose.pose.position.z;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(odomMsg->pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(fusionTrans[3], fusionTrans[4], fusionTrans[5]);

        // 发布TF变换
        tf::TransformBroadcaster tfMap2Odom;
        tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(fusionTrans[3], fusionTrans[4], fusionTrans[5]), tf::Vector3(fusionTrans[0], fusionTrans[1], fusionTrans[2]));
        tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, odomMsg->header.stamp, mapFrame, robot_id + "/" + odometryFrame));
    }

    // 激光雷达里程计处理函数
    void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        lidarOdomAffine = odom2affine(*odomMsg);
        lidarOdomTime = odomMsg->header.stamp.toSec();
    }

    // IMU里程计处理函数
    void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        // 发布地图到里程计的变换（不应该是静态TF，如果是静态的，它们将不会改变！）
        // static tf::TransformBroadcaster tfMap2Odom;
        tf::TransformBroadcaster tfMap2Odom;
        // static tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), 
                                                                // tf::Vector3(0, 0, 0));
        tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(fusionTrans[3], 
                                                    fusionTrans[4], fusionTrans[5]), 
                                                    tf::Vector3(fusionTrans[0], fusionTrans[1], 
                                                        fusionTrans[2]));
        tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, odomMsg->header.stamp, mapFrame, robot_id + "/" + odometryFrame));

        std::lock_guard<std::mutex> lock(mtx);

        imuOdomQueue.push_back(*odomMsg);

        // 获取最新的里程计（在当前IMU时间戳）
        if (lidarOdomTime == -1)
            return;
        while (!imuOdomQueue.empty())
        {
            if (imuOdomQueue.front().header.stamp.toSec() <= lidarOdomTime)
                imuOdomQueue.pop_front();
            else
                break;
        }
        Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front());
        Eigen::Affine3f imuOdomAffineBack = odom2affine(imuOdomQueue.back());
        Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
        Eigen::Affine3f imuOdomAffineLast = lidarOdomAffine * imuOdomAffineIncre;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch, yaw);
        
        // 发布最新的里程计
        nav_msgs::Odometry laserOdometry = imuOdomQueue.back();
        laserOdometry.pose.pose.position.x = x;
        laserOdometry.pose.pose.position.y = y;
        laserOdometry.pose.pose.position.z = z;
        laserOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        pubImuOdometry.publish(laserOdometry);

        // 发布TF变换
        static tf::TransformBroadcaster tfOdom2BaseLink;
        tf::Transform tCur;
        tf::poseMsgToTF(laserOdometry.pose.pose, tCur);
        if(lidarFrame != baselinkFrame)
            tCur = tCur * lidar2Baselink;
        tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, odomMsg->header.stamp, robot_id + "/" + odometryFrame, robot_id + "/" + baselinkFrame);
        tfOdom2BaseLink.sendTransform(odom_2_baselink);

        // 发布IMU路径
        static nav_msgs::Path imuPath;
        static double last_path_time = -1;
        double imuTime = imuOdomQueue.back().header.stamp.toSec();
        if (imuTime - last_path_time > 0.1)
        {
            last_path_time = imuTime;
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
            pose_stamped.header.frame_id = robot_id + "/" + odometryFrame;
            pose_stamped.pose = laserOdometry.pose.pose;
            imuPath.poses.push_back(pose_stamped);
            // 修改里程计时间
            while(!imuPath.poses.empty() && imuPath.poses.front().header.stamp.toSec() < lidarOdomTime - 1.0)
            // while(!imuPath.poses.empty() && imuPath.poses.front().header.stamp.toSec() < lidarOdomTime - 0.1)
                imuPath.poses.erase(imuPath.poses.begin());
            if (pubImuPath.getNumSubscribers() != 0)
            {
                imuPath.header.stamp = imuOdomQueue.back().header.stamp;
                imuPath.header.frame_id = robot_id + "/" + odometryFrame;
                pubImuPath.publish(imuPath);
            }
        }
    }
};

// IMU预积分类
class IMUPreintegration : public ParamServer
{
public:

    std::mutex mtx; // 互斥锁

    // 订阅器和发布器
    ros::Subscriber subImu;
    ros::Subscriber subOdometry;
    ros::Publisher pubImuOdometry;

    bool systemInitialized = false; // 系统是否初始化

    // 噪声模型
    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;

    // IMU积分器
    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

    // IMU消息队列
    std::deque<sensor_msgs::Imu> imuQueOpt;
    std::deque<sensor_msgs::Imu> imuQueImu;

    // 上一状态
    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    bool doneFirstOpt = false;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;

    // 优化器和因子图
    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    const double delta_t = 0;

    int key = 1;

    // IMU到激光雷达的变换
    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

    // 构造函数
    IMUPreintegration()
    {
        // 订阅IMU和里程计
        subImu      = nh.subscribe<sensor_msgs::Imu>  (robot_id + "/" + imuTopic,                   2000, &IMUPreintegration::imuHandler,      this, ros::TransportHints().tcpNoDelay());
        subOdometry = nh.subscribe<nav_msgs::Odometry>(robot_id + "/disco_double/mapping/odometry_incremental", 5,    &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay());

        // 发布IMU里程计
        pubImuOdometry = nh.advertise<nav_msgs::Odometry> (robot_id + "/" + odomTopic+"_incremental", 2000);

        // 设置IMU预积分参数
        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2); // 加速度计白噪声（连续）
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2); // 陀螺仪白噪声（连续）
        p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // 从速度积分位置时产生的误差
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // 假设初始偏差为零

        // 设置先验噪声
        priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
        priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 似乎是不错的选择
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();
        
        // 设置IMU积分器
        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // 为IMU消息线程设置IMU积分
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // 为优化设置IMU积分        
    }

    // 重置优化
    void resetOptimization()
    {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

    // 重置参数
    void resetParams()
    {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }

    // 里程计处理函数
    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        double currentCorrectionTime = ROS_TIME(odomMsg);

        // 确保我们有IMU数据进行积分
        if (imuQueOpt.empty())
            return;

        float p_x = odomMsg->pose.pose.position.x;
        float p_y = odomMsg->pose.pose.position.y;
        float p_z = odomMsg->pose.pose.position.z;
        float r_x = odomMsg->pose.pose.orientation.x;
        float r_y = odomMsg->pose.pose.orientation.y;
        float r_z = odomMsg->pose.pose.orientation.z;
        float r_w = odomMsg->pose.pose.orientation.w;
        bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
        gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));

        // 0. 初始化系统
        if (systemInitialized == false)
        {
            resetOptimization();

            // 弹出旧的IMU消息
            while (!imuQueOpt.empty())
            {
                if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t)
                {
                    lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                    imuQueOpt.pop_front();
                }
                else
                    break;
            }
            // 初始位姿
            // 初始化因子图的prior状态
            // 将雷达里程计位姿平移到IMU坐标系，只是做了平移
            prevPose_ = lidarPose.compose(lidar2Imu);
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            graphFactors.add(priorPose);
            // 初始速度
            prevVel_ = gtsam::Vector3(0, 0, 0);
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
            graphFactors.add(priorVel);
            // 初始偏差
            prevBias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
            graphFactors.add(priorBias);
            // 添加值
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // 优化一次
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
            
            key = 1;
            systemInitialized = true;
            return;
        }

        // 为了提高速度，重置图
        if (key == 100)
        {
            // 在重置前获取更新的噪声
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key-1)));
            // 重置图
            resetOptimization();
            // 添加位姿
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors.add(priorPose);
            // 添加速度
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors.add(priorVel);
            // 添加偏差
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors.add(priorBias);
            // 添加值
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // 优化一次
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            key = 1;
        }

        // 1. 积分IMU数据并优化
        while (!imuQueOpt.empty())
        {
            // 弹出并积分两次优化之间的IMU数据
            sensor_msgs::Imu *thisImu = &imuQueOpt.front();
            double imuTime = ROS_TIME(thisImu);
            if (imuTime < currentCorrectionTime - delta_t)
            {
                double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                
                lastImuT_opt = imuTime;
                imuQueOpt.pop_front();
            }
            else
                break;
        }
        // 向图中添加IMU因子
        const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);
        // 添加IMU偏差之间的因子
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                         gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
        // 添加位姿因子
        gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
        graphFactors.add(pose_factor);
        // 插入预测值
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);
        // 优化
        optimizer.update(graphFactors, graphValues);
        optimizer.update();
        graphFactors.resize(0);
        graphValues.clear();
        // 覆盖下一步预积分的开始
        gtsam::Values result = optimizer.calculateEstimate();
        prevPose_  = result.at<gtsam::Pose3>(X(key));
        prevVel_   = result.at<gtsam::Vector3>(V(key));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
        // 重置优化预积分对象
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        // 检查优化
        if (failureDetection(prevVel_, prevBias_))
        {
            resetParams();
            return;
        }

        // 2. 优化后，重新传播IMU里程计预积分
        prevStateOdom = prevState_;
        prevBiasOdom  = prevBias_;
        // 同样先做IMU数据队列和雷达里程计的时间同步
        // 首先弹出早于当前校正数据的IMU消息
        double lastImuQT = -1;
        while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t)
        {
            lastImuQT = ROS_TIME(&imuQueImu.front());
            imuQueImu.pop_front();
        }
        // 重新传播
        if (!imuQueImu.empty())
        {
            // 使用新优化的偏差重置偏差
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            // 从这次优化的开始积分IMU消息
            for (int i = 0; i < (int)imuQueImu.size(); ++i)
            {
                sensor_msgs::Imu *thisImu = &imuQueImu[i];
                double imuTime = ROS_TIME(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / 500.0) :(imuTime - lastImuQT);

                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                lastImuQT = imuTime;
            }
        }

        ++key;
        doneFirstOpt = true;
    }

    // 故障检测函数 - 检测IMU积分是否出现异常
    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur)
    {
        // 将当前速度转换为Eigen向量格式
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        // 检查速度是否过大（超过30m/s）
        // if (vel.norm() > 30)
        // 适应低频IMU
        if (vel.norm() > 50)
        {
            ROS_WARN("Large velocity, reset IMU-preintegration!"); // 警告：速度过大，重置IMU预积分
            return true; // 返回故障标志
        }

        // 获取加速度计和陀螺仪的偏差
        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        // 检查偏差是否过大（超过1.0）
        if (ba.norm() > 1.0 || bg.norm() > 1.0)
        {
            ROS_WARN("Large bias, reset IMU-preintegration!"); // 警告：偏差过大，重置IMU预积分
            return true; // 返回故障标志
        }

        return false; // 无故障
    }

    // IMU数据处理函数
    void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw)
    {
        std::lock_guard<std::mutex> lock(mtx); // 加锁，确保线程安全

        // 转换IMU数据（可能包括坐标系转换或单位转换）
        sensor_msgs::Imu thisImu = imuConverter(*imu_raw);

        // 将IMU数据添加到优化队列和IMU队列
        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);

        // 如果尚未完成第一次优化，则直接返回
        if (doneFirstOpt == false)
            return;

        // 计算当前IMU时间和时间间隔
        double imuTime = ROS_TIME(&thisImu);
        // 如果是第一个IMU数据，使用默认时间间隔(1/500s)，否则计算实际时间间隔
        double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime; // 更新上一次IMU时间

        // 积分单个IMU消息（将加速度和角速度积分到当前状态）
        imuIntegratorImu_->integrateMeasurement(
            gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
            gtsam::Vector3(thisImu.angular_velocity.x, thisImu.angular_velocity.y, thisImu.angular_velocity.z), 
            dt);

        // 预测当前导航状态（位置、姿态和速度）
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

        // 创建并发布里程计消息
        nav_msgs::Odometry odometry;
        odometry.header.stamp = thisImu.header.stamp; // 使用IMU时间戳
        odometry.header.frame_id = robot_id + "/" + odometryFrame; // 设置坐标系
        odometry.child_frame_id = robot_id + "/" + lidarFrame + "/odom_imu"; // 设置子坐标系
        
        // 将IMU里程计完全对齐到雷达（剩下一个平移关系）
        // 将IMU位姿转换为激光雷达位姿
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar); // 应用IMU到激光雷达的变换

        // 设置位姿信息
        odometry.pose.pose.position.x = lidarPose.translation().x();
        odometry.pose.pose.position.y = lidarPose.translation().y();
        odometry.pose.pose.position.z = lidarPose.translation().z();
        odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
        
        // 设置速度和角速度信息
        odometry.twist.twist.linear.x = currentState.velocity().x(); // 线性速度x
        odometry.twist.twist.linear.y = currentState.velocity().y(); // 线性速度y
        odometry.twist.twist.linear.z = currentState.velocity().z(); // 线性速度z
        // 角速度需要加上陀螺仪偏差进行补偿
        odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        
        // 发布IMU里程计消息
        pubImuOdometry.publish(odometry);
    }
};


// 主函数
int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "disco_double");
    
    // 创建IMU预积分对象
    IMUPreintegration ImuP;

    // 创建变换融合对象
    TransformFusion TF;

    // 输出启动信息（绿色文字）
    ROS_INFO("\033[1;32m----> IMU Preintegration Started.\033[0m");
    
    // 创建多线程spinner，使用4个线程处理回调
    ros::MultiThreadedSpinner spinner(4);
    // 开始处理回调
    spinner.spin();
    
    return 0;
}
