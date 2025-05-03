//
// Created by yewei on 8/31/20.
//

//msg
#include "disco_slam/cloud_info.h"
#include "disco_slam/context_info.h"

//third party
#include "scanContext/scanContext.h"
#include "fast_max-clique_finder/src/findClique.h"

#include "nabo/nabo.h"

//ros
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

//gtsam
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

//expression graph
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/slam/dataset.h>

//factor graph
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ISAM2.h>

//pcl
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <unordered_map>
#include <thread>
#include <mutex>

// 定义一个内联函数，用于计算两个位姿之间的变换
// 参数x: 源位姿
// 参数p: 目标位姿
// 返回值: 表示从x到p的变换位姿
inline gtsam::Pose3_ transformTo(const gtsam::Pose3_& x, const gtsam::Pose3_& p) {
    return gtsam::Pose3_(x, &gtsam::Pose3::transform_pose_to, p);
}
/**
 * @brief 发布点云数据到ROS话题
 * @param thisPub ROS发布器指针，用于发布点云数据
 * @param thisCloud 待发布的点云数据指针
 * @param thisStamp ROS时间戳，用于设置点云消息的时间
 * @param thisFrame 坐标系名称，用于设置点云消息的frame_id
 * @return 返回构建好的sensor_msgs::PointCloud2消息
 */
sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;  // 创建ROS点云消息对象
    pcl::toROSMsg(*thisCloud, tempCloud);  // 将PCL点云转换为ROS消息格式
    tempCloud.header.stamp = thisStamp;  // 设置消息时间戳
    tempCloud.header.frame_id = thisFrame;  // 设置坐标系
    
    // 如果有订阅者则发布消息
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
        
    return tempCloud;  // 返回构建好的消息
}

class MapFusion{

private:
    // ROS节点句柄，用于发布和订阅
ros::NodeHandle nh;

// ROS订阅器 - 订阅各种传感器和状态信息
ros::Subscriber _sub_laser_cloud_info;      // 激光点云信息
ros::Subscriber _sub_scan_context_info;     // 扫描上下文信息  
ros::Subscriber _sub_odom_trans;           // 里程计变换
ros::Subscriber _sub_loop_info_global;     // 全局闭环信息
ros::Subscriber _sub_communication_signal; // 通信信号
ros::Subscriber _sub_signal_1;             // 信号1
ros::Subscriber _sub_signal_2;             // 信号2

// 信号ID字符串
std::string _signal_id_1;
std::string _signal_id_2;

// ROS发布器 - 发布各种信息
ros::Publisher _pub_context_info;      // 上下文信息
ros::Publisher _pub_loop_info;         // 闭环信息
ros::Publisher _pub_cloud;             // 点云数据
ros::Publisher _pub_trans_odom2map;    // 里程计到地图的变换
ros::Publisher _pub_trans_odom2odom;   // 里程计间的变换
ros::Publisher _pub_loop_info_global;  // 全局闭环信息

// 系统参数
std::string _robot_id;      // 当前机器人ID
std::string _robot_this;    // 当前处理的机器人ID
std::string _sc_topic;      // 扫描上下文话题
std::string _sc_frame;      // 扫描上下文坐标系
std::string _local_topic;   // 本地话题

// 状态标志
bool _communication_signal; // 通信信号状态
bool _signal_1;            // 信号1状态
bool _signal_2;            // 信号2状态
bool _use_position_search;  // 是否使用位置搜索

// 各种阈值和参数
int _num_bin;              // bin数量
int _robot_id_th;          // 机器人ID阈值
int _robot_this_th;        // 当前机器人ID阈值
int _max_range;            // 最大范围
int _num_sectors;          // 扇区数量
int _knn_feature_dim;      // KNN特征维度
int _num_nearest_matches;  // 最近邻匹配数量
int _num_match_candidates; // 匹配候选数量
int _pcm_start_threshold;  // PCM启动阈值
float _loop_thres;         // 闭环阈值
float _pcm_thres;          // PCM阈值
float _icp_thres;          // ICP阈值
int _loop_frame_thres;     // 闭环帧阈值

// 互斥锁
std::mutex mtx_publish_1;  // 发布锁1
std::mutex mtx_publish_2;  // 发布锁2
std::mutex mtx;            // 通用锁

// 初始化和文件夹参数
std::string _robot_initial;  // 初始机器人
std::string _pcm_matrix_folder; // PCM矩阵文件夹

// 数据存储结构
disco_slam::cloud_info _cloud_info;  // 点云信息
std::vector<ScanContextBin> _context_list_to_publish_1; // 待发布上下文列表1
std::vector<ScanContextBin> _context_list_to_publish_2; // 待发布上下文列表2

// 点云处理相关
pcl::KdTreeFLANN<PointType>::Ptr _kdtree_pose_to_publish; // 发布位姿的KD树
pcl::PointCloud<PointType>::Ptr _cloud_pose_to_publish;   // 待发布位姿点云
pcl::KdTreeFLANN<PointType>::Ptr _kdtree_pose_to_search;  // 搜索位姿的KD树
pcl::PointCloud<PointType>::Ptr _cloud_pose_to_search_this; // 当前搜索位姿点云
pcl::PointCloud<PointType>::Ptr _cloud_pose_to_search_other; // 其他搜索位姿点云
pcl::KdTreeFLANN<PointType>::Ptr _kdtree_loop_to_search;  // 闭环搜索KD树
pcl::PointCloud<PointType>::Ptr _cloud_loop_to_search;    // 闭环搜索点云
pcl::VoxelGrid<PointType> _downsize_filter_icp;          // ICP降采样滤波器

// 状态变量
std::pair<int, int> _initial_loop;  // 初始闭环
int _id_bin_last;                   // 上一个bin ID
disco_slam::context_info _loop_info; // 闭环信息
std_msgs::Header _cloud_header;      // 点云头信息

// 点云数据
pcl::PointCloud<PointType>::Ptr _laser_cloud_sum;     // 总点云
pcl::PointCloud<PointType>::Ptr _laser_cloud_feature; // 特征点云
pcl::PointCloud<PointType>::Ptr _laser_cloud_corner;  // 角点点云
pcl::PointCloud<PointType>::Ptr _laser_cloud_surface;  // 平面点云

// 扫描上下文相关
Nabo::NNSearchF* _nns = NULL;        // KD树搜索
Eigen::MatrixXf _target_matrix;       // 目标矩阵
ScanContext *_scan_context_factory;   // 扫描上下文工厂

// 数据存储容器
std::vector<int> _robot_received_list;  // 接收到的机器人列表
std::vector<std::pair<int, int>> _idx_nearest_list; // 最近邻索引列表
std::unordered_map<int, ScanContextBin> _bin_with_id; // ID到bin的映射

// 位姿和闭环队列
std::unordered_map<int, std::vector<std::tuple<gtsam::Pose3, gtsam::Pose3, float>>> _pose_queue;
std::unordered_map<int, std::vector<std::tuple<int, int, gtsam::Pose3>>> _loop_queue;

// 全局变换信息
std::unordered_map<std::string, std::vector<PointTypePose>> _global_odom_trans;
// 
std::unordered_map< int, std::vector<int> > _loop_accept_queue;
// 全局变换信息优化
std::unordered_map<int, std::vector<PointTypePose>> _global_map_trans;
std::unordered_map<int, PointTypePose> _global_map_trans_optimized;

// 其他变量
int number_print;  // 打印计数
PointTypePose _trans_to_publish;  // 待发布的变换
std::vector<std::pair<string, double>> _processing_time_list; // 处理时间列表

public:

        /**
     * @brief MapFusion类的构造函数
     * 
     * 1. 加载参数和初始化成员变量
     * 2. 设置ROS订阅器和发布器
     * 3. 根据是否为初始机器人决定是否订阅额外话题
     */
    MapFusion(){
        // 加载参数配置
        ParamLoader();
        // 初始化成员变量和数据结构
        initialization();

        // 设置通信信号订阅器
        _sub_communication_signal = nh.subscribe<std_msgs::Bool>(_robot_id + "/disco_slam/signal",
                 100, &MapFusion::communicationSignalHandler, this, ros::TransportHints().tcpNoDelay());

        // 设置信号1和信号2的订阅器
        _sub_signal_1 = nh.subscribe<std_msgs::Bool>(_signal_id_1 + "/disco_slam/signal",
                 100, &MapFusion::signalHandler1, this, ros::TransportHints().tcpNoDelay());
        _sub_signal_2 = nh.subscribe<std_msgs::Bool>(_signal_id_2 + "/disco_slam/signal",
                 100, &MapFusion::signalHandler2, this, ros::TransportHints().tcpNoDelay());

        // 设置激光点云信息订阅器
        _sub_laser_cloud_info = nh.subscribe<disco_slam::cloud_info>(_robot_id + "/" + _local_topic,
                1, &MapFusion::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());

        // 设置全局闭环信息订阅器
        _sub_loop_info_global = nh.subscribe<disco_slam::context_info>(_sc_topic + "/loop_info_global",
                            100, &MapFusion::globalLoopInfoHandler, this, ros::TransportHints().tcpNoDelay());

        // 如果不是初始机器人，则订阅额外话题
        if(_robot_id != _robot_initial){
            // 设置扫描上下文信息订阅器
            _sub_scan_context_info = nh.subscribe<disco_slam::context_info>(_sc_topic + "/context_info",
                20, &MapFusion::scanContextInfoHandler, this, ros::TransportHints().tcpNoDelay());
            // 设置里程计变换订阅器
            _sub_odom_trans = nh.subscribe<nav_msgs::Odometry>(_sc_topic + "/trans_odom",
                           20, &MapFusion::OdomTransHandler, this, ros::TransportHints().tcpNoDelay());
        }

        // 设置各种发布器
        _pub_context_info     = nh.advertise<disco_slam::context_info> (_sc_topic + "/context_info", 1);
        _pub_loop_info        = nh.advertise<disco_slam::context_info> (_robot_id + "/" + _sc_topic + "/loop_info", 1);
        _pub_cloud            = nh.advertise<sensor_msgs::PointCloud2> (_robot_id + "/" + _sc_topic + "/cloud", 1);
        _pub_trans_odom2map   = nh.advertise<nav_msgs::Odometry> ( _robot_id + "/" + _sc_topic + "/trans_map", 1);
        _pub_trans_odom2odom  = nh.advertise<nav_msgs::Odometry> ( _sc_topic + "/trans_odom", 1);
        _pub_loop_info_global = nh.advertise<disco_slam::context_info>(_sc_topic + "/loop_info_global", 1);
    }


        /**
     * @brief 发布上下文信息的线程函数
     * 
     * 1. 持续运行直到ROS关闭
     * 2. 根据通信信号和机器人ID优先级决定发布顺序
     * 3. 使用互斥锁保护共享数据
     */
    void publishContextInfoThread(){
        // 将信号ID转换为数字形式
        int signal_id_th_1 = robotID2Number(_signal_id_1);
        int signal_id_th_2 = robotID2Number(_signal_id_2);
        
        // 主循环，持续运行直到ROS关闭
        while (ros::ok())
        {
            // 检查信号1的条件是否满足
            if (_communication_signal && _signal_1 && _robot_id_th < signal_id_th_1){
                // 如果发布列表为空则跳过
                if (_context_list_to_publish_1.empty())
                    continue;
                    
                // 加锁保护共享数据
                mtx_publish_1.lock();
                // 获取最新的上下文信息
                ScanContextBin bin = _context_list_to_publish_1.back();
                _context_list_to_publish_1.pop_back();
                mtx_publish_1.unlock();
                
                // 发布上下文信息
                publishContextInfo(bin, _signal_id_1);
            }
            
            // 检查信号2的条件是否满足
            if (_communication_signal && _signal_2 && _robot_id_th < signal_id_th_2){
                // 如果发布列表为空则跳过
                if (_context_list_to_publish_2.empty())
                    continue;
                    
                // 加锁保护共享数据
                mtx_publish_2.lock();
                // 获取最新的上下文信息
                ScanContextBin bin = _context_list_to_publish_2.back();
                _context_list_to_publish_2.pop_back();
                mtx_publish_2.unlock();
                
                // 发布上下文信息
                publishContextInfo(bin, _signal_id_2);
            }
        }
    }


private:
        /**
     * @brief 参数加载函数，从ROS参数服务器加载各种配置参数
     * 
     * 1. 从私有命名空间加载机器人ID和信号ID等基本参数
     * 2. 从全局命名空间加载扫描上下文相关参数
     * 3. 从全局命名空间加载多机器人交互相关参数
     */
    void ParamLoader(){
        // 创建私有节点句柄，用于获取私有参数
        ros::NodeHandle n("~");
        
        // 加载基本参数
        n.param<std::string>("robot_id", _robot_id, "jackal0");  // 当前机器人ID，默认jackal0
        n.param<std::string>("id_1",  _signal_id_1, "jackal1");  // 信号1机器人ID，默认jackal1
        n.param<std::string>("id_2",  _signal_id_2, "jackal2");  // 信号2机器人ID，默认jackal2
        n.param<int>("no", number_print, 100);  // 打印计数，默认100
        n.param<std::string>("pcm_matrix_folder",  _pcm_matrix_folder, "aaa");  // PCM矩阵存储文件夹，默认aaa

        // 加载扫描上下文相关参数
        nh.getParam("/mapfusion/scancontext/knn_feature_dim", _knn_feature_dim);  // KNN特征维度
        nh.getParam("/mapfusion/scancontext/max_range", _max_range);  // 最大扫描范围
        nh.getParam("/mapfusion/scancontext/num_sector", _num_sectors);  // 扇区数量
        nh.getParam("/mapfusion/scancontext/num_nearest_matches", _num_nearest_matches);  // 最近邻匹配数量
        nh.getParam("/mapfusion/scancontext/num_match_candidates", _num_match_candidates);  // 匹配候选数量

        // 加载多机器人交互相关参数
        nh.getParam("/mapfusion/interRobot/loop_threshold", _loop_thres);  // 闭环检测阈值
        nh.getParam("/mapfusion/interRobot/pcm_threshold",_pcm_thres);  // PCM阈值
        nh.getParam("/mapfusion/interRobot/icp_threshold",_icp_thres);  // ICP阈值
        nh.getParam("/mapfusion/interRobot/robot_initial",_robot_initial);  // 初始机器人ID
        nh.getParam("/mapfusion/interRobot/loop_frame_threshold", _loop_frame_thres);  // 闭环帧阈值

        // 加载话题和帧名称参数
        nh.getParam("/mapfusion/interRobot/sc_topic", _sc_topic);  // 扫描上下文话题
        nh.getParam("/mapfusion/interRobot/sc_frame", _sc_frame);  // 扫描上下文坐标系
        nh.getParam("/mapfusion/interRobot/local_topic", _local_topic);  // 本地话题
        nh.getParam("/mapfusion/interRobot/pcm_start_threshold", _pcm_start_threshold);  // PCM启动阈值
        nh.getParam("/mapfusion/interRobot/use_position_search", _use_position_search);  // 是否使用位置搜索
    }


        /**
     * @brief 初始化函数，用于初始化各类成员变量和数据结构
     * 
     * 1. 初始化点云相关数据结构
     * 2. 创建扫描上下文工厂对象
     * 3. 初始化KD树和点云容器
     * 4. 设置ICP降采样滤波器参数
     * 5. 初始化状态变量和标志位
     */
    void initialization(){
        // 初始化点云容器
        _laser_cloud_sum.reset(new pcl::PointCloud<PointType>());      // 总点云初始化
        _laser_cloud_feature.reset(new pcl::PointCloud<PointType>());  // 特征点云初始化
        _laser_cloud_corner.reset(new pcl::PointCloud<PointType>());   // 角点点云初始化
        _laser_cloud_surface.reset(new pcl::PointCloud<PointType>());  // 平面点云初始化

        // 创建扫描上下文工厂对象
        _scan_context_factory = new ScanContext(_max_range, _knn_feature_dim, _num_sectors);

        // 初始化KD树和点云容器
        _kdtree_pose_to_publish.reset(new pcl::KdTreeFLANN<PointType>());  // 发布位姿的KD树
        _cloud_pose_to_publish.reset(new pcl::PointCloud<PointType>());    // 待发布位姿点云

        _kdtree_pose_to_search.reset(new pcl::KdTreeFLANN<PointType>());   // 搜索位姿的KD树
        _cloud_pose_to_search_this.reset(new pcl::PointCloud<PointType>()); // 当前搜索位姿点云
        _cloud_pose_to_search_other.reset(new pcl::PointCloud<PointType>()); // 其他搜索位姿点云

        _kdtree_loop_to_search.reset(new pcl::KdTreeFLANN<PointType>());  // 闭环搜索KD树
        _cloud_loop_to_search.reset(new pcl::PointCloud<PointType>());     // 闭环搜索点云

        // 设置ICP降采样滤波器参数
        // _downsize_filter_icp.setLeafSize(0.4, 0.4, 0.4);  // 设置体素网格大小为0.4m
        _downsize_filter_icp.setLeafSize(0.4, 0.2, 0.4);  // 设置体素网格大小为0.4m

        // 初始化状态变量
        _initial_loop.first = -1;  // 初始闭环标记为无效
        _robot_id_th = robotID2Number(_robot_id);  // 转换机器人ID为数字形式
        _trans_to_publish.intensity = 0;  // 待发布变换强度初始化为0
        _num_bin = 0;  // bin计数器初始化为0

        // 初始化通信信号标志位
        _communication_signal = true;  // 通信信号初始化为有效
        _signal_1 = true;             // 信号1初始化为有效
        _signal_2 = true;             // 信号2初始化为有效
    }


        /**
     * @brief 将机器人ID字符串转换为数字形式
     * @param robo 机器人ID字符串，如"jackal0"
     * @return 返回机器人ID中的数字部分，如"jackal0"返回0
     * 
     * 该函数通过提取字符串最后一个字符并减去'0'的ASCII值，
     * 将机器人ID转换为对应的数字标识
     */
    int robotID2Number(std::string robo){
        return robo.back() - '0';
    }


        /**
     * @brief 激光点云信息处理函数
     * @param msgIn 输入的点云信息消息
     * 
     * 1. 清空并加载最新的点云数据
     * 2. 创建扫描上下文bin
     * 3. 将bin信息加入待发布队列
     * 4. 发布上下文信息
     */
    void laserCloudInfoHandler(const disco_slam::cloud_infoConstPtr& msgIn)
    {
        // 清空各类点云容器
        _laser_cloud_sum->clear();      // 清空总点云
        _laser_cloud_feature->clear();   // 清空特征点云
        _laser_cloud_corner->clear();    // 清空角点点云
        _laser_cloud_surface->clear();   // 清空平面点云

        // 加载最新数据
        _cloud_info = *msgIn;           // 更新点云信息
        _cloud_header = msgIn->header;  // 更新点云头信息
        pcl::fromROSMsg(msgIn->cloud_deskewed, *_laser_cloud_sum);     // 加载去畸变点云
        pcl::fromROSMsg(msgIn->cloud_corner, *_laser_cloud_corner);    // 加载角点点云
        pcl::fromROSMsg(msgIn->cloud_surface, *_laser_cloud_surface);   // 加载平面点云
        *_laser_cloud_feature += *_laser_cloud_corner;  // 合并角点特征
        *_laser_cloud_feature += *_laser_cloud_surface; // 合并平面特征

        // 创建扫描上下文bin
        ScanContextBin bin = _scan_context_factory->ptcloud2bin(_laser_cloud_sum);
        bin.robotname = _robot_id;  // 设置机器人ID
        bin.time = _cloud_header.stamp.toSec();  // 设置时间戳
        // 设置位姿信息
        bin.pose.x = _cloud_info.initialGuessX;
        bin.pose.y = _cloud_info.initialGuessY;
        bin.pose.z = _cloud_info.initialGuessZ;
        bin.pose.roll  =  _cloud_info.initialGuessRoll;
        bin.pose.pitch =  _cloud_info.initialGuessPitch;
        bin.pose.yaw   =  _cloud_info.initialGuessYaw;
        bin.pose.intensity = _cloud_info.imuAvailable;  // 设置IMU可用标志

        // 准备特征点云
        bin.cloud->clear();
        pcl::copyPointCloud(*_laser_cloud_feature,  *bin.cloud);  // 复制特征点云

        // 将bin信息加入待发布队列1
        mtx_publish_1.lock();
        _context_list_to_publish_1.push_back(bin);
        mtx_publish_1.unlock();
        
        // 将bin信息加入待发布队列2
        mtx_publish_2.lock();
        _context_list_to_publish_2.push_back(bin);
        mtx_publish_2.unlock();

        // 发布上下文信息
        publishContextInfo(bin, _robot_id);
    }


        /**
     * @brief 通信信号处理函数
     * @param msg 接收到的布尔类型消息
     * 
     * 该函数用于处理来自其他机器人的通信信号，
     * 更新内部通信状态标志_communication_signal
     */
    void communicationSignalHandler(const std_msgs::Bool::ConstPtr& msg){
        _communication_signal = msg->data;  // 更新通信信号状态
    }


        /**
     * @brief 信号1处理函数
     * @param msg 接收到的布尔类型消息
     * 
     * 该函数用于处理来自其他机器人的信号1消息，
     * 更新内部信号1状态标志_signal_1
     */
    void signalHandler1(const std_msgs::Bool::ConstPtr& msg){
        _signal_1 = msg->data;  // 更新信号1状态
    }


        /**
     * @brief 信号2处理函数
     * @param msg 接收到的布尔类型消息
     * 
     * 该函数用于处理来自其他机器人的信号2消息，
     * 更新内部信号2状态标志_signal_2
     */
    void signalHandler2(const std_msgs::Bool::ConstPtr& msg){
        _signal_2 = msg->data;  // 更新信号2状态
    }


        /**
     * @brief 发布上下文信息函数
     * @param bin 包含扫描上下文信息的结构体
     * @param robot_to 目标机器人ID
     * 
     * 1. 构建上下文信息消息
     * 2. 填充扫描上下文和环键数据
     * 3. 设置位姿和点云信息
     * 4. 发布消息（线程安全）
     */
    void publishContextInfo(ScanContextBin bin, std::string robot_to){
        // 初始化上下文信息消息
        disco_slam::context_info context_info;
        context_info.robotID = _robot_id;  // 设置当前机器人ID

        // 设置扫描上下文参数
        context_info.numRing = _knn_feature_dim;    // 环数量
        context_info.numSector = _num_sectors;      // 扇区数量

        // 初始化扫描上下文和环键数据存储
        context_info.scanContextBin.assign(_knn_feature_dim * _num_sectors, 0);  // 扫描上下文数据
        context_info.ringKey.assign(_knn_feature_dim, 0);  // 环键数据
        context_info.header = _cloud_header;  // 设置消息头

        // 填充扫描上下文和环键数据
        int cnt = 0;
        for (int i = 0; i < _knn_feature_dim; i++){
            context_info.ringKey[i] = bin.ringkey(i);  // 设置环键值
            for (int j = 0; j < _num_sectors; j++){
                context_info.scanContextBin[cnt] = bin.bin(i,j);  // 设置扫描上下文值
                ++cnt;
            }
        }

        // 设置位姿信息
        context_info.robotIDReceive = robot_to;  // 目标机器人ID
        context_info.poseX = bin.pose.x;        // X坐标
        context_info.poseY = bin.pose.y;        // Y坐标
        context_info.poseZ = bin.pose.z;        // Z坐标
        context_info.poseRoll = bin.pose.roll;  // 滚转角
        context_info.posePitch = bin.pose.pitch; // 俯仰角
        context_info.poseYaw = bin.pose.yaw;    // 偏航角
        context_info.poseIntensity = bin.pose.intensity; // 强度/标志位

        // 发布点云数据并获取消息
        context_info.scanCloud = publishCloud(&_pub_cloud, bin.cloud, ros::Time(bin.time), _robot_id + "/" + _sc_frame);

        // 线程安全地发布上下文信息
        mtx.lock();
        _pub_context_info.publish(context_info);
        mtx.unlock();
    }


        /**
     * @brief 里程计变换处理函数
     * @param odomMsg 输入的里程计消息
     * 
     * 1. 解析里程计消息中的机器人ID和位姿信息
     * 2. 将位姿信息存储到全局变换映射中
     * 3. 更新因子图并发送地图输出消息
     */
    void OdomTransHandler(const nav_msgs::Odometry::ConstPtr& odomMsg){
        // 获取发布该消息的机器人ID
        std::string robot_publish = odomMsg->header.frame_id;
        // 如果是本节点发布的消息则跳过
        if( robot_publish == _robot_id)
            return;
            
        // 获取子坐标系ID并构建索引键
        std::string robot_child = odomMsg->child_frame_id;
        std::string index = robot_child + robot_publish;
        
        // 解析位姿信息
        PointTypePose pose;
        pose.x = odomMsg->pose.pose.position.x;  // X坐标
        pose.y = odomMsg->pose.pose.position.y;  // Y坐标
        pose.z = odomMsg->pose.pose.position.z;  // Z坐标
        
        // 转换四元数到欧拉角
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(odomMsg->pose.pose.orientation, orientation);
        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        pose.roll = roll; pose.pitch = pitch; pose.yaw = yaw;  // 设置姿态角

        // 使用intensity字段存储机器人ID的数字形式
        pose.intensity = robotID2Number(robot_child);

        // 查找或创建该索引对应的位姿列表
        auto ite = _global_odom_trans.find(index);
        if(ite == _global_odom_trans.end()) {  // 如果是新变换
            std::vector<PointTypePose> tmp_pose_list;
            tmp_pose_list.push_back(pose);
            _global_odom_trans.emplace(std::make_pair(index, tmp_pose_list));
        }
        else {  // 如果已有变换则追加
            _global_odom_trans[index].push_back(pose);
        }

        // 更新因子图并发送地图输出
        gtsamFactorGraph();
        sendMapOutputMessage();
    }


        /**
     * @brief 全局闭环信息处理函数
     * @param msgIn 输入的全局闭环信息消息
     * 
     * 1. 检查消息是否属于当前机器人
     * 2. 发布闭环信息
     * 3. 发送地图输出消息
     */
    void globalLoopInfoHandler(const disco_slam::context_infoConstPtr& msgIn){
        // 跳过调试用的返回语句
        // return;
        
        // 检查消息是否属于当前机器人，不属于则直接返回
        if (msgIn->robotID != _robot_id)
            return;
            
        // 发布闭环信息到对应话题
        _pub_loop_info.publish(*msgIn);
        
        // 发送地图输出消息
        sendMapOutputMessage();
    }


    /**
     * @brief 构建GTSAM因子图并进行优化
     * 
     * 1. 检查是否有足够的变换数据
     * 2. 初始化因子图和初始估计
     * 3. 添加地图变换约束
     * 4. 添加里程计变换约束
     * 5. 执行优化并更新全局变换
     */
    void gtsamFactorGraph(){
        // 检查是否有足够的变换数据
        if (_global_map_trans.size() == 0 && _global_odom_trans.size() == 0)
            return;
            
        // 初始化GTSAM相关变量
        gtsam::Vector Vector6(6);  // 6维向量
        gtsam::NonlinearFactorGraph graph;  // 非线性因子图
        gtsam::Values initialEstimate;  // 初始估计值

        // 获取已接收的机器人ID列表
        std::vector<int> id_received = _robot_received_list;
        std::vector<std::tuple <int, int, gtsam::Pose3>> trans_list;  // 变换列表

        // 如果存在地图变换，将当前机器人ID加入接收列表
        if (_global_map_trans.size() != 0)
            id_received.push_back(_robot_id_th);

        // 设置初始位姿和噪声模型
        Vector6 << 1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8;
        auto odometryNoise0 = gtsam::noiseModel::Diagonal::Variances(Vector6);
        // 添加先验因子
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(0, 0, 0)), odometryNoise0));
        initialEstimate.insert(0, gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(0, 0, 0)));

        bool ill_posed = true;  // 标记问题是否适定

        // 添加地图变换约束
        for(auto ite : _global_map_trans){
            int id_0 = std::min(ite.first, _robot_id_th);
            int id_1 = std::max(ite.first, _robot_id_th);

            // 添加每个测量值作为约束
            for(auto ite_measure : ite.second){
                PointTypePose pclpose = ite_measure;
                gtsam::Pose3 measurement = gtsam::Pose3(gtsam::Rot3::RzRyRx(pclpose.roll, pclpose.pitch, pclpose.yaw),
                                                       gtsam::Point3(pclpose.x, pclpose.y, pclpose.z));
                Vector6 << 1, 1, 1, 1, 1, 1;
                auto odometryNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
                graph.add(gtsam::BetweenFactor<gtsam::Pose3>(id_0, id_1, measurement, odometryNoise));
            }

            // 处理最新测量值
            PointTypePose pclpose = ite.second[ite.second.size() - 1];
            gtsam::Pose3 measurement_latest = gtsam::Pose3(gtsam::Rot3::RzRyRx(pclpose.roll, pclpose.pitch, pclpose.yaw),
                                                          gtsam::Point3(pclpose.x, pclpose.y, pclpose.z));
            if(id_0 == 0){
                initialEstimate.insert(id_1, measurement_latest);
                ill_posed = false;
            }
            else{
                trans_list.emplace_back(std::make_tuple(id_0, id_1, measurement_latest));
            }
        }

        // 添加里程计变换约束
        for(auto ite: _global_odom_trans){
            int id_publish = robotID2Number(ite.first);
            int id_child = ite.second[0].intensity;
            int id_0 = std::min(id_publish, id_child);
            int id_1 = std::max(id_publish, id_child);

            // 添加每个测量值作为约束
            for(auto ite_measure: ite.second){
                PointTypePose pclpose = ite_measure;
                gtsam::Pose3 measurement = gtsam::Pose3(gtsam::Rot3::RzRyRx(pclpose.roll, pclpose.pitch, pclpose.yaw),
                                                       gtsam::Point3(pclpose.x, pclpose.y, pclpose.z));
                Vector6 << 1, 1, 1, 1, 1, 1;
                auto odometryNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
                graph.add(gtsam::BetweenFactor<gtsam::Pose3>(id_0, id_1, measurement, odometryNoise));
            }

            // 处理最新测量值
            PointTypePose pclpose = ite.second[ite.second.size() - 1];
            gtsam::Pose3 measurement_latest = gtsam::Pose3(gtsam::Rot3::RzRyRx(pclpose.roll, pclpose.pitch, pclpose.yaw),
                                                      gtsam::Point3(pclpose.x, pclpose.y, pclpose.z));
            if(id_0 == 0){
                initialEstimate.insert(id_1, measurement_latest);
                ill_posed = false;
            }
            else{
                trans_list.emplace_back(std::make_tuple(id_0, id_1, measurement_latest));
            }

            // 更新接收到的机器人ID列表
            if(find(id_received.begin(), id_received.end(), id_0) == id_received.end())
                id_received.push_back(id_0);

            if(find(id_received.begin(), id_received.end(), id_1) == id_received.end())
                id_received.push_back(id_1);
        }
        // 检查当前机器人ID是否在接收列表中，不在则直接返回
        if (find(id_received.begin(), id_received.end(), _robot_id_th) == id_received.end()){
            return;
        }
        // 检查问题是否适定，不适定则直接返回
        if (ill_posed)
            return;

        // 初始化终止信号标志
        bool terminate_signal = false;
        // 循环处理直到所有位姿估计完成
        while (!terminate_signal){
            // 如果接收列表为空则退出循环
            if (id_received.size() == 0)
                break;

            terminate_signal = true;  // 假设本次循环可以终止
            
            // 遍历接收列表中的机器人ID
            for(auto id = id_received.begin(); id != id_received.end();)
            {
                int id_this = *id;
                // 如果该ID已有估计值，则从列表中移除
                if(initialEstimate.exists(id_this)){
                    id = id_received.erase(id);
                    continue;
                }
                else
                    ++id;

                // 在变换列表中查找包含该ID的变换
                auto it = std::find_if(trans_list.begin(), trans_list.end(),
                                       [id_this](auto& e)
                                       {return std::get<0>(e) == id_this || std::get<1>(e) == id_this;});

                if(it == trans_list.end())
                    continue;

                // 计算关联的另一个机器人ID
                int id_t = get<0>(*it) + get<1>(*it) - id_this;

                // 如果关联ID没有估计值则跳过
                if(!initialEstimate.exists(id_t))
                    continue;
                    
                // 获取关联ID的位姿估计
                gtsam::Pose3 pose_t = initialEstimate.at<gtsam::Pose3>(id_t);
                
                // 根据ID在变换中的位置计算新的位姿估计
                if ( id_this == get<1>(*it)){
                    gtsam::Pose3 pose_f = pose_t * get<2>(*it);
                    initialEstimate.insert(id_this, pose_f);


                    terminate_signal = false;  // 有新估计值产生，不能终止
                }
                if ( id_this == get<0>(*it)){
                    gtsam::Pose3 pose_f = pose_t * get<2>(*it).inverse();
                    initialEstimate.insert(id_this, pose_f);

                    terminate_signal = false;  // 有新估计值产生，不能终止
                }
            }
        }

        // 为剩余未估计的ID设置默认位姿
        for (auto it : id_received){
            initialEstimate.insert(it, gtsam::Pose3( gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(0, 0, 0) ));
        }

        // 清空临时列表
        id_received.clear();
        trans_list.clear();

        // 执行因子图优化
        gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initialEstimate).optimize();

        // 清空临时变量
        initialEstimate.clear();
        graph.resize(0);

        // 获取当前机器人的优化结果
        gtsam::Pose3 est = result.at<gtsam::Pose3>(_robot_id_th);

        // 将优化结果存储到发布变量中
        _trans_to_publish.x = est.translation().x();
        _trans_to_publish.y = est.translation().y();
        _trans_to_publish.z = est.translation().z();
        _trans_to_publish.roll = est.rotation().roll();
        _trans_to_publish.pitch = est.rotation().pitch();
        _trans_to_publish.yaw = est.rotation().yaw();
        // 检查变换是否有效，设置强度标志
        if (_trans_to_publish.x != 0 || _trans_to_publish.y != 0 || _trans_to_publish.z != 0)
            _trans_to_publish.intensity = 1;

        if (_trans_to_publish.intensity == 1){
            int robot_id_initial = robotID2Number(_robot_initial);
            if (_global_map_trans_optimized.find(robot_id_initial) == _global_map_trans_optimized.end()){
                _global_map_trans_optimized.emplace(std::make_pair(robot_id_initial, _trans_to_publish));
            }


            else{
                _global_map_trans[robot_id_initial].push_back(_trans_to_publish);
                _global_map_trans_optimized[robot_id_initial] = _trans_to_publish;
                }
        }
    }

        /**
     * @brief 扫描上下文信息处理函数
     * @param msgIn 输入的扫描上下文信息消息
     * 
     * 1. 检查通信信号和目标机器人ID是否匹配
     * 2. 从消息中提取扫描上下文数据
     * 3. 构建ScanContextBin结构体
     * 4. 调用run函数处理扫描上下文数据
     */
    void scanContextInfoHandler(const disco_slam::context_infoConstPtr& msgIn){
        // 复制消息内容到本地变量
        disco_slam::context_info context_info_input = *msgIn;
        
        // 检查通信信号是否有效
        if (!_communication_signal)
            return;
            
        // 检查消息是否发送给当前机器人
        if (msgIn->robotIDReceive != _robot_id)
            return;

        // 初始化扫描上下文bin结构体
        ScanContextBin bin;
        
        // 设置机器人ID和时间戳
        bin.robotname = msgIn->robotID;
        bin.time = msgIn->header.stamp.toSec();

        // 设置位姿信息
        bin.pose.x = msgIn->poseX;        // X坐标
        bin.pose.y = msgIn->poseY;        // Y坐标
        bin.pose.z = msgIn->poseZ;        // Z坐标
        bin.pose.roll  = msgIn->poseRoll;  // 滚转角
        bin.pose.pitch = msgIn->posePitch; // 俯仰角
        bin.pose.yaw   = msgIn->poseYaw;   // 偏航角
        bin.pose.intensity = msgIn->poseIntensity; // 强度/标志位

        // 初始化点云并填充数据
        bin.cloud.reset(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(msgIn->scanCloud, *bin.cloud);

        // 初始化扫描上下文矩阵和环键向量
        bin.bin = Eigen::MatrixXf::Zero(_knn_feature_dim, _num_sectors);
        bin.ringkey = Eigen::VectorXf::Zero(_knn_feature_dim);
        
        // 填充扫描上下文数据
        int cnt = 0;
        for (int i=0; i<msgIn->numRing; i++){
            bin.ringkey(i) = msgIn->ringKey[i];  // 设置环键值
            for (int j=0; j<msgIn->numSector; j++){
                bin.bin(i,j) = msgIn->scanContextBin[cnt];  // 设置扫描上下文值
                ++cnt;
            }
        }

        // 调用run函数处理扫描上下文
        run(bin);
    }


        /**
     * @brief 处理扫描上下文数据的主流程函数
     * @param bin 输入的扫描上下文数据结构体
     * 
     * 1. 构建KD树用于快速搜索
     * 2. 执行K近邻搜索寻找相似扫描
     * 3. 如果K近邻搜索无结果且启用位置搜索，则执行距离搜索
     * 4. 获取初始位姿估计
     * 5. 执行增量式PCM(概率一致性测量)验证
     * 6. 进行GTSAM因子图优化
     * 7. 发送优化后的变换结果
     */
    void run(ScanContextBin bin){
        // 构建KD树用于后续搜索
        buildKDTree(bin);
        
        // 执行K近邻搜索寻找相似扫描上下文
        KNNSearch(bin);

        // 如果K近邻搜索无结果且启用位置搜索，则执行基于距离的搜索
        if(_idx_nearest_list.empty() && _use_position_search)
            distanceSearch(bin);

        // 获取初始位姿估计，失败则直接返回
        if(!getInitialGuesses(bin)){
            return;
        }

        // 执行增量式PCM验证，失败则直接返回
        if(!incrementalPCM()){
            return;
        }

        // 进行GTSAM因子图优化
        gtsamExpressionGraph();

        // 发送优化后的变换结果
        sendOdomOutputMessage();
    }


        /**
     * @brief 基于距离的搜索函数，用于在KD树中查找最近的位姿点
     * @param bin 输入的扫描上下文数据结构体
     * 
     * 1. 根据机器人ID判断搜索模式
     * 2. 对于其他机器人：转换查询点到目标机器人坐标系
     * 3. 对于当前机器人：转换其他机器人点到当前坐标系
     * 4. 执行半径搜索并存储匹配结果
     */
    void distanceSearch(ScanContextBin bin){
        // 获取当前机器人ID的数字形式
        int id_this = robotID2Number(bin.robotname);
        
        // 处理其他机器人的情况
        if(bin.robotname != _robot_id){
            // 检查目标机器人是否有优化后的位姿
            if(_global_map_trans_optimized.find(id_this) == _global_map_trans_optimized.end())
                return;
                
            // 初始化查询点和结果容器
            PointType pt_query;
            std::vector<int> idx_list;
            std::vector<float> dist_list;
            
            // 获取目标机器人的优化位姿
            PointTypePose pose_this = _global_map_trans_optimized[id_this];
            gtsam::Pose3 T_pose_this = gtsam::Pose3(gtsam::Rot3::RzRyRx(pose_this.roll, pose_this.pitch, pose_this.yaw),
                            gtsam::Point3(pose_this.x, pose_this.y, pose_this.z));
            
            // 将查询点转换到目标机器人坐标系
            auto T_query = gtsam::Pose3(gtsam::Rot3::RzRyRx(bin.pose.roll, bin.pose.pitch, bin.pose.yaw),
                                        gtsam::Point3(bin.pose.x, bin.pose.y, bin.pose.z));
            T_query = T_pose_this.inverse() * T_query;

            // 设置查询点坐标
            pt_query.x = T_query.translation().x(); 
            pt_query.y = T_query.translation().y(); 
            pt_query.z = T_query.translation().z();

            // 执行半径搜索(5米范围内)
            _kdtree_pose_to_search->setInputCloud(_cloud_pose_to_search_this);
            _kdtree_pose_to_search->radiusSearch(pt_query, 5, idx_list, dist_list, 0);
            
            // 存储匹配结果
            if (!idx_list.empty()){
                int tmp_id = _cloud_pose_to_search_this->points[idx_list[0]].intensity;
                _idx_nearest_list.emplace_back(std::make_pair(tmp_id, 0));
            }
        }
        // 处理当前机器人的情况
        else{
            // 初始化查询点和结果容器
            PointType pt_query;
            std::vector<int> idx_list;
            std::vector<float> dist_list;
            
            // 创建临时点云容器
            pcl::PointCloud<PointType>::Ptr cloud_pose_to_search_other_copy(new pcl::PointCloud<PointType>());
            
            // 转换其他机器人点到当前坐标系
            for (unsigned int i = 0; i < _cloud_pose_to_search_other->size(); i++){
                PointType tmp = _cloud_pose_to_search_other->points[i];
                int id_this = robotID2Number(_bin_with_id[tmp.intensity].robotname);
                
                // 跳过没有优化位姿的机器人
                if(_global_map_trans_optimized.find(id_this) == _global_map_trans_optimized.end())
                    continue;
                    
                // 获取并转换位姿
                PointTypePose pose_this = _global_map_trans_optimized[id_this];
                gtsam::Pose3 T_pose_this = gtsam::Pose3(gtsam::Rot3::RzRyRx(pose_this.roll, pose_this.pitch, pose_this.yaw),
                                                        gtsam::Point3(pose_this.x, pose_this.y, pose_this.z));
                auto T_this = gtsam::Point3(tmp.x,tmp.y,tmp.z);
                T_this = T_pose_this.inverse() * T_this;
                
                // 更新点坐标
                tmp.x = T_this.x();
                tmp.y = T_this.y();
                tmp.z = T_this.z();
                cloud_pose_to_search_other_copy->push_back(tmp);
            }
            
            // 设置查询点坐标(当前机器人位姿)
            pt_query.x = bin.pose.x;
            pt_query.y = bin.pose.y;
            pt_query.z = bin.pose.z;
            
            // 跳过空点云
            if (cloud_pose_to_search_other_copy->empty())
                return;

            // 执行半径搜索(10米范围内)
            _kdtree_pose_to_search->setInputCloud(cloud_pose_to_search_other_copy);
            _kdtree_pose_to_search->radiusSearch(pt_query, 10, idx_list, dist_list, 0);
            
            // 存储匹配结果
            if (!idx_list.empty()){
                for (unsigned int i = 0; i< cloud_pose_to_search_other_copy->size(); i++){
                    int tmp_id = cloud_pose_to_search_other_copy->points[idx_list[i]].intensity;
                    if(tmp_id == _num_bin)
                        continue;
                    _idx_nearest_list.emplace_back(std::make_pair(tmp_id, 0));
                    break;
                }
            }
            // 清空临时点云
            cloud_pose_to_search_other_copy->clear();
        }
    }


        /**
     * @brief 构建KD树的函数，用于存储和索引扫描上下文数据
     * @param bin 输入的扫描上下文数据结构体
     * 
     * 1. 增加bin计数器并存储接收到的数据
     * 2. 根据机器人ID将位姿点存入不同的点云容器
     * 3. 更新目标矩阵并重建KD树
     */
    void buildKDTree(ScanContextBin bin){
        // 增加bin计数器
        _num_bin++;
        
        // 存储接收到的数据，key为bin索引，value为扫描上下文数据
        _bin_with_id.emplace( std::make_pair(_num_bin-1, bin) );

        // 创建位姿点并设置坐标和索引
        PointType tmp_pose;
        tmp_pose.x = bin.pose.x; 
        tmp_pose.y = bin.pose.y; 
        tmp_pose.z = bin.pose.z;
        tmp_pose.intensity = _num_bin - 1;  // 使用intensity字段存储bin索引

        // 根据机器人ID将位姿点存入不同的点云容器
        if (bin.robotname == _robot_id)
            _cloud_pose_to_search_this->push_back(tmp_pose);  // 当前机器人的位姿点
        else
            _cloud_pose_to_search_other->push_back(tmp_pose); // 其他机器人的位姿点

        // 调整目标矩阵大小以容纳新的环键数据
        _target_matrix.conservativeResize(_knn_feature_dim, _num_bin);
        
        // 将新的环键数据添加到目标矩阵
        _target_matrix.block(0, _num_bin-1, _knn_feature_dim, 1) =
            bin.ringkey.block(0, 0, _knn_feature_dim, 1);
            
        // 使用更新后的目标矩阵重建KD树
        _nns = Nabo::NNSearchF::createKDTreeLinearHeap(_target_matrix);
    }


        /**
     * @brief 执行K近邻搜索函数，寻找与当前扫描上下文最相似的候选
     * @param bin 输入的扫描上下文数据结构体
     * 
     * 1. 检查是否有足够的候选bin
     * 2. 执行K近邻搜索获取候选索引和距离
     * 3. 过滤无效候选(同机器人、无关机器人等)
     * 4. 计算完整扫描上下文距离
     * 5. 排序并存储最佳匹配结果
     */
    void KNNSearch(ScanContextBin bin){
        // 检查候选bin数量是否足够
        if (_num_nearest_matches >= _num_bin){
            return; // 候选不足直接返回
        }

        // 设置近邻搜索数量
        int num_neighbors = _num_nearest_matches;

        // 初始化结果容器
        Eigen::VectorXi indices(num_neighbors);    // 候选索引
        Eigen::VectorXf dists2(num_neighbors);    // 候选距离

        // 执行K近邻搜索(基于环键特征)
        _nns->knn(bin.ringkey, indices, dists2, num_neighbors);

        // 初始化变量
        int idx_candidate, rot_idx;
        float distance_to_query;
        // 候选列表: (距离, bin索引, 旋转索引)
        std::vector<std::tuple<float, int, int>> idx_list;

        // 遍历所有候选结果
        for (int i = 0; i < std::min(num_neighbors, int(indices.size())); ++i){
            // 检查搜索是否正常工作
            if (indices.sum() == 0)
                continue;

            // 检查候选索引是否有效
            idx_candidate = indices[i];
            if (idx_candidate >= _num_bin)
                continue;

            // 跳过同机器人的候选
            if (bin.robotname == _bin_with_id.at(idx_candidate).robotname)
                continue;

            // 跳过与当前机器人无关的匹配对
            if (bin.robotname != _robot_id && _bin_with_id.at(idx_candidate).robotname != _robot_id)
                continue;

            // 跳过特定ID范围的机器人
            if(robotID2Number(bin.robotname) >= _robot_id_th && 
               robotID2Number(_bin_with_id.at(idx_candidate).robotname) >= _robot_id_th)
                continue;

            // 计算完整扫描上下文距离
            distance_to_query = distBtnScanContexts(bin.bin, _bin_with_id.at(idx_candidate).bin, rot_idx);

            // 跳过距离超过阈值的候选
            if(distance_to_query > _loop_thres)
                continue;

            // 添加到候选列表
            idx_list.emplace_back(std::make_tuple(distance_to_query, idx_candidate, rot_idx));
        }

        // 清空之前的最近邻列表
        _idx_nearest_list.clear();

        // 无有效候选则返回
        if (idx_list.size() == 0)
            return;

        // 按距离排序候选列表
        std::sort(idx_list.begin(), idx_list.end());
        
        // 选择前N个最佳匹配
        for (int i = 0; i < std::min(_num_match_candidates, int(idx_list.size())); i++){
            std::tie(distance_to_query, idx_candidate, rot_idx) = idx_list[i];
            _idx_nearest_list.emplace_back(std::make_pair(idx_candidate, rot_idx));
        }
        
        // 清空临时列表
        idx_list.clear();
    }


        /**
     * @brief 获取初始位姿估计的函数
     * @param bin 输入的扫描上下文数据结构体
     * @return 如果成功获取至少一个有效初始估计返回true，否则返回false
     * 
     * 1. 检查最近邻列表是否为空
     * 2. 遍历所有最近邻候选
     * 3. 对每个候选调用getInitialGuess获取初始估计
     * 4. 返回是否有新候选的标志
     */
    bool getInitialGuesses(ScanContextBin bin){
        // 检查最近邻列表是否为空
        if(_idx_nearest_list.size() == 0){
            return false;
        }
        
        // 初始化新候选标志
        bool new_candidate_signal = false;
        
        // 遍历所有最近邻候选
        for (auto it: _idx_nearest_list){
            // 对每个候选获取初始估计
            new_candidate_signal = getInitialGuess(bin, it.first, it.second);
        }
        
        // 返回是否有新候选的标志
        return new_candidate_signal;
    }


    /**
     * @brief 获取单个候选的初始位姿估计
     * @param bin 当前扫描上下文数据
     * @param idx_nearest 最近邻bin的索引
     * @param min_idx 最小旋转索引
     * @return 如果成功获取有效初始估计返回true，否则返回false
     * 
     * 1. 计算扫描上下文旋转角度
     * 2. 处理机器人ID交换逻辑
     * 3. 获取目标位姿
     * 4. 计算初始位姿估计(三种情况)
     * 5. 执行ICP配准验证
     */
    bool getInitialGuess(ScanContextBin bin, int idx_nearest, int min_idx){
        // 初始化bin索引
        int id0 = idx_nearest, id1 = _num_bin - 1;

        // 声明变量
        ScanContextBin bin_nearest;
        PointTypePose source_pose_initial, target_pose;

        // 计算扫描上下文旋转角度(弧度)
        float sc_pitch = (min_idx+1) * M_PI * 2 /_num_sectors;
        if (sc_pitch > M_PI)
            sc_pitch -= (M_PI * 2);  // 规范化到[-π,π]

        // 获取当前机器人ID的数字形式
        int robot_id_this = robotID2Number(bin.robotname);

        // 检查是否已记录该机器人ID
        auto robot_id_this_ite = std::find(_robot_received_list.begin(), _robot_received_list.end(), robot_id_this);

        // 记录新接收的机器人ID
        if (robot_id_this_ite == _robot_received_list.end() && robot_id_this != _robot_id_th)
            _robot_received_list.push_back(robot_id_this);

        // 处理机器人ID交换逻辑(优先级高的机器人交换bin)
        if (robot_id_this < _robot_id_th){
            bin_nearest = bin;
            bin = _bin_with_id.at(idx_nearest);

            id0 = _num_bin - 1;
            id1 = idx_nearest;
            sc_pitch = -sc_pitch;  // 反转旋转角度
        }
        else{
            bin_nearest = _bin_with_id.at(idx_nearest);
        }

        // 设置当前处理的机器人信息
        _robot_this = bin_nearest.robotname;
        _robot_this_th = robotID2Number(_robot_this);

        // 从扫描上下文获取目标位姿
        target_pose = bin_nearest.pose;

        // 情况1: 已有优化后的全局变换
        if (_global_map_trans_optimized.find(_robot_this_th) != _global_map_trans_optimized.end()){
            PointTypePose trans_to_that = _global_map_trans_optimized[_robot_this_th];
            // 计算变换矩阵
            Eigen::Affine3f t_source2target = pcl::getTransformation(trans_to_that.x, trans_to_that.y, trans_to_that.z,
                                                               trans_to_that.roll, trans_to_that.pitch, trans_to_that.yaw);
            Eigen::Affine3f t_source = pcl::getTransformation(bin.pose.x, bin.pose.y, bin.pose.z, 
                                                              bin.pose.roll, bin.pose.pitch, bin.pose.yaw);
            // 计算初始位姿
            Eigen::Affine3f t_initial_source = t_source2target * t_source;
            pcl::getTranslationAndEulerAngles(t_initial_source, source_pose_initial.x, source_pose_initial.y, source_pose_initial.z,
                                              source_pose_initial.roll, source_pose_initial.pitch, source_pose_initial.yaw);
        }
        // 情况2: 旋转角度很小，直接使用目标位姿
        else if(abs(sc_pitch) < 0.3){
            source_pose_initial = target_pose;
        }
        // 情况3: 使用扫描上下文旋转角度计算初始位姿
        else{
            Eigen::Affine3f sc_initial = pcl::getTransformation(0, 0, 0, 0, 0, sc_pitch);
            Eigen::Affine3f t_target = pcl::getTransformation(target_pose.x, target_pose.y, target_pose.z,
                                                              target_pose.roll, target_pose.pitch, target_pose.yaw);
            Eigen::Affine3f t_initial_source = sc_initial * t_target;
            pcl::getTranslationAndEulerAngles(t_initial_source, source_pose_initial.x, source_pose_initial.y, source_pose_initial.z,
                                              source_pose_initial.roll, source_pose_initial.pitch, source_pose_initial.yaw);
            // 保持位置不变，仅使用旋转
            source_pose_initial.x = target_pose.x;
            source_pose_initial.y = target_pose.y;
            source_pose_initial.z = target_pose.z;
        }

        // 执行ICP配准验证初始位姿
        PointTypePose pose_source_lidar = icpRelativeMotion(
            transformPointCloud(bin.cloud, &source_pose_initial),
            transformPointCloud(bin_nearest.cloud, &target_pose), 
            source_pose_initial);

        // 检查ICP结果是否有效
        if (pose_source_lidar.intensity == -1 || pose_source_lidar.intensity > _icp_thres)
            return false;

        // 准备GTSAM位姿数据
        // 将源位姿转换为GTSAM格式
        gtsam::Pose3 pose_from = gtsam::Pose3(
            gtsam::Rot3::RzRyRx(bin.pose.roll, bin.pose.pitch, bin.pose.yaw),
            gtsam::Point3(bin.pose.x, bin.pose.y, bin.pose.z));

        // 将ICP优化后的位姿转换为GTSAM格式
        gtsam::Pose3 pose_to =
            gtsam::Pose3(gtsam::Rot3::RzRyRx(pose_source_lidar.roll, pose_source_lidar.pitch, pose_source_lidar.yaw),
                  gtsam::Point3(pose_source_lidar.x, pose_source_lidar.y, pose_source_lidar.z));

        // 将目标位姿转换为GTSAM格式
        gtsam::Pose3 pose_target =
            gtsam::Pose3(gtsam::Rot3::RzRyRx(target_pose.roll, target_pose.pitch, target_pose.yaw),
                  gtsam::Point3(target_pose.x, target_pose.y, target_pose.z));

        // 检查当前机器人是否已有位姿队列，没有则创建
        auto ite = _pose_queue.find(_robot_this_th);
        if(ite == _pose_queue.end()){
            // 初始化新的位姿队列和闭环队列
            std::vector< std::tuple<gtsam::Pose3, gtsam::Pose3, float> > new_pose_queue;
            std::vector< std::tuple<int, int, gtsam::Pose3> > new_loop_queue;
            _pose_queue.emplace( std::make_pair(_robot_this_th, new_pose_queue) );
            _loop_queue.emplace( std::make_pair(_robot_this_th, new_loop_queue) );
        }

        // 将当前位姿对和ICP分数加入位姿队列
        _pose_queue[_robot_this_th].emplace_back(std::make_tuple(pose_from, pose_to, pose_source_lidar.intensity));
        
        // 计算相对位姿变换并加入闭环队列
        _loop_queue[_robot_this_th].emplace_back(std::make_tuple(id0, id1, pose_to.between(pose_target)));

        return true;  // 返回成功标志

    }

        /**
     * @brief 计算两个扫描上下文之间的距离
     * @param bin1 第一个扫描上下文矩阵
     * @param bin2 第二个扫描上下文矩阵
     * @param idx [输出] 最大相似度对应的旋转索引
     * @return 两个扫描上下文之间的距离(1-最大相似度)
     * 
     * 1. 对bin1进行循环移位，计算每个旋转位置的相似度
     * 2. 对每个扇区计算余弦相似度
     * 3. 找到最大相似度及其对应的旋转索引
     * 4. 返回距离值(1-最大相似度)
     */
    float distBtnScanContexts(Eigen::MatrixXf bin1, Eigen::MatrixXf bin2, int & idx){
        // 初始化存储每个旋转位置相似度的向量
        Eigen::VectorXf sim_for_each_cols(_num_sectors);
        
        // 遍历所有可能的旋转位置
        for (int i = 0; i<_num_sectors; i++) {
            // 对bin1进行循环移位
            int one_step = 1;
            Eigen::MatrixXf bint = circShift(bin1, one_step);
            bin1 = bint;

            // 初始化相似度累加和和有效列计数器
            float sum_of_cos_sim = 0;
            int num_col_engaged = 0;

            // 遍历所有扇区计算相似度
            for (int j = 0; j < _num_sectors; j++) {
                // 提取当前扇区的列向量
                Eigen::VectorXf col_j_1(_knn_feature_dim);
                Eigen::VectorXf col_j_2(_knn_feature_dim);
                col_j_1.block(0, 0, _knn_feature_dim, 1) = bin1.block(0, j, _knn_feature_dim, 1);
                col_j_2.block(0, 0, _knn_feature_dim, 1) = bin2.block(0, j, _knn_feature_dim, 1);

                // 跳过空列
                if(col_j_1.isZero() || col_j_2.isZero())
                    continue;

                // 计算余弦相似度
                float cos_similarity = col_j_1.dot(col_j_2) / col_j_1.norm() / col_j_2.norm();
                sum_of_cos_sim += cos_similarity;
                num_col_engaged++;
            }
            
            // 计算平均相似度(考虑有效列数)
            sim_for_each_cols(i) = sum_of_cos_sim / float(num_col_engaged);
        }
        
        // 找到最大相似度及其索引
        Eigen::VectorXf::Index idx_max;
        float sim = sim_for_each_cols.maxCoeff(& idx_max);
        idx = idx_max;
        
        // 返回距离值(1-最大相似度)
        float dist = 1-sim;
        return dist;
    }


        /**
     * @brief 点云变换函数
     * @param cloudIn 输入点云指针
     * @param transformIn 变换位姿指针(包含x,y,z,roll,pitch,yaw)
     * @return 变换后的点云指针
     * 
     * 1. 创建输出点云并预分配空间
     * 2. 从位姿参数构建仿射变换矩阵
     * 3. 使用OpenMP并行处理每个点
     * 4. 应用变换矩阵计算新坐标
     * 5. 保留原始点强度信息
     */
    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        // 创建输出点云对象
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        PointType *pointFrom;  // 临时点指针

        // 获取输入点云大小并预分配输出空间
        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        // 从位姿参数构建3D仿射变换矩阵
        Eigen::Affine3f transCur = pcl::getTransformation(
            transformIn->x, transformIn->y, transformIn->z,
            transformIn->roll, transformIn->pitch, transformIn->yaw);

        // 使用OpenMP并行处理点云(线程数由numberOfCores指定)
#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            // 获取当前输入点
            pointFrom = &cloudIn->points[i];
            
            // 应用变换矩阵计算新坐标(x,y,z分量)
            cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
            
            // 保留原始点强度信息
            cloudOut->points[i].intensity = pointFrom->intensity;
        }
        
        // 返回变换后的点云
        return cloudOut;
    }


        /**
     * @brief 使用ICP算法计算相对位姿变换
     * @param source 源点云(待配准点云)
     * @param target 目标点云(参考点云)
     * @param pose_source 源点云的初始位姿估计
     * @return 返回配准后的位姿(包含变换后的位置和姿态)
     * 
     * 1. 配置ICP参数(最大距离、迭代次数、收敛条件等)
     * 2. 对输入点云进行降采样处理
     * 3. 执行ICP配准
     * 4. 计算并返回校正后的位姿
     */
    PointTypePose icpRelativeMotion(pcl::PointCloud<PointType>::Ptr source,
                                   pcl::PointCloud<PointType>::Ptr target,
                                   PointTypePose pose_source)
    {
        // 配置ICP算法参数
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(100);  // 设置最大对应点距离(米)
        icp.setMaximumIterations(100);          // 设置最大迭代次数
        icp.setTransformationEpsilon(1e-6);     // 设置变换收敛阈值
        icp.setEuclideanFitnessEpsilon(1e-6);   // 设置欧式距离误差收敛阈值
        icp.setRANSACIterations(0);             // 禁用RANSAC(0表示不启用)

        // 创建临时点云并执行降采样
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        _downsize_filter_icp.setInputCloud(source);  // 设置源点云输入
        _downsize_filter_icp.filter(*cloud_temp);    // 执行降采样滤波
        *source = *cloud_temp;                      // 更新源点云

        _downsize_filter_icp.setInputCloud(target);  // 设置目标点云输入
        _downsize_filter_icp.filter(*cloud_temp);     // 执行降采样滤波
        *target = *cloud_temp;                       // 更新目标点云

        // 执行ICP配准
        icp.setInputSource(source);  // 设置源点云
        icp.setInputTarget(target);  // 设置目标点云
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);   // 执行配准
        PointTypePose pose_from;     // 存储结果位姿

        // 检查ICP是否收敛
        if (icp.hasConverged() == false){
            pose_from.intensity = -1;  // 设置无效标志
            return pose_from;          // 返回无效结果
        }

        // 获取ICP变换矩阵并转换为欧拉角
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();  // 获取最终变换矩阵
        pcl::getTranslationAndEulerAngles(correctionLidarFrame, x, y, z, roll, pitch, yaw);

        // 计算校正后的位姿(世界坐标系到校正后位姿的变换)
        Eigen::Affine3f tWrong = pcl::getTransformation(
            pose_source.x, pose_source.y, pose_source.z,
            pose_source.roll, pose_source.pitch, pose_source.yaw);
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;  // 组合变换

        // 将校正后的位姿转换为欧拉角表示
        pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);

        // 填充返回的位姿结构体
        pose_from.x = x;
        pose_from.y = y;
        pose_from.z = z;
        pose_from.yaw = yaw;
        pose_from.roll = roll;
        pose_from.pitch = pitch;
        pose_from.intensity = icp.getFitnessScore();  // 存储ICP匹配分数

        return pose_from;  // 返回校正后的位姿
    }


        /**
     * @brief 增量式概率一致性测量(PCM)验证函数
     * @return 如果检测到新的最大团返回true，否则返回false
     * 
     * 1. 检查位姿队列是否达到PCM启动阈值
     * 2. 计算一致性矩阵并输出到文件
     * 3. 使用启发式算法寻找最大团
     * 4. 更新闭环接受队列
     */
    bool incrementalPCM() {
        // 检查位姿队列大小是否达到PCM启动阈值
        if (_pose_queue[_robot_this_th].size() < _pcm_start_threshold)
            return false;

        // 计算闭环队列的一致性矩阵
        Eigen::MatrixXi consistency_matrix = computePCMMatrix(_loop_queue[_robot_this_th]);
        
        // 生成一致性矩阵文件路径并输出图形
        std::string consistency_matrix_file = _pcm_matrix_folder + "/consistency_matrix" + _robot_id + ".clq.mtx";
        printPCMGraph(consistency_matrix, consistency_matrix_file);
        
        // 读取图形文件并计算最大团
        FMC::CGraphIO gio;
        gio.readGraph(consistency_matrix_file);
        int max_clique_size = 0;
        std::vector<int> max_clique_data;

        // 使用启发式算法寻找最大团
        max_clique_size = FMC::maxCliqueHeu(gio, max_clique_data);

        // 对最大团数据进行排序
        std::sort(max_clique_data.begin(), max_clique_data.end());

        // 查找当前机器人的闭环接受队列
        auto loop_accept_queue_this = _loop_accept_queue.find(_robot_this_th);
        
        // 如果不存在则创建新条目
        if (loop_accept_queue_this == _loop_accept_queue.end()){
            _loop_accept_queue.emplace(std::make_pair(_robot_this_th, max_clique_data));
            return true;  // 返回有新最大团标志
        }

        // 如果最大团数据未变化则返回false
        if(max_clique_data == loop_accept_queue_this->second)
            return false;

        // 更新闭环接受队列
        _loop_accept_queue[_robot_this_th].clear();
        _loop_accept_queue[_robot_this_th] = max_clique_data;
        return true;  // 返回有新最大团标志
    }


        /**
     * @brief 计算概率一致性测量(PCM)矩阵
     * @param loop_queue_this 当前机器人的闭环队列，包含位姿对信息
     * @return 返回PCM矩阵，1表示一致性匹配，0表示不一致
     * 
     * 1. 初始化PCM矩阵为全零矩阵
     * 2. 遍历所有闭环对组合
     * 3. 计算每对闭环的相对位姿变换
     * 4. 计算残差并判断是否满足一致性阈值
     * 5. 填充PCM矩阵
     */
    Eigen::MatrixXi computePCMMatrix(std::vector<std::tuple<int, int, gtsam::Pose3>> loop_queue_this) {
        // 初始化PCM矩阵，大小与闭环队列相同
        Eigen::MatrixXi PCMMat;
        PCMMat.setZero(loop_queue_this.size(), loop_queue_this.size());
        
        // 声明变量用于存储位姿和变换
        int id_0, id_1;
        gtsam::Pose3 z_aj_bk, z_ai_bl;
        gtsam::Pose3 z_ai_aj, z_bk_bl;
        gtsam::Pose3 t_ai, t_aj, t_bk, t_bl;

        // 遍历所有闭环对组合
        for (unsigned int i = 0; i < loop_queue_this.size(); i++) {
            // 获取第i个闭环对的信息
            std::tie(id_0, id_1, z_aj_bk) = loop_queue_this[i];
            
            // 获取对应bin的位姿信息
            PointTypePose tmp_pose_0 = _bin_with_id.at(id_0).pose;
            PointTypePose tmp_pose_1 = _bin_with_id.at(id_1).pose;
            
            // 将位姿转换为GTSAM格式
            t_aj = gtsam::Pose3(gtsam::Rot3::RzRyRx(tmp_pose_0.roll, tmp_pose_0.pitch, tmp_pose_0.yaw),
                                gtsam::Point3(tmp_pose_0.x, tmp_pose_0.y, tmp_pose_0.z));
            t_bk = gtsam::Pose3(gtsam::Rot3::RzRyRx(tmp_pose_1.roll, tmp_pose_1.pitch, tmp_pose_1.yaw),
                                gtsam::Point3(tmp_pose_1.x, tmp_pose_1.y, tmp_pose_1.z));

            // 与后续所有闭环对进行比较
            for (unsigned int j = i + 1; j < loop_queue_this.size(); j++) {
                // 获取第j个闭环对的信息
                std::tie(id_0, id_1, z_ai_bl) = loop_queue_this[j];
                
                // 获取对应bin的位姿信息
                PointTypePose tmp_pose_0 = _bin_with_id.at(id_0).pose;
                PointTypePose tmp_pose_1 = _bin_with_id.at(id_1).pose;
                
                // 将位姿转换为GTSAM格式
                t_ai = gtsam::Pose3(gtsam::Rot3::RzRyRx(tmp_pose_0.roll, tmp_pose_0.pitch, tmp_pose_0.yaw),
                                    gtsam::Point3(tmp_pose_0.x, tmp_pose_0.y, tmp_pose_0.z));
                t_bl = gtsam::Pose3(gtsam::Rot3::RzRyRx(tmp_pose_1.roll, tmp_pose_1.pitch, tmp_pose_1.yaw),
                                    gtsam::Point3(tmp_pose_1.x, tmp_pose_1.y, tmp_pose_1.z));
                
                // 计算相对位姿变换
                z_ai_aj = t_ai.between(t_aj);
                z_bk_bl = t_bk.between(t_bl);
                
                // 计算残差并判断一致性
                float resi = residualPCM(z_aj_bk, z_ai_bl, z_ai_aj, z_bk_bl, 1);
                if (resi < _pcm_thres)
                    PCMMat(i,j) = 1;  // 一致性匹配
                else
                    PCMMat(i,j) = 0;  // 不一致
            }
        }
        return PCMMat;
    }


        /**
     * @brief 计算概率一致性测量(PCM)的残差
     * @param inter_jk 机器人j到k的相对位姿变换
     * @param inter_il 机器人i到l的相对位姿变换 
     * @param inner_ij 机器人i到j的内部位姿变换
     * @param inner_kl 机器人k到l的内部位姿变换
     * @param intensity 权重系数
     * @return 返回计算得到的残差值
     * 
     * 1. 计算相对位姿变换的逆
     * 2. 计算残差位姿(四元数乘法链)
     * 3. 将残差位姿转换为李代数向量
     * 4. 构建协方差矩阵
     * 5. 计算加权残差范数
     */
    float residualPCM(gtsam::Pose3 inter_jk, gtsam::Pose3 inter_il, gtsam::Pose3 inner_ij, gtsam::Pose3 inner_kl, float intensity){
        // 计算inter_il的逆变换
        gtsam::Pose3 inter_il_inv = inter_il.inverse();
        
        // 计算残差位姿: inner_ij * inter_jk * inner_kl * inter_il_inv
        gtsam::Pose3 res_pose = inner_ij * inter_jk * inner_kl * inter_il_inv;
        
        // 将残差位姿转换为李代数向量(6维)
        gtsam::Vector6 res_vec = gtsam::Pose3::Logmap(res_pose);

        // 构建协方差矩阵(对角矩阵，对角线元素为intensity)
        Eigen::Matrix< double, 6, 1> v ;
        v << intensity, intensity, intensity,
            intensity, intensity, intensity;
        Eigen::Matrix< double, 6, 6> m_cov = v.array().matrix().asDiagonal();

        // 计算加权残差范数: sqrt(res_vec^T * m_cov * res_vec)
        return sqrt(res_vec.transpose()* m_cov * res_vec);
    }


        /**
     * @brief 将PCM(概率一致性测量)矩阵输出为MatrixMarket格式的文件
     * @param pcm_matrix 输入的一致性矩阵，1表示一致性匹配，0表示不匹配
     * @param file_name 输出文件的路径和名称
     * 
     * 1. 统计一致性匹配的数量
     * 2. 格式化边信息(矩阵非零元素)
     * 3. 写入MatrixMarket格式的文件头
     * 4. 写入矩阵维度和非零元素数量
     * 5. 写入边信息数据
     */
    void printPCMGraph(Eigen::MatrixXi pcm_matrix, std::string file_name) {
        // 初始化一致性匹配计数器
        int nb_consistent_measurements = 0;

        // 格式化边信息(矩阵非零元素)
        std::stringstream ss;
        // 遍历矩阵上三角部分(包括对角线)
        for (int i = 0; i < pcm_matrix.rows(); i++) {
            for (int j = i; j < pcm_matrix.cols(); j++) {
                if (pcm_matrix(i,j) == 1) {
                    // 记录边信息(索引从1开始)
                    ss << i+1 << " " << j+1 << std::endl;
                    nb_consistent_measurements++;  // 增加匹配计数
                }
            }
        }

        // 写入MatrixMarket格式文件
        std::ofstream output_file;
        output_file.open(file_name);
        // 写入文件头(对称模式矩阵)
        output_file << "%%MatrixMarket matrix coordinate pattern symmetric" << std::endl;
        // 写入矩阵维度(行数 列数 非零元素数)
        output_file << pcm_matrix.rows() << " " << pcm_matrix.cols() << " " << nb_consistent_measurements << std::endl;
        // 写入边信息数据
        output_file << ss.str();
        output_file.close();
    }


        /**
     * @brief 使用GTSAM构建表达式图并进行位姿优化
     * 
     * 1. 检查闭环接受队列是否足够大
     * 2. 初始化GTSAM变量和因子图
     * 3. 添加位姿约束因子
     * 4. 执行优化计算
     * 5. 存储优化结果并更新全局变换
     */
    void gtsamExpressionGraph(){
        // 检查闭环接受队列是否至少有2个元素
        if (_loop_accept_queue[_robot_this_th].size()<2)
            return;

        // 初始化6维向量用于噪声模型
        gtsam::Vector Vector6(6);

        // 获取最新的两个位姿作为初始估计
        gtsam::Pose3 initial_pose_0, initial_pose_1;
        initial_pose_0 = std::get<0>(_pose_queue[_robot_this_th][ _loop_accept_queue[_robot_this_th][_loop_accept_queue[_robot_this_th].size() -1] ]);
        initial_pose_1 = std::get<1>(_pose_queue[_robot_this_th][ _loop_accept_queue[_robot_this_th][_loop_accept_queue[_robot_this_th].size() -1] ]);

        // 初始化GTSAM值和因子图
        gtsam::Values initial;
        gtsam::ExpressionFactorGraph graph;

        // 创建位姿表达式变量
        gtsam::Pose3_ trans(0);

        // 设置初始估计值(相对位姿变换)
        initial.insert(0, initial_pose_1 * initial_pose_0.inverse());

        gtsam::Pose3 p, measurement;
        float noiseScore;

        // 遍历闭环接受队列中的所有位姿对
        for (auto i:_loop_accept_queue[_robot_this_th]){
            std::tie(measurement, p, noiseScore) = _pose_queue[_robot_this_th][i];

            // 跳过无效测量(噪声分数为0表示异常值)
            if(noiseScore == 0)
                continue;

            // 计算预测位姿
            gtsam::Pose3_ predicted = transformTo(trans,p);

            // 设置噪声模型(对角协方差矩阵)
            Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore,
                noiseScore;
            auto measurementNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);

            // 添加表达式因子到图中
            graph.addExpressionFactor(predicted, measurement, measurementNoise);
        }

        // 执行LM优化
        gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();

        // 获取优化结果
        gtsam::Pose3 est = result.at<gtsam::Pose3>(0);

        // 将优化结果转换为点云位姿格式
        PointTypePose map_trans_this;
        map_trans_this.x = est.translation().x();
        map_trans_this.y = est.translation().y();
        map_trans_this.z = est.translation().z();
        map_trans_this.roll  = est.rotation().roll();
        map_trans_this.pitch = est.rotation().pitch();
        map_trans_this.yaw   = est.rotation().yaw();

        // 更新全局变换映射
        auto ite = _global_map_trans.find(_robot_this_th);
        if(ite == _global_map_trans.end()){
            // 如果不存在则创建新条目
            std::vector<PointTypePose> tmp_pose_list;
            tmp_pose_list.push_back(map_trans_this);
            _global_map_trans.emplace(std::make_pair( _robot_this_th,  tmp_pose_list ) );
            _global_map_trans_optimized.emplace(std::make_pair( _robot_this_th, map_trans_this));
        }
        else{
            // 如果存在则追加新结果
            _global_map_trans[_robot_this_th].push_back(map_trans_this);
            _global_map_trans_optimized[_robot_this_th] = map_trans_this;
        }

        // 如果有里程计变换则构建因子图
        if (_global_odom_trans.size() != 0)
            gtsamFactorGraph();

        // 处理发布变换的逻辑
        if (_trans_to_publish.intensity == 0){
            ite = _global_map_trans.find(0);
            if(ite == _global_map_trans.end())
                return;
            _global_map_trans_optimized[0].intensity = 1;
            _trans_to_publish = _global_map_trans_optimized[0];
        }

        // 特殊情况下直接使用优化结果
        if (_global_map_trans.size() == 1  && _global_odom_trans.size() == 0)
            _trans_to_publish = _global_map_trans_optimized[0];

        // 清空因子图
        graph.resize(0);
    }


        /**
     * @brief 发布地图变换消息到SLAM节点
     * 
     * 1. 检查变换是否有效(强度值非0)
     * 2. 构建Odometry消息
     * 3. 设置消息头和时间戳
     * 4. 填充坐标系ID和变换数据
     * 5. 发布变换消息
     */
    void sendMapOutputMessage(){
        // 检查变换有效性(强度值为0表示无效变换)
        if (_trans_to_publish.intensity == 0)
            return;

        // 初始化Odometry消息
        nav_msgs::Odometry odom2map;
        
        // 设置消息头和时间戳(使用点云头的时间戳)
        odom2map.header.stamp = _cloud_header.stamp;
        // 设置父坐标系ID(机器人ID/扫描上下文帧)
        odom2map.header.frame_id = _robot_id + "/" + _sc_frame;
        // 设置子坐标系ID(机器人ID/扫描上下文帧/odom2map)
        odom2map.child_frame_id = _robot_id + "/" + _sc_frame + "/odom2map";
        
        // 填充位置变换数据(x,y,z坐标)
        odom2map.pose.pose.position.x = _trans_to_publish.x;
        odom2map.pose.pose.position.y = _trans_to_publish.y;
        odom2map.pose.pose.position.z = _trans_to_publish.z;
        
        // 将欧拉角转换为四元数并设置姿态变换数据
        odom2map.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw
            (_trans_to_publish.roll, _trans_to_publish.pitch, _trans_to_publish.yaw);
        
        // 发布变换消息
        _pub_trans_odom2map.publish(odom2map);
    }


        /**
     * @brief 通过KD树发送全局闭环消息
     * 
     * 1. 获取当前机器人的最新闭环信息
     * 2. 检查时间间隔是否足够(避免频繁闭环)
     * 3. 处理初始闭环情况
     * 4. 发送当前和历史的闭环信息
     * 5. 更新最后处理的bin ID
     */
    void sendGlobalLoopMessageKDTree(){
        // 获取当前机器人的闭环队列和最新闭环信息
        auto loop_list = _loop_queue[_robot_this_th];
        int len_loop_list = loop_list.size() - 1;
        auto loop_this = loop_list[len_loop_list];
        int id_bin_this = std::get<1>(loop_this);  // 获取当前bin ID

        // 处理初始闭环情况(第一次检测到闭环)
        if (_initial_loop.first == -1){
            // 记录当前机器人和闭环索引
            _initial_loop.first = _robot_this_th;
            _initial_loop.second = len_loop_list;
            // 初始化最后处理的bin ID
            _id_bin_last = id_bin_this;
            return;
        }

        // 检查时间间隔是否足够(避免频繁闭环)
        if (!compare_timestamp(id_bin_this, _id_bin_last)){
            return;  // 时间间隔不足直接返回
        }

        // 获取历史闭环信息(上次检测到的闭环)
        int tmp_robot_id_th =  _initial_loop.first;
        int tmp_len_loop_list = _initial_loop.second;
        auto loop_that = _loop_queue[tmp_robot_id_th][tmp_len_loop_list];
        int id_bin_that = std::get<1>(loop_that);

        // 发送当前和历史闭环信息
        sendLoopThis(_robot_this_th, tmp_robot_id_th, len_loop_list, tmp_len_loop_list);  // 发送当前闭环
        sendLoopThat(_robot_this_th, tmp_robot_id_th, len_loop_list, tmp_len_loop_list);  // 发送历史闭环

        // 更新最后处理的bin ID
        _id_bin_last = id_bin_this;
    }



        /**
     * @brief 比较两个bin的时间戳差异是否超过阈值
     * @param id_0 第一个bin的ID
     * @param id_1 第二个bin的ID
     * @return 如果时间差超过阈值返回true，否则返回false
     * 
     * 1. 获取两个bin的位姿强度值(存储时间戳信息)
     * 2. 计算绝对时间差
     * 3. 与预设阈值比较判断是否有效
     */
    bool compare_timestamp(int id_0, int id_1){
        // 计算两个bin位姿强度值(时间戳)的绝对差
        if(abs(_bin_with_id[id_0].pose.intensity - _bin_with_id[id_1].pose.intensity) > _loop_frame_thres)
            return true;  // 时间差超过阈值，返回true
        else
            return false; // 时间差未超过阈值，返回false
    }


        /**
     * @brief 发送当前机器人的闭环信息
     * @param robot_id_this 当前机器人ID
     * @param robot_id_that 目标机器人ID
     * @param id_loop_this 当前闭环索引
     * @param id_loop_that 目标闭环索引
     * 
     * 1. 获取当前和目标机器人的闭环队列和位姿信息
     * 2. 计算相对位姿变换(考虑是否同一机器人)
     * 3. 计算位姿匹配质量分数
     * 4. 更新闭环信息并发布
     */
    void sendLoopThis(int robot_id_this, int robot_id_that, int id_loop_this, int id_loop_that){
        // 获取当前和目标机器人的闭环队列
        auto loop_list_this = _loop_queue[robot_id_this];
        auto loop_list_that = _loop_queue[robot_id_that];
        
        // 获取特定索引的闭环信息
        auto loop_this = loop_list_this[id_loop_this];
        auto loop_that = loop_list_that[id_loop_that];

        // 提取闭环对应的bin ID
        int id_bin_this = std::get<1>(loop_this);
        int id_bin_last = std::get<1>(loop_that);

        // 获取对应的位姿信息
        auto pose_this = _pose_queue[robot_id_this][id_loop_this];
        auto pose_that = _pose_queue[robot_id_that][id_loop_that];

        // 计算相对位姿变换
        gtsam::Pose3 pose_to_this, pose_to_that;
        if(robot_id_this == robot_id_that){
            // 同一机器人直接获取位姿
            pose_to_this = std::get<1>(pose_this);
            pose_to_that = std::get<1>(pose_that);
        }
        else{
            // 不同机器人需要转换坐标系
            if (_global_map_trans_optimized.find(robot_id_this) == _global_map_trans_optimized.end() ||
                    _global_map_trans_optimized.find(robot_id_that) == _global_map_trans_optimized.end() )
                return;
                
            // 获取并转换位姿到全局坐标系
            PointTypePose trans_this = _global_map_trans_optimized[robot_id_this];
            gtsam::Pose3 trans_this3 = gtsam::Pose3( 
                gtsam::Rot3::RzRyRx(trans_this.roll, trans_this.pitch, trans_this.yaw),
                gtsam::Point3(trans_this.x, trans_this.y, trans_this.z) );

            PointTypePose trans_that = _global_map_trans_optimized[robot_id_that];
            gtsam::Pose3 trans_that3 = gtsam::Pose3( 
                gtsam::Rot3::RzRyRx(trans_that.roll, trans_that.pitch, trans_that.yaw),
                gtsam::Point3(trans_that.x, trans_that.y, trans_that.z) );

            // 计算相对位姿
            pose_to_this = trans_this3.inverse() * std::get<1>(pose_this);
            pose_to_that = trans_that3.inverse() * std::get<1>(pose_that);
        }

        // 计算位姿匹配质量分数(取平均值)
        float tmp_intensity = (std::get<2>(pose_this) + std::get<2>(pose_that)) / 2.0;

        // 计算位姿变换关系
        auto dpose = pose_to_that.between(pose_to_this);
        auto thisp = _bin_with_id[id_bin_this].pose;
        auto thatp = _bin_with_id[id_bin_last].pose;
        
        // 转换为GTSAM位姿格式
        auto pose_to_this1 = gtsam::Pose3( 
            gtsam::Rot3::RzRyRx(thisp.roll, thisp.pitch, thisp.yaw),
            gtsam::Point3(thisp.x, thisp.y, thisp.z) );
        auto pose_to_that1 = gtsam::Pose3( 
            gtsam::Rot3::RzRyRx(thatp.roll, thatp.pitch, thatp.yaw),
            gtsam::Point3(thatp.x, thatp.y, thatp.z) );
        auto dpose1 = pose_to_that1.between(pose_to_this1);

        // 更新并发布闭环信息
        update_loop_info(id_bin_last, id_bin_this, pose_to_that, pose_to_this, tmp_intensity);
        _pub_loop_info.publish(_loop_info);
    }


        /**
     * @brief 发送历史闭环信息到全局话题
     * @param robot_id_this 当前机器人ID
     * @param robot_id_that 目标机器人ID
     * @param id_loop_this 当前闭环索引
     * @param id_loop_that 历史闭环索引
     * 
     * 1. 检查是否为同一机器人(仅处理同一机器人的历史闭环)
     * 2. 获取闭环队列和特定闭环信息
     * 3. 提取闭环对应的bin ID
     * 4. 计算相对位姿变换
     * 5. 计算位姿匹配质量分数
     * 6. 更新并发布全局闭环信息
     */
    void sendLoopThat(int robot_id_this,int robot_id_that, int id_loop_this, int id_loop_that){
        // 仅处理同一机器人的历史闭环
        if(robot_id_this != robot_id_that)
            return;
            
        // 获取当前机器人的闭环队列和特定闭环信息
        auto loop_list = _loop_queue[robot_id_this];
        auto loop_this = loop_list[id_loop_this];
        auto loop_that = loop_list[id_loop_that];

        // 提取闭环对应的bin ID
        int id_bin_this = std::get<0>(loop_this);  // 当前bin ID
        int id_bin_last = std::get<0>(loop_that);  // 历史bin ID

        // 获取对应的位姿信息
        auto pose_this = _pose_queue[robot_id_this][id_loop_this];
        auto pose_that = _pose_queue[robot_id_this][id_loop_that];
        
        // 计算相对位姿变换
        auto pose_to_this = std::get<0>(pose_this) * std::get<2>(loop_this);
        auto pose_to_that = std::get<0>(pose_that) * std::get<2>(loop_that);
        
        // 计算位姿匹配质量分数(取平均值)
        float tmp_intensity = (std::get<2>(pose_this) + std::get<2>(pose_that)) / 2.0;
        
        // 更新并发布全局闭环信息
        update_loop_info(id_bin_last, id_bin_this, pose_to_that, pose_to_this, tmp_intensity);
        _pub_loop_info_global.publish(_loop_info);
    }


        /**
     * @brief 更新闭环信息结构体
     * @param id_bin_last 历史bin的ID
     * @param id_bin_this 当前bin的ID
     * @param pose_to_last 到历史位姿的变换
     * @param pose_to_this 到当前位姿的变换
     * @param intensity 位姿匹配质量分数
     * 
     * 1. 计算两个位姿间的相对变换
     * 2. 设置机器人ID和bin信息
     * 3. 填充相对位姿的平移和旋转分量
     * 4. 设置位姿匹配质量分数
     */
    void update_loop_info(int id_bin_last, int id_bin_this, gtsam::Pose3 pose_to_last, gtsam::Pose3 pose_to_this, float intensity){
        // 计算两个位姿间的相对变换(历史位姿到当前位姿)
        auto dpose = pose_to_last.between(pose_to_this);
        
        // 设置机器人ID和bin信息
        _loop_info.robotID = _bin_with_id[id_bin_last].robotname;  // 来源机器人ID
        _loop_info.numRing = _bin_with_id[id_bin_last].pose.intensity;  // 来源bin的环号
        _loop_info.numSector = _bin_with_id[id_bin_this].pose.intensity;  // 目标bin的扇区号

        // 填充相对位姿的平移分量(x,y,z坐标)
        _loop_info.poseX = dpose.translation().x();
        _loop_info.poseY = dpose.translation().y();
        _loop_info.poseZ = dpose.translation().z();
        
        // 填充相对位姿的旋转分量(欧拉角)
        _loop_info.poseRoll = dpose.rotation().roll();   // 滚转角
        _loop_info.posePitch = dpose.rotation().pitch(); // 俯仰角
        _loop_info.poseYaw = dpose.rotation().yaw();     // 偏航角
        
        // 设置位姿匹配质量分数
        _loop_info.poseIntensity = intensity;
    }


        /**
     * @brief 发送里程计输出消息
     * 
     * 1. 发送地图变换消息
     * 2. 发布相对变换到其他机器人
     * 3. 发送全局闭环消息
     * 
     * 该函数组合了地图变换发布和闭环检测功能，是SLAM系统的主要输出接口
     */
    void sendOdomOutputMessage(){
        // 首先发送地图变换消息
        sendMapOutputMessage();

        // 初始化Odometry消息用于发布相对变换
        nav_msgs::Odometry odom2odom;
        
        // 设置消息头和时间戳(使用点云头的时间戳)
        odom2odom.header.stamp = _cloud_header.stamp;
        // 设置父坐标系ID(机器人ID)
        odom2odom.header.frame_id = _robot_id;
        // 设置子坐标系ID(当前机器人ID)
        odom2odom.child_frame_id = _robot_this;
        
        // 填充位置变换数据(x,y,z坐标)
        odom2odom.pose.pose.position.x = _global_map_trans_optimized[_robot_this_th].x;
        odom2odom.pose.pose.position.y = _global_map_trans_optimized[_robot_this_th].y;
        odom2odom.pose.pose.position.z = _global_map_trans_optimized[_robot_this_th].z;
        
        // 将欧拉角转换为四元数并设置姿态变换数据
        odom2odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
            _global_map_trans_optimized[_robot_this_th].roll,
            _global_map_trans_optimized[_robot_this_th].pitch,
            _global_map_trans_optimized[_robot_this_th].yaw);
        
        // 发布相对变换消息
        _pub_trans_odom2odom.publish(odom2odom);

        // 发送全局闭环检测消息
        sendGlobalLoopMessageKDTree();
    }

};

/**
 * @brief 主函数，ROS节点的入口点
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 程序退出状态码
 */
int main(int argc, char** argv)
{
    // 初始化ROS节点，节点名为"fushion"
    ros::init(argc, argv, "fushion");
    
    // 创建MapFusion类的实例
    MapFusion MapF;

    // 打印启动信息（绿色文字）
    ROS_INFO("\033[1;32m----> Map Fushion Started.\033[0m");

    // 创建并启动发布线程，运行MapFusion::publishContextInfoThread方法
    std::thread publishThread(&MapFusion::publishContextInfoThread, &MapF);

    // ROS主循环，处理回调函数
    ros::spin();

    // 等待发布线程结束
    publishThread.join();

    // 程序正常退出

    return 0;
}
