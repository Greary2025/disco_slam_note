/**
 * @file save_pcd.cpp
 * @brief ROS节点，用于订阅点云话题并保存为PCD文件
 * @details 本节点实现以下功能：
 * 1. 创建指定存储目录
 * 2. 初始化ROS订阅器监听点云话题
 * 3. 接收点云数据并转换为PCL格式
 * 4. 按顺序保存PCD文件并记录时间戳
 * 5. 异常处理和安全关闭机制
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <mutex>

class PCDSaver
{
private:
    // ROS节点句柄（私有节点，支持参数服务器访问）
    ros::NodeHandle nh_; 
    // 点云订阅器（自动处理消息队列）
    ros::Subscriber cloud_sub_;  
    // 路径订阅器（用于获取位姿信息）
    ros::Subscriber path_sub_;
    // 超时检测定时器
    ros::Timer timeout_timer_;
    
    // 配置参数
    std::string save_directory_;  // PCD文件存储路径
    std::string topic_name_;      // 订阅的点云话题名称
    std::string path_topic_;      // 订阅的路径话题名称
    std::string pose_file_;       // pose.json文件路径
    
    // 文件计数器（保证文件名唯一性）
    int file_counter_;            
    // pose文件输出流
    std::ofstream pose_stream_;
    // 最后接收到消息的时间
    ros::Time last_msg_time_;
    // 超时时间（秒）
    double timeout_duration_;
    // 是否已接收到第一条消息
    bool received_first_msg_;
    
    // 数据更新检测相关变量
    size_t last_cloud_size_;             // 上一次点云的大小
    ros::Time last_data_change_time_;    // 最后一次数据变化的时间
    int same_data_count_;                // 连续相同数据的计数
    // 当前位姿信息（tx ty tz qw qx qy qz）
    std::vector<double> current_pose_;
    // 位姿信息互斥锁
    std::mutex pose_mutex_;  
    
    /**
     * @brief 检查存储目录是否存在
     * @param path 目录路径
     * @return true 目录存在且可访问
     * @return false 目录不存在或不可访问
     * @note 使用POSIX标准库函数进行目录检查，
     *       不会自动创建目录，需要用户手动创建
     */
    bool checkDirectory(const std::string& path)
    {
        struct stat info;
        if (stat(path.c_str(), &info) != 0)
        {
            // 目录不存在
            ROS_ERROR("Directory does not exist: %s", path.c_str());
            ROS_ERROR("Please create the directory manually before running this node.");
            return false;
        }
        else if (info.st_mode & S_IFDIR)
        {
            // 目录已存在
            ROS_INFO("Using existing directory: %s", path.c_str());
            return true;
        }
        else
        {
            ROS_ERROR("Path exists but is not a directory: %s", path.c_str());
            return false;
        }
    }
    
    /**
     * @brief 检查PCD子目录是否存在
     * @param base_directory 基础目录路径
     * @return bool PCD目录存在返回true，不存在返回false
     */
    bool checkPCDDirectory(const std::string& base_directory)
    {
        std::string pcd_dir = base_directory + "/pcd";
        struct stat info;
        if (stat(pcd_dir.c_str(), &info) != 0)
        {
            ROS_ERROR("PCD directory does not exist: %s", pcd_dir.c_str());
            ROS_ERROR("Please create the pcd folder manually in the save directory.");
            return false;
        }
        else if (!(info.st_mode & S_IFDIR))
        {
            ROS_ERROR("PCD path exists but is not a directory: %s", pcd_dir.c_str());
            return false;
        }
        return true;
    }
    
    /**
     * @brief 路径回调函数
     * @param msg 接收到的路径消息
     * @details 从路径消息中提取最新的位姿信息并存储
     */
    void pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        if (msg->poses.empty())
            return;
            
        // 获取最新的位姿信息
        const auto& latest_pose = msg->poses.back().pose;
        
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_pose_.clear();
        current_pose_.resize(7);
        
        // 存储位姿信息：tx ty tz qw qx qy qz
        current_pose_[0] = latest_pose.position.x;
        current_pose_[1] = latest_pose.position.y;
        current_pose_[2] = latest_pose.position.z;
        current_pose_[3] = latest_pose.orientation.w;
        current_pose_[4] = latest_pose.orientation.x;
        current_pose_[5] = latest_pose.orientation.y;
        current_pose_[6] = latest_pose.orientation.z;
    }
    
    /**
     * @brief 超时检测回调函数
     * @param event 定时器事件
     * @details 检查是否超过指定时间没有接收到新的点云数据，
     *          如果数据没有更新则自动停止程序
     */
    void timeoutCallback(const ros::TimerEvent& event)
    {
        if (!received_first_msg_)
        {
            // 还没有接收到第一条消息，继续等待
            return;
        }
        
        ros::Time current_time = ros::Time::now();
        double time_since_last_change = (current_time - last_data_change_time_).toSec();
        
        // 如果连续接收到相同数据超过5次，且超过超时时间，则认为数据流结束
        if (same_data_count_ >= 5 && time_since_last_change > timeout_duration_)
        {
            ROS_INFO("Point cloud data has not changed for %.1f seconds (received %d identical clouds).", 
                     time_since_last_change, same_data_count_);
            ROS_INFO("Data stream appears to have ended. Stopping PCD saving.");
            ROS_INFO("Total saved: %d PCD files.", file_counter_);
            ros::shutdown();
        }
        
        // 如果完全没有接收到消息超过超时时间的2倍，也停止程序
        double time_since_last_msg = (current_time - last_msg_time_).toSec();
        if (time_since_last_msg > timeout_duration_ * 2)
        {
            ROS_INFO("No point cloud messages received for %.1f seconds.", time_since_last_msg);
            ROS_INFO("Data stream appears to have ended. Stopping PCD saving.");
            ROS_INFO("Total saved: %d PCD files.", file_counter_);
            ros::shutdown();
        }
    }
    
    /**
     * @brief 生成四位补零的文件名
     * @param counter 当前文件序号
     * @return std::string 格式化文件名
     * @example 输入1 → "0001.pcd"
     */
    std::string generateFileName(int counter)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(4) << counter << ".pcd";
        return ss.str();
    }

public:
    /**
     * @brief 构造函数初始化节点
     * @details 执行以下初始化步骤：
     * 1. 从参数服务器获取配置参数
     * 2. 检查存储目录是否存在
     * 3. 初始化点云订阅器和时间戳文件
     * 4. 启动超时检测定时器
     * 5. 打印初始化状态信息
     */
    PCDSaver() : nh_("~"), file_counter_(0), received_first_msg_(false), 
                 last_cloud_size_(0), same_data_count_(0)
    {
        // 从参数服务器获取配置参数
        nh_.param<std::string>("save_directory", save_directory_, "/tmp/saved_pcds");
        nh_.param<std::string>("topic_name", topic_name_, "/points_raw");
        nh_.param<std::string>("path_topic", path_topic_, "/disco_double/mapping/path");
        nh_.param<double>("timeout_duration", timeout_duration_, 10.0); // 默认10秒超时
        
        // 初始化数据更新检测变量
        last_data_change_time_ = ros::Time::now();
        
        // 初始化位姿信息为默认值
        current_pose_.resize(7);
        current_pose_[0] = 0; // tx
        current_pose_[1] = 0; // ty
        current_pose_[2] = 0; // tz
        current_pose_[3] = 1; // qw
        current_pose_[4] = 0; // qx
        current_pose_[5] = 0; // qy
        current_pose_[6] = 0; // qz
        
        // 检查保存目录是否存在
        if (!checkDirectory(save_directory_))
        {
            ROS_ERROR("Save directory does not exist. Exiting.");
            ros::shutdown();
            return;
        }
        
        // 检查PCD子目录是否存在
        if (!checkPCDDirectory(save_directory_))
        {
            ROS_ERROR("PCD subdirectory does not exist. Exiting.");
            ros::shutdown();
            return;
        }
        
        // 设置pose文件路径
        pose_file_ = save_directory_ + "/pose.json";
        
        // 打开pose文件
        pose_stream_.open(pose_file_.c_str(), std::ios::out);
        if (!pose_stream_.is_open())
        {
            ROS_ERROR("Cannot open pose file: %s", pose_file_.c_str());
            ros::shutdown();
            return;
        }
        
        // pose文件不需要头部信息，直接开始写入pose数据
        
        // 订阅点云话题
        cloud_sub_ = nh_.subscribe(topic_name_, 10, &PCDSaver::cloudCallback, this);
        
        // 订阅路径话题
        path_sub_ = nh_.subscribe(path_topic_, 10, &PCDSaver::pathCallback, this);
        
        // 启动超时检测定时器（每秒检查一次）
        timeout_timer_ = nh_.createTimer(ros::Duration(1.0), &PCDSaver::timeoutCallback, this);
        
        ROS_INFO("PCDSaver initialized:");
        ROS_INFO("  - Topic: %s", topic_name_.c_str());
        ROS_INFO("  - Path topic: %s", path_topic_.c_str());
        ROS_INFO("  - Save directory: %s", save_directory_.c_str());
        ROS_INFO("  - Pose file: %s", pose_file_.c_str());
        ROS_INFO("  - Timeout duration: %.1f seconds", timeout_duration_);
        ROS_INFO("Waiting for point cloud messages...");
    }
    
    /**
     * @brief 析构函数保证资源释放
     * @note 自动关闭pose文件流
     */
    ~PCDSaver()
    {
        if (pose_stream_.is_open())
         {
             pose_stream_.close();
         }
        ROS_INFO("Saved %d PCD files total.", file_counter_);
    }
    
    /**
     * @brief 点云回调处理函数
     * @param cloud_msg ROS点云消息指针
     * @details 执行流程：
     * 1. 转换ROS消息为PCL格式
     * 2. 生成唯一文件名
     * 3. 保存PCD文件（二进制格式）
     * 4. 记录时间戳信息
     * 5. 安全限制最大文件数量
     */
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        try
        {
            // 更新最后接收消息的时间
            last_msg_time_ = ros::Time::now();
            if (!received_first_msg_)
            {
                received_first_msg_ = true;
                ROS_INFO("Received first point cloud message. Starting timeout monitoring.");
            }
            
            // 转换ROS消息到PCL点云
            pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*cloud_msg, *pcl_cloud);
            
            // 检测数据是否发生变化
            size_t current_cloud_size = pcl_cloud->points.size();
            if (current_cloud_size != last_cloud_size_)
            {
                // 数据发生变化，重置计数器并更新时间
                same_data_count_ = 0;
                last_data_change_time_ = ros::Time::now();
                last_cloud_size_ = current_cloud_size;
            }
            else
            {
                // 数据没有变化，增加计数器
                same_data_count_++;
                if (same_data_count_ == 1)
                {
                    ROS_WARN("Detected identical point cloud data (size: %lu). Count: %d", 
                             current_cloud_size, same_data_count_);
                }
                else if (same_data_count_ % 5 == 0)
                {
                    ROS_WARN("Still receiving identical point cloud data. Count: %d", same_data_count_);
                }
            }
            
            // 只有当数据发生变化时才保存文件
            if (same_data_count_ == 0)
            {
                // 生成文件名
                std::string filename = generateFileName(file_counter_);
                std::string full_path = save_directory_ + "/pcd/" + filename;
                
                // 保存PCD文件
                if (pcl::io::savePCDFileBinary(full_path, *pcl_cloud) == -1)
                {
                    ROS_ERROR("Failed to save PCD file: %s", full_path.c_str());
                    return;
                }
                
                // 记录pose信息（格式：tx ty tz qw qx qy qz）
                {
                    std::lock_guard<std::mutex> lock(pose_mutex_);
                    pose_stream_ << current_pose_[0] << " " << current_pose_[1] << " " << current_pose_[2] << " "
                                << current_pose_[3] << " " << current_pose_[4] << " " << current_pose_[5] << " " << current_pose_[6] << std::endl;
                    pose_stream_.flush(); // 确保立即写入文件
                }
                
                ROS_INFO("Saved: %s (points: %lu)", 
                        filename.c_str(), 
                        pcl_cloud->points.size());
                
                file_counter_++;
            }
            else
            {
                // 跳过保存重复数据
                ROS_DEBUG("Skipping duplicate point cloud data (count: %d)", same_data_count_);
            }
            
            // 检查是否达到最大文件数量限制
            if (file_counter_ >= 10000)
            {
                ROS_WARN("Reached maximum file count (10000). Stopping to prevent overflow.");
                ros::shutdown();
            }
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("Exception in cloudCallback: %s", e.what());
        }
    }
};

/**
 * @brief ROS节点主入口函数
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出码（0表示正常退出）
 * @details 执行流程：
 * 1. 初始化ROS节点和运行时系统
 * 2. 创建PCD存储对象
 * 3. 进入消息循环等待点云数据
 * 4. 异常安全机制保证资源释放
 */
int main(int argc, char** argv)
{
    // 初始化ROS节点（节点名称保持与launch文件一致）
    ros::init(argc, argv, "save_pcd_node");
    
    ROS_INFO("Starting PCD Saver Node...");
    ROS_INFO("Usage: rosrun disco_double save_pcd _save_directory:=/path/to/save _topic_name:=/your_topic _path_topic:=/path_topic");
    ROS_INFO("Parameters:");
    ROS_INFO("  _save_directory: Directory to save PCD files (default: /tmp/saved_pcds)");
    ROS_INFO("  _topic_name: Point cloud topic to subscribe (default: /points_raw)");
    ROS_INFO("  _path_topic: Path topic to get pose data (default: /disco_double/mapping/path)");
    ROS_INFO("  _timeout_duration: Timeout in seconds to stop saving (default: 10.0)");
    
    // 异常处理保证程序健壮性
    try
    {
        // 构造点云存储对象（自动触发参数加载和订阅器初始化）
        PCDSaver saver;
        
        // 进入消息循环（自动唤醒处理回调函数）
        ros::spin();
    }
    catch (const std::exception& e)
    {
        // 捕获并记录致命异常（如内存分配失败、文件系统错误等）
        ROS_ERROR("主函数异常: %s", e.what());
        return -1;
    }
    
    ROS_INFO("PCD Saver Node terminated.");
    // 正常退出（自动调用析构函数释放资源）
    return 0;
}
