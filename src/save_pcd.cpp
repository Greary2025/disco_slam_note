/**
 * @file save_pcd.cpp
 * @brief ROS节点，用于订阅点云话题并保存为PCD文件
 * @author Generated for disco_slam_note workspace
 * @date 2025-07-22
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
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

class PCDSaver
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    
    std::string save_directory_;
    std::string topic_name_;
    std::string timestamp_file_;
    
    int file_counter_;
    std::ofstream timestamp_stream_;
    
    // 创建目录的辅助函数
    bool createDirectory(const std::string& path)
    {
        struct stat info;
        if (stat(path.c_str(), &info) != 0)
        {
            // 目录不存在，尝试创建
            if (mkdir(path.c_str(), 0755) == 0)
            {
                ROS_INFO("Created directory: %s", path.c_str());
                return true;
            }
            else
            {
                ROS_ERROR("Failed to create directory: %s", path.c_str());
                return false;
            }
        }
        else if (info.st_mode & S_IFDIR)
        {
            // 目录已存在
            return true;
        }
        else
        {
            ROS_ERROR("Path exists but is not a directory: %s", path.c_str());
            return false;
        }
    }
    
    // 生成文件名的辅助函数
    std::string generateFileName(int counter)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(4) << counter << ".pcd";
        return ss.str();
    }

public:
    PCDSaver() : nh_("~"), file_counter_(0)
    {
        // 获取参数
        nh_.param<std::string>("save_directory", save_directory_, "/tmp/saved_pcds");
        nh_.param<std::string>("topic_name", topic_name_, "/points_raw");
        
        // 确保保存目录存在
        if (!createDirectory(save_directory_))
        {
            ROS_ERROR("Cannot create save directory. Exiting.");
            ros::shutdown();
            return;
        }
        
        // 设置时间戳文件路径
        timestamp_file_ = save_directory_ + "/timestamps.txt";
        
        // 打开时间戳文件
        timestamp_stream_.open(timestamp_file_.c_str(), std::ios::out);
        if (!timestamp_stream_.is_open())
        {
            ROS_ERROR("Cannot open timestamp file: %s", timestamp_file_.c_str());
            ros::shutdown();
            return;
        }
        
        // 写入时间戳文件头部
        timestamp_stream_ << "# PCD文件时间戳记录" << std::endl;
        timestamp_stream_ << "# 格式: 文件名 时间戳(秒) 时间戳(纳秒) ROS时间字符串" << std::endl;
        timestamp_stream_ << "# File_Name Timestamp_Sec Timestamp_Nsec ROS_Time_String" << std::endl;
        
        // 订阅点云话题
        cloud_sub_ = nh_.subscribe(topic_name_, 10, &PCDSaver::cloudCallback, this);
        
        ROS_INFO("PCDSaver initialized:");
        ROS_INFO("  - Topic: %s", topic_name_.c_str());
        ROS_INFO("  - Save directory: %s", save_directory_.c_str());
        ROS_INFO("  - Timestamp file: %s", timestamp_file_.c_str());
        ROS_INFO("Waiting for point cloud messages...");
    }
    
    ~PCDSaver()
    {
        if (timestamp_stream_.is_open())
        {
            timestamp_stream_.close();
        }
        ROS_INFO("Saved %d PCD files total.", file_counter_);
    }
    
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        try
        {
            // 转换ROS消息到PCL点云
            pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*cloud_msg, *pcl_cloud);
            
            // 生成文件名
            std::string filename = generateFileName(file_counter_);
            std::string full_path = save_directory_ + "/" + filename;
            
            // 保存PCD文件
            if (pcl::io::savePCDFileBinary(full_path, *pcl_cloud) == -1)
            {
                ROS_ERROR("Failed to save PCD file: %s", full_path.c_str());
                return;
            }
            
            // 记录时间戳信息
            ros::Time stamp = cloud_msg->header.stamp;
            timestamp_stream_ << filename << " " 
                            << stamp.sec << " " 
                            << stamp.nsec << " " 
                            << stamp << std::endl;
            timestamp_stream_.flush(); // 确保立即写入文件
            
            ROS_INFO("Saved: %s (points: %lu, time: %f)", 
                    filename.c_str(), 
                    pcl_cloud->points.size(),
                    stamp.toSec());
            
            file_counter_++;
            
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "save_pcd_node");
    
    try
    {
        PCDSaver saver;
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Exception in main: %s", e.what());
        return -1;
    }
    
    return 0;
}
