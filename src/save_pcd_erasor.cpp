//
// 节点功能概述（ERASOR保存）：
// 1) 订阅激光点云（/xianfeng/lidar），逐帧保存为PCD文件到 <output_dir>/pcds/
// 2) 订阅轨迹路径（/xianfeng/disco_double/mapping/path），将最新位姿以CSV格式追加到
//    <output_dir>/poses_lidar2body_origin.csv（表头：frame_id,timestamp,x,y,z,qx,qy,qz,qw）
// 3) 订阅全局地图（/xianfeng/disco_double/mapping/map_global），在超时或退出时保存最后一帧
//    地图为 <output_dir>/map_global_final.pcd
// 4) 若超过 timeout_sec 未收到任一话题消息，则自动停止：保存最终地图，关闭订阅并退出
//
// 说明：尽量避免在回调中做耗时操作；PCD保存采用二进制格式以提升写入效率。

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

namespace
{
// 工具函数：检测目录是否存在
bool directoryExists(const std::string &path)
{
    struct stat info;
    if (stat(path.c_str(), &info) != 0) return false;
    return (info.st_mode & S_IFDIR) != 0;
}

// 工具函数：递归创建多级目录（容忍中间目录已存在）
bool createDirectoriesRecursively(const std::string &path)
{
    if (path.empty()) return false;
    if (directoryExists(path)) return true;

    // Create parents first
    size_t pos = 0;
    bool ok = true;
    while (ok)
    {
        pos = path.find('/', pos + 1);
        std::string sub = path.substr(0, pos);
        if (!sub.empty() && !directoryExists(sub))
        {
            if (mkdir(sub.c_str(), 0775) != 0 && errno != EEXIST)
            {
                ok = false;
                break;
            }
        }
        if (pos == std::string::npos) break;
    }
    return ok && directoryExists(path);
}

// 工具函数：清空目录中的所有文件
bool clearDirectory(const std::string &path)
{
    if (!directoryExists(path)) return true;
    
    std::string command = "rm -rf " + path + "/*";
    int result = system(command.c_str());
    return result == 0;
}
} // namespace

class SavePcdErasorNode
{
public:
    SavePcdErasorNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : nh_(nh), pnh_(pnh)
    {
        // 私有参数（可在launch文件中配置）
        // - lidar_topic: 点云话题
        // - path_topic: 轨迹话题（nav_msgs/Path）
        // - map_topic: 全局地图点云话题
        // - output_dir: 输出根目录（会创建 pcds 子目录与 CSV 文件）
        // - timeout_sec: 超时判定秒数，超过此时间无消息则触发安全退出
        pnh_.param<std::string>("lidar_topic", lidar_topic_, std::string("/xianfeng/lidar"));
        pnh_.param<std::string>("path_topic", path_topic_, std::string("/xianfeng/disco_double/mapping/path"));
        pnh_.param<std::string>("map_topic", map_topic_, std::string("/xianfeng/disco_double/mapping/map_global"));
        pnh_.param<std::string>("output_dir", output_dir_, std::string("/tmp/disco_outputs"));
        pnh_.param<double>("timeout_sec", timeout_sec_, 10.0);

        // Prepare directories and files
        // 创建根目录与 pcds 子目录；若失败则直接退出
        if (!createDirectoriesRecursively(output_dir_))
        {
            ROS_FATAL_STREAM("Failed to create output_dir: " << output_dir_);
            throw std::runtime_error("Cannot create output_dir");
        }
        pcds_dir_ = output_dir_ + "/pcds";
        if (!createDirectoriesRecursively(pcds_dir_))
        {
            ROS_FATAL_STREAM("Failed to create pcds dir: " << pcds_dir_);
            throw std::runtime_error("Cannot create pcds dir");
        }

        // 清空pcds目录和CSV文件（覆盖写模式）
        if (!clearDirectory(pcds_dir_))
        {
            ROS_WARN_STREAM("Failed to clear pcds directory: " << pcds_dir_);
        }
        else
        {
            ROS_INFO_STREAM("Cleared pcds directory: " << pcds_dir_);
        }

        poses_csv_path_ = output_dir_ + "/poses_lidar2body_origin.csv"; // 位姿CSV完整路径
        openCsv();

        // 重置计数器，确保每次启动都从0开始
        saved_pcd_count_ = 0;
        frame_seq_ = 0;

        // 订阅三个话题；使用较小队列以降低延迟
        lidar_sub_ = nh_.subscribe(lidar_topic_, 10, &SavePcdErasorNode::lidarCallback, this);
        path_sub_ = nh_.subscribe(path_topic_, 10, &SavePcdErasorNode::pathCallback, this);
        map_sub_ = nh_.subscribe(map_topic_, 2, &SavePcdErasorNode::mapCallback, this);

        // 初始化时不设置last_msg_time_，等待第一个消息到达后才开始超时计时
        data_started_ = false;
        timeout_timer_ = nh_.createTimer(ros::Duration(1.0), &SavePcdErasorNode::timeoutCheck, this);

        ROS_INFO_STREAM("save_pcd_erasor started. Output: " << output_dir_);
    }

    ~SavePcdErasorNode()
    {
        // 析构时尽力保存最终地图并关闭CSV
        flushFinalMap();
        if (csv_stream_.is_open()) csv_stream_.close();
    }

private:
    void openCsv()
    {
        // 以覆盖模式打开CSV文件，每次启动都重新开始
        csv_stream_.open(poses_csv_path_, std::ios::out | std::ios::trunc);
        if (!csv_stream_.is_open())
        {
            ROS_FATAL_STREAM("Failed to open CSV: " << poses_csv_path_);
            throw std::runtime_error("Cannot open CSV");
        }
        
        // 写入表头
        csv_stream_ << "index,timestamp,x,y,z,qx,qy,qz,qw\n";
        csv_stream_.flush();
        ROS_INFO_STREAM("Opened CSV file in overwrite mode: " << poses_csv_path_);
    }

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        // 点云帧：缓存最新点云数据，等待位姿数据到达后再保存
        if (!data_started_) {
            data_started_ = true;
            ROS_INFO("Data started - received first lidar message");
        }
        last_msg_time_ = ros::Time::now();
        
        // 缓存最新的点云数据
        latest_lidar_ = *msg;
        have_lidar_ = true;
    }

    void pathCallback(const nav_msgs::PathConstPtr &msg)
    {
        // 路径：仅取最新一条位姿并写入CSV；用时间戳去重
        if (!data_started_) {
            data_started_ = true;
            ROS_INFO("Data started - received first path message");
        }
        last_msg_time_ = ros::Time::now();
        if (msg->poses.empty()) return;

        const geometry_msgs::PoseStamped &ps = msg->poses.back();
        const double t = ps.header.stamp.toSec();

        // Only append if timestamp increased to avoid duplicates
        if (t <= last_path_stamp_) return;
        last_path_stamp_ = t;

        const auto &p = ps.pose.position;
        const auto &q = ps.pose.orientation;

        // 保存PCD文件（如果有缓存的点云数据）
        if (have_lidar_) {
            pcl::PointCloud<pcl::PointXYZI> cloud;
            pcl::fromROSMsg(latest_lidar_, cloud);

            std::ostringstream oss;
            oss << pcds_dir_ << "/" << std::setw(6) << std::setfill('0') << saved_pcd_count_ << ".pcd";
            const std::string pcd_path = oss.str();

            if (pcl::io::savePCDFileBinary(pcd_path, cloud) != 0)
            {
                ROS_WARN_STREAM("Failed to save PCD: " << pcd_path);
            }
            else
            {
                saved_pcd_count_++;
            }
        }

        // 写入CSV文件
        csv_stream_ << (saved_pcd_count_ > 0 ? (saved_pcd_count_ - 1) : 0) << ","
                    << std::fixed << std::setprecision(9) << t << ","
                    << std::setprecision(9) << p.x << "," << p.y << "," << p.z << ","
                    << q.x << "," << q.y << "," << q.z << "," << q.w << "\n";
        csv_stream_.flush(); // 确保及时落盘，防止异常退出丢数据
    }

    void mapCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        // 地图：仅缓存最后一帧，退出或超时再保存
        if (!data_started_) {
            data_started_ = true;
            ROS_INFO("Data started - received first map message");
        }
        last_msg_time_ = ros::Time::now();
        latest_map_ = *msg; // copy
        have_map_ = true;
    }

    void timeoutCheck(const ros::TimerEvent &)
    {
        // 定时器：检测全局超时（任一话题未到达均会触发）；触发后保存地图并优雅退出
        if (stopped_) return;
        
        // 如果数据还没有开始，不进行超时检查
        if (!data_started_) return;
        
        ros::Time now = ros::Time::now();
        if ((now - last_msg_time_).toSec() > timeout_sec_)
        {
            ROS_WARN_STREAM("No messages for " << timeout_sec_ << "s. Saving final map and shutting down.");
            flushFinalMap();
            stopped_ = true;
            // Stop writing further by shutting down subscribers
            lidar_sub_.shutdown();
            path_sub_.shutdown();
            map_sub_.shutdown();
            ros::shutdown();
        }
    }

    void flushFinalMap()
    {
        // 将缓存的最后地图保存为PCD（若存在）
        if (!have_map_) return;
        try
        {
            // 同样将全局地图转换为XYZI存盘
            pcl::PointCloud<pcl::PointXYZI> map_cloud;
            pcl::fromROSMsg(latest_map_, map_cloud);
            const std::string map_path = output_dir_ + "/map_global_final.pcd";
            if (pcl::io::savePCDFileBinary(map_path, map_cloud) == 0)
            {
                ROS_INFO_STREAM("Saved final map to: " << map_path);
            }
            else
            {
                ROS_WARN_STREAM("Failed to save final map: " << map_path);
            }
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM("Exception while saving final map: " << e.what());
        }
        have_map_ = false;
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::string lidar_topic_;
    std::string path_topic_;
    std::string map_topic_;
    std::string output_dir_;
    std::string pcds_dir_;
    std::string poses_csv_path_;

    ros::Subscriber lidar_sub_;
    ros::Subscriber path_sub_;
    ros::Subscriber map_sub_;
    ros::Timer timeout_timer_;

    std::ofstream csv_stream_;
    uint64_t frame_seq_ = 0;
    uint64_t saved_pcd_count_ = 0;  // 实际保存的PCD文件数量
    double last_path_stamp_ = -1.0;
    double timeout_sec_ = 10.0;
    ros::Time last_msg_time_;
    bool stopped_ = false;
    bool data_started_ = false;  // 标志是否已开始接收数据

    sensor_msgs::PointCloud2 latest_map_;
    bool have_map_ = false;
    
    sensor_msgs::PointCloud2 latest_lidar_;  // 缓存最新的点云数据
    bool have_lidar_ = false;  // 是否有缓存的点云数据
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_pcd_erasor");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    try
    {
        SavePcdErasorNode node(nh, pnh);
        ros::spin();
    }
    catch (const std::exception &e)
    {
        ROS_FATAL_STREAM("Node initialization failed: " << e.what());
        return 1;
    }
    return 0;
}


