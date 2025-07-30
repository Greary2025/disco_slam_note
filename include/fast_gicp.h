#ifndef FAST_GICP_H
#define FAST_GICP_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <limits>
#include <ros/ros.h>

template<typename PointSource, typename PointTarget>
class FastGICP {
public:
    FastGICP(const std::string& config_prefix = "disco_double/fast_gicp") {
        has_converged_ = false;
        fitness_score_ = std::numeric_limits<double>::max();
        final_transformation_ = Eigen::Matrix4f::Identity();
        
        // 从ROS参数服务器加载配置参数
        loadParameters(config_prefix);
        
        // 初始化体素滤波器
        voxel_filter_.setLeafSize(voxel_resolution_, voxel_resolution_, voxel_resolution_);
    }
    
    void loadParameters(const std::string& config_prefix) {
        ros::NodeHandle nh;
        
        // 加载FastGICP配置参数，如果参数不存在则使用默认值
        nh.param<int>(config_prefix + "/num_threads", num_threads_, 4);
        nh.param<int>(config_prefix + "/correspondence_randomness", correspondence_randomness_, 20);
        nh.param<double>(config_prefix + "/max_correspondence_distance", max_correspondence_distance_, 1.0);
        nh.param<double>(config_prefix + "/voxel_resolution", voxel_resolution_, 0.1);
        nh.param<int>(config_prefix + "/max_iterations", max_iterations_, 64);
        nh.param<double>(config_prefix + "/transformation_epsilon", transformation_epsilon_, 1e-6);
        nh.param<double>(config_prefix + "/euclidean_fitness_epsilon", euclidean_fitness_epsilon_, 1e-6);
        
        ROS_INFO("FastGICP parameters loaded: max_dist=%.2f, max_iter=%d, voxel_res=%.3f, threads=%d",
                 max_correspondence_distance_, max_iterations_, voxel_resolution_, num_threads_);
    }

    void setNumThreads(int threads) { num_threads_ = threads; }
    void setCorrespondenceRandomness(int randomness) { correspondence_randomness_ = randomness; }
    void setMaxCorrespondenceDistance(double distance) { max_correspondence_distance_ = distance; }
    void setVoxelResolution(double resolution) { 
        voxel_resolution_ = resolution;
        voxel_filter_.setLeafSize(resolution, resolution, resolution);
    }
    void setMaximumIterations(int iterations) { max_iterations_ = iterations; }
    void setTransformationEpsilon(double epsilon) { transformation_epsilon_ = epsilon; }
    void setEuclideanFitnessEpsilon(double epsilon) { euclidean_fitness_epsilon_ = epsilon; }
    void setRANSACIterations(int iterations) { /* FastGICP不使用RANSAC */ }

    void setInputSource(typename pcl::PointCloud<PointSource>::Ptr cloud) {
        input_source_ = cloud;
        // 对源点云进行体素降采样
        if (cloud && !cloud->empty()) {
            voxel_filter_.setInputCloud(cloud);
            source_downsampled_.reset(new pcl::PointCloud<PointSource>());
            voxel_filter_.filter(*source_downsampled_);
        }
    }

    void setInputTarget(typename pcl::PointCloud<PointTarget>::Ptr cloud) {
        input_target_ = cloud;
        // 对目标点云进行体素降采样并构建KD树
        if (cloud && !cloud->empty()) {
            voxel_filter_.setInputCloud(cloud);
            target_downsampled_.reset(new pcl::PointCloud<PointTarget>());
            voxel_filter_.filter(*target_downsampled_);
            
            // 构建KD树
            target_kdtree_.reset(new pcl::KdTreeFLANN<PointTarget>());
            target_kdtree_->setInputCloud(target_downsampled_);
        }
    }

    void align(pcl::PointCloud<PointSource>& output) {
        if (!input_source_ || !input_target_ || input_source_->empty() || input_target_->empty()) {
            has_converged_ = false;
            fitness_score_ = std::numeric_limits<double>::max();
            return;
        }

        // 使用改进的ICP算法进行配准
        pcl::IterativeClosestPoint<PointSource, PointTarget> icp;
        
        // 设置参数
        icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
        icp.setMaximumIterations(max_iterations_);
        icp.setTransformationEpsilon(transformation_epsilon_);
        icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);
        icp.setRANSACIterations(0);
        
        // 使用降采样后的点云进行配准
        icp.setInputSource(source_downsampled_);
        icp.setInputTarget(target_downsampled_);
        
        // 执行配准
        typename pcl::PointCloud<PointSource>::Ptr aligned(new pcl::PointCloud<PointSource>());
        icp.align(*aligned);
        
        // 获取结果
        has_converged_ = icp.hasConverged();
        fitness_score_ = icp.getFitnessScore();
        final_transformation_ = icp.getFinalTransformation();
        
        // 将变换应用到原始点云
        if (has_converged_) {
            pcl::transformPointCloud(*input_source_, output, final_transformation_);
        } else {
            output = *input_source_;
        }
    }

    bool hasConverged() const { return has_converged_; }
    double getFitnessScore() const { return fitness_score_; }
    Eigen::Matrix4f getFinalTransformation() const { return final_transformation_; }
private:
    int num_threads_;
    int correspondence_randomness_;
    double max_correspondence_distance_;
    double voxel_resolution_;
    int max_iterations_;
    double transformation_epsilon_;
    double euclidean_fitness_epsilon_;
    
    typename pcl::PointCloud<PointSource>::Ptr input_source_;
    typename pcl::PointCloud<PointTarget>::Ptr input_target_;
    typename pcl::PointCloud<PointSource>::Ptr source_downsampled_;
    typename pcl::PointCloud<PointTarget>::Ptr target_downsampled_;
    
    typename pcl::KdTreeFLANN<PointTarget>::Ptr target_kdtree_;
    pcl::VoxelGrid<PointSource> voxel_filter_;
    
    bool has_converged_;
    double fitness_score_;
    Eigen::Matrix4f final_transformation_;
};

// 类型别名
using FastGICPPointXYZ = FastGICP<pcl::PointXYZ, pcl::PointXYZ>;
using FastGICPPointXYZI = FastGICP<pcl::PointXYZI, pcl::PointXYZI>;

#endif // FAST_GICP_H