//
// Created by yewei on 2/27/20.

// This is an unofficial c++ implementation of Scan Context:
// @ARTICLE{ gkim-2019-ral,
//     author = {G. {Kim} and B. {Park} and A. {Kim}},
//     journal = {IEEE Robotics and Automation Letters},
//     title = {1-Day Learning, 1-Year Localization: Long-Term LiDAR Localization Using Scan Context Image},
//     year = {2019},
//     volume = {4},
//     number = {2},
//     pages = {1948-1955},
//     month = {April}
// }
// For more information please visit: https://github.com/irapkaist/scancontext

// 头文件保护宏，防止头文件被重复包含
#ifndef SRC_SCANCONTEXT_H
#define SRC_SCANCONTEXT_H
// 注释掉的头文件包含语句
//#include "utility.h"

// 包含PCL库相关头文件，用于点云处理
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

// 包含标准库头文件
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>

// 防止头文件被重复包含的预处理指令
#pragma once
// 包含Eigen库的核心头文件
#include <Eigen/Core>

// 定义点云的点类型为 PointXYZI
typedef pcl::PointXYZI  PointType;

/**
 * @brief 自定义点类型，包含位置、强度、姿态和时间信息
 * 
 * 继承自 PCL 的基础点类型，添加了姿态（滚转、俯仰、偏航）和时间戳信息
 */
struct PointXYZIRPYT
{
    // 添加 PCL 基础的 4D 点信息（x, y, z, 1）
    PCL_ADD_POINT4D
    // 添加点的强度信息
    PCL_ADD_INTENSITY;
    // 滚转角度
    float roll;
    // 俯仰角度
    float pitch;
    // 偏航角度
    float yaw;
    // 时间戳
    double time;
    // 确保内存对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// 定义自定义点类型的别名
typedef PointXYZIRPYT  PointTypePose;

/**
 * @brief 扫描上下文数据结构，包含机器人信息、时间、姿态、点云、扫描上下文矩阵和环形键向量
 */
struct ScanContextBin
{
    // 机器人名称
    std::string robotname;
    // 时间戳
    double time;
    // 机器人的姿态信息
    PointTypePose pose;
    // 点云指针
    pcl::PointCloud<PointType>::Ptr cloud;
    // 扫描上下文矩阵
    Eigen::MatrixXf bin;
    // 环形键向量
    Eigen::VectorXf ringkey;
};

/**
 * @brief 扫描上下文类，用于将点云转换为扫描上下文表示
 */
class ScanContext {
public:
    /**
     * @brief 构造函数，初始化扫描上下文的参数
     * 
     * @param max_range 扫描的最大范围
     * @param num_rings 扫描上下文的环数
     * @param num_sectors 扫描上下文的扇区数
     */
    ScanContext(int max_range, int num_rings, int num_sectors);

    /**
     * @brief 将点云转换为 ScanContextBin 对象
     * 
     * @param pt_cloud 输入的点云指针
     * @return ScanContextBin 转换后的 ScanContextBin 对象
     */
    ScanContextBin ptcloud2bin(pcl::PointCloud<PointType>::Ptr pt_cloud);

private:
    // 扫描的最大范围
    int _max_range;
    // 扫描上下文的环数
    int _num_rings;
    // 扫描上下文的扇区数
    int _num_sectors;
    // 每个环的间距
    float _gap;
    // 每个扇区对应的角度
    float _angle_one_sector;

    // 注释掉的体素滤波对象
    //pcl::VoxelGrid<PointType> downSizeFilterInput;

    /**
     * @brief 根据点的 x 和 y 坐标计算其相对于原点的角度
     * 
     * @param x 点的 x 坐标
     * @param y 点的 y 坐标
     * @return float 点相对于原点的角度，范围 0 ~ 360 度
     */
    float xy2Theta(float x, float y);

    /**
     * @brief 从扫描上下文矩阵计算环形键向量
     * 
     * @param _max_bin 输入的扫描上下文矩阵
     * @return Eigen::VectorXf 计算得到的环形键向量
     */
    Eigen::VectorXf scanContext2RingKey(Eigen::MatrixXf _max_bin);

    /**
     * @brief 将点云转换为扫描上下文矩阵
     * 
     * @param pt_cloud 输入的点云指针
     * @return Eigen::MatrixXf 转换后的扫描上下文矩阵
     */
    Eigen::MatrixXf ptCloud2ScanContext(pcl::PointCloud<PointType>::Ptr pt_cloud);
};

//circShift from https://github.com/irapkaist/SC-LeGO-LOAM
/**
 * @brief 对矩阵进行循环右移操作
 * 
 * @param _mat 输入的矩阵
 * @param _num_shift 右移的列数，必须大于等于 0
 * @return Eigen::MatrixXf 循环右移后的矩阵
 */
Eigen::MatrixXf circShift( Eigen::MatrixXf &_mat, int _num_shift )
{
    // 确保右移的列数大于等于 0，即只支持向右循环移动
    assert(_num_shift >= 0);

    // 如果右移列数为 0，直接返回原矩阵
    if( _num_shift == 0 )
    {
        Eigen::MatrixXf shifted_mat( _mat );
        return shifted_mat; // 提前返回
    }

    // 初始化一个全零矩阵，用于存储循环右移后的结果
    Eigen::MatrixXf shifted_mat = Eigen::MatrixXf::Zero( _mat.rows(), _mat.cols() );
    // 遍历原矩阵的每一列
    for ( int col_idx = 0; col_idx < _mat.cols(); col_idx++ )
    {
        // 计算该列移动后的新位置
        int new_location = (col_idx + _num_shift) % _mat.cols();
        // 将原矩阵的列赋值到新位置
        shifted_mat.col(new_location) = _mat.col(col_idx);
    }

    return shifted_mat;
}

// 结束头文件保护
#endif //SRC_SCANCONTEXT_H
