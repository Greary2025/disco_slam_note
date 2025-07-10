//
// Created by yewei on 2/27/20.
//

// 这是 Scan Context 的非官方 C++ 实现
// 引用论文信息如下：
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
// 更多信息请访问: https://github.com/irapkaist/scancontext

// 包含扫描上下文的头文件
#include "scanContext.h"

/**
 * @brief ScanContext 类的构造函数
 * 
 * 初始化扫描上下文的最大范围、环数和扇区数，并计算环间距和每个扇区的角度。
 * 
 * @param max_range 扫描的最大范围
 * @param num_rings 扫描上下文的环数
 * @param num_sectors 扫描上下文的扇区数
 */
ScanContext::ScanContext(int max_range, int num_rings, int num_sectors)
  : _max_range(max_range), _num_rings(num_rings), _num_sectors(num_sectors){
  // 计算每个环的间距
  _gap = float(_max_range) / float(_num_rings);
  // 计算每个扇区对应的角度
  _angle_one_sector = 360.0 / float(_num_sectors);
}

/**
 * @brief 将点云转换为 ScanContextBin 对象
 * 
 * 该函数接收一个点云指针，将其转换为 ScanContextBin 对象，
 * 其中包含点云数据、扫描上下文矩阵和环形键向量。
 * 
 * @param pt_cloud 输入的点云指针
 * @return ScanContextBin 转换后的 ScanContextBin 对象
 */
ScanContextBin ScanContext::ptcloud2bin(pcl::PointCloud<PointType>::Ptr pt_cloud){
  // 创建一个 ScanContextBin 对象
  ScanContextBin sc_bin;
  // 为 ScanContextBin 对象中的点云分配内存
  sc_bin.cloud.reset(new pcl::PointCloud<PointType>());
  // 交换输入点云和 ScanContextBin 对象中点云的所有权
  std::swap( sc_bin.cloud, pt_cloud);
  // 注释掉的代码，功能为直接赋值点云指针
  //sc_bin.cloud = pt_cloud;
  // 将点云转换为扫描上下文矩阵
  sc_bin.bin = ptCloud2ScanContext(sc_bin.cloud);
  // 从扫描上下文矩阵计算环形键向量
  sc_bin.ringkey = scanContext2RingKey(sc_bin.bin);
  // 返回转换后的 ScanContextBin 对象
  return sc_bin;
}

/**
 * @brief 将点云转换为扫描上下文矩阵
 * 
 * 该函数遍历点云中的每个点，根据点的距离和角度信息，
 * 将点分配到扫描上下文的环和扇区中，并记录每个网格的最大高度。
 * 最后对计数较少的网格进行清零处理。
 * 
 * @param pt_cloud 输入的点云指针
 * @return Eigen::MatrixXf 转换后的扫描上下文矩阵
 */
Eigen::MatrixXf ScanContext::ptCloud2ScanContext(pcl::PointCloud<PointType>::Ptr pt_cloud){
  // 初始化最大高度矩阵，用于存储每个网格的最大高度
  Eigen::MatrixXf max_bin = Eigen::MatrixXf::Zero(_num_rings, _num_sectors);
  // 初始化计数器矩阵，用于记录每个网格的点数量
  Eigen::MatrixXf bin_counter = Eigen::MatrixXf::Zero(_num_rings, _num_sectors);

  // 获取点云中的点数量
  int num_points = pt_cloud->points.size();

  // 遍历点云中的每个点
  for (int i = 0; i < num_points; ++i){
    // 获取当前点的信息
    PointType point_this = pt_cloud->points[i];
    // 计算当前点到原点的距离
    float range = sqrt(point_this.x*point_this.x + point_this.y*point_this.y);
    // 计算当前点相对于原点的角度
    float theta = xy2Theta(point_this.x, point_this.y);

    // 找到当前点对应的环索引
    // 环索引范围: 0 ~ num_rings - 1
    int ring_index = floor(range/_gap);
    // 如果环索引超出范围，将其设置为最大环索引
    if (ring_index >= _num_rings)
      ring_index = _num_rings - 1;

    // 找到当前点对应的扇区索引
    // 扇区索引范围: 1 ~ num_sectors - 1
    int sector_index = ceil(theta/_angle_one_sector);
    // 如果扇区索引为 0，跳过当前点
    if (sector_index == 0)
      continue;
    // 如果扇区索引超出范围，将其设置为最大扇区索引
    else if(sector_index > _num_sectors || sector_index < 1)
      sector_index = _num_sectors - 1;
    else
      // 调整扇区索引使其从 0 开始
      sector_index = sector_index - 1;

    // 如果当前点的 z 坐标大于该网格的最大高度，更新最大高度
    if (point_this.z > max_bin(ring_index, sector_index))
      max_bin(ring_index, sector_index) = point_this.z;

    // 对应网格的点计数器加 1
    bin_counter(ring_index, sector_index)++;
  }

  // 遍历最大高度矩阵和计数器矩阵
  for (int i = 0; i < _num_rings; i++){
    for (int j = 0; j < _num_sectors; j++){
      // 如果某个网格的点数量少于 5 个，将该网格的最大高度置为 0
      if(bin_counter(i,j)<5)
        max_bin(i,j) = 0;
    }
  }
  // 返回处理后的最大高度矩阵
  return max_bin;
}

/**
 * @brief 从扫描上下文矩阵计算环形键向量
 * 
 * 该函数遍历扫描上下文矩阵的每一行，统计每行中非零元素的比例，
 * 并将这些比例存储在环形键向量中。
 * 
 * @param sc_bin 输入的扫描上下文矩阵
 * @return Eigen::VectorXf 计算得到的环形键向量
 */
Eigen::VectorXf ScanContext::scanContext2RingKey(Eigen::MatrixXf sc_bin){
  // 初始化环形键向量
  Eigen::VectorXf ringkey = Eigen::VectorXf::Zero(_num_rings);
  int nonzeros;
  // 遍历扫描上下文矩阵的每一行
  for (int i = 0; i< _num_rings; i++){
    // 初始化非零元素计数器
    nonzeros = 0;
    // 遍历当前行的每个元素
    for (int j = 0; j < _num_sectors; j++)
      // 如果当前元素不为 0，非零元素计数器加 1
      if (sc_bin(i,j) != 0)
        nonzeros++;
    // 计算当前行非零元素的比例并存储在环形键向量中
    ringkey(i) = float(nonzeros)/float(_num_sectors);
  }
  // 返回计算得到的环形键向量
  return ringkey;
}

/**
 * @brief 根据点的 x 和 y 坐标计算其相对于原点的角度
 * 
 * 该函数根据点所在的象限，计算其相对于原点的角度（范围 0 ~ 360 度）。
 * 
 * @param x 点的 x 坐标
 * @param y 点的 y 坐标
 * @return float 点相对于原点的角度
 */
float ScanContext::xy2Theta(float x, float y){
  // 第一象限
  if ( x>=0 && y>=0)
    return 180/M_PI * atan(y/x);

  // 第二象限
  if ( x<0 && y>=0)
    return 180 - ((180/M_PI) * atan(y/(-x)));

  // 第三象限
  if (x < 0 && y < 0)
    return 180 + ((180/M_PI) * atan(y/x));

  // 第四象限
  if ( x >= 0 && y < 0)
    return 360 - ((180/M_PI) * atan((-y)/x));
}