#include "utility.h"
// #include "disco_double/cloud_info.h"
#include "disco_double/ring_cloud_info.h"

// 定义点云平滑度结构体，用于存储曲率值和索引
struct smoothness_t{ 
    float value;  // 曲率值
    size_t ind;   // 点在点云中的索引
};

// 定义排序函数对象，用于按曲率值排序
struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

/*
 * 特征提取类，继承自参数服务器
 * 主要功能：
 * 1. 计算点云曲率
 * 2. 标记遮挡点和平行光束点
 * 3. 提取角点和平面点特征
 */
//  表示 FeatureExtraction 类公有继承自 ParamServer 类
class FeatureExtraction : public ParamServer
{
public:
    // ROS订阅器和发布器
    ros::Subscriber subLaserCloudInfo;  // 订阅去畸变后的点云信息
    ros::Publisher pubLaserCloudInfo;   // 发布带特征的点云信息
    ros::Publisher pubCornerPoints;     // 发布角点特征
    ros::Publisher pubSurfacePoints;    // 发布平面点特征

    // 点云容器
    pcl::PointCloud<PointType>::Ptr extractedCloud;  // 输入点云
    pcl::PointCloud<PointType>::Ptr cornerCloud;     // 角点特征点云
    pcl::PointCloud<PointType>::Ptr surfaceCloud;    // 平面特征点云

    pcl::VoxelGrid<PointType> downSizeFilter;  // 降采样滤波器

    // disco_double::cloud_info cloudInfo;  // 点云信息
    disco_double::ring_cloud_info cloudInfo;  // 点云信息
    std_msgs::Header cloudHeader;      // 点云头信息

    // 特征提取相关变量
    std::vector<smoothness_t> cloudSmoothness;  // 点云平滑度(曲率)数组
    float *cloudCurvature;    // 点云曲率数组
    int *cloudNeighborPicked;  // 邻域点选取标记数组
    int *cloudLabel;           // 点标签数组(1=角点, -1=平面点)

    /*
     * 构造函数，初始化订阅器和发布器
     */
    FeatureExtraction()
    {
        // 订阅去畸变后的点云信息
        // subLaserCloudInfo = nh.subscribe<disco_double::cloud_info>(robot_id + "/disco_double/deskew/cloud_info", 1, &FeatureExtraction::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        subLaserCloudInfo = nh.subscribe<disco_double::ring_cloud_info>(robot_id + "/disco_double/deskew/cloud_info", 1, &FeatureExtraction::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());

        // 初始化发布器
        // pubLaserCloudInfo = nh.advertise<disco_double::cloud_info> (robot_id + "/disco_double/feature/cloud_info", 1);
        pubLaserCloudInfo = nh.advertise<disco_double::ring_cloud_info> (robot_id + "/disco_double/feature/cloud_info", 1);
        pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>(robot_id + "/disco_double/feature/cloud_corner", 1);
        pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>(robot_id + "/disco_double/feature/cloud_surface", 1);
        
        initializationValue();  // 初始化变量
    }

    /*
     * 初始化变量和内存
     */
    void initializationValue()
    {
        cloudSmoothness.resize(N_SCAN*Horizon_SCAN);  // 调整平滑度数组大小

        // 设置降采样滤波器参数
        downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

        // 初始化点云指针
        extractedCloud.reset(new pcl::PointCloud<PointType>());
        cornerCloud.reset(new pcl::PointCloud<PointType>());
        surfaceCloud.reset(new pcl::PointCloud<PointType>());

        // 分配动态数组内存
        cloudCurvature = new float[N_SCAN*Horizon_SCAN];
        cloudNeighborPicked = new int[N_SCAN*Horizon_SCAN];
        cloudLabel = new int[N_SCAN*Horizon_SCAN];
    }

    /*
     * 点云信息处理回调函数
     * @param msgIn 输入的点云信息消息
     */
    // 重要线程：处理点云信息，提取特征并发布
    // void laserCloudInfoHandler(const disco_double::cloud_infoConstPtr& msgIn)
    void laserCloudInfoHandler(const disco_double::ring_cloud_infoConstPtr& msgIn)
    {
        cloudInfo = *msgIn;  // 保存点云信息
        cloudHeader = msgIn->header;  // 保存头信息
        pcl::fromROSMsg(msgIn->cloud_deskewed, *extractedCloud);  // 转换ROS消息为PCL点云

        calculateSmoothness();  // 计算平滑度(曲率)（只有水平方向有曲率）
        markOccludedPoints();   // 标记遮挡点和平行点
        extractFeatures();       // 提取特征
        publishFeatureCloud();   // 发布特征点云
    }

    /*
     * 计算点云平滑度(曲率)
     * 使用前后5个点的距离差计算曲率
     */
    void calculateSmoothness()
    {
        int cloudSize = extractedCloud->points.size();
        for (int i = 5; i < cloudSize - 5; i++)
        {
            // 计算当前点与前后5个点的距离差(类似二阶导数)
            float diffRange = cloudInfo.pointRange[i-5] + cloudInfo.pointRange[i-4]
                            + cloudInfo.pointRange[i-3] + cloudInfo.pointRange[i-2]
                            + cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i] * 10
                            + cloudInfo.pointRange[i+1] + cloudInfo.pointRange[i+2]
                            + cloudInfo.pointRange[i+3] + cloudInfo.pointRange[i+4]
                            + cloudInfo.pointRange[i+5];            

            cloudCurvature[i] = diffRange*diffRange;  // 曲率=距离差的平方

            // 初始化标记数组
            cloudNeighborPicked[i] = 0;
            cloudLabel[i] = 0;
            
            // 保存平滑度值用于排序
            cloudSmoothness[i].value = cloudCurvature[i];
            cloudSmoothness[i].ind = i;
        }
    }

    /*
     * 标记遮挡点和平行光束点(需去除，这两种特征点不固定，不可靠)
     * 遮挡点: 深度突变较大的点(如深度变化超过0.3米)
     * 平行光束点（几乎平行于激光光束，前一帧距离近，后一帧无穷远）: 两侧距离变化较大的点(如距离变化超过0.05米)
     * **********            **********
     * *********              **********
     * ********                **********
     * *******^(平行光束点)    ^^^**********
     * ****** ^          ^^^   (特征点)********
     * ***** ^       ^^^         *********
     * ****  ^   ^^^             *********
     * ***  &(激光雷达)            ***********
     * 
     * *******************^(遮挡点)
     * ******            ^
     * ******           ^
     * ******          ^***************
     * *****          ^^   (特征点)*******
     *               ^^          ******
     *              ^^
     *             &(激光雷达)
     */
    void markOccludedPoints()
    {
        int cloudSize = extractedCloud->points.size();
        // mark occluded points and parallel beam points
        for (int i = 5; i < cloudSize - 6; ++i)
        {
            // 检查遮挡点(深度突变)
            float depth1 = cloudInfo.pointRange[i];
            float depth2 = cloudInfo.pointRange[i+1];
            int columnDiff = std::abs(int(cloudInfo.pointColInd[i+1] - cloudInfo.pointColInd[i]));

            // 如果相邻点在图像中距离较近但深度差较大，则标记为遮挡点
            if (columnDiff < 10){
                // 10 pixel diff in range image
                
                // 前向遮挡检测：当前点比后一个点深度大0.3米以上
                if (depth1 - depth2 > 0.3){
                    // 标记当前点及其前5个邻域点为遮挡点(1表示被遮挡)
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }
                // 后向遮挡检测：后一个点比当前点深度大0.3米以上  
                else if (depth2 - depth1 > 0.3){
                    // 标记当前点及其后6个邻域点为遮挡点
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }
            
            // 检查平行光束点(两侧距离变化较大)
            float diff1 = std::abs(float(cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i]));
            float diff2 = std::abs(float(cloudInfo.pointRange[i+1] - cloudInfo.pointRange[i]));

            if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
                cloudNeighborPicked[i] = 1;
        }
    }

    /*
     * 提取特征点(角点和平面点)
     */
    void extractFeatures()
    {
        cornerCloud->clear();  // 清空角点云
        surfaceCloud->clear(); // 清空平面点云

        // 创建一个智能指针 surfaceCloudScan，用于临时存储单条激光线束上的平面点云数据
        // 在后续处理中，会将每条激光线束上的平面点云数据存储在这个指针所指向的点云对象中
        pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());

        // 创建一个智能指针 surfaceCloudScanDS，用于存储经过降采样处理后的平面点云数据
        // 降采样可以减少点云数据量，提高处理效率，处理后的点云数据将存储在这个指针所指向的点云对象中
        pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

        // 解释：surfaceCloudScan是除角点外的所有点云，surfaceCloudScanDS是降采样后的点云
        // 问题是：特征提取加速ICP的计算，特征提取的点云数量和计算精度之间的矛盾。
        // 例如：角点阈值太小，个数过多，限制角点个数，或者平面点个数过多（一般是这样的），怎么进行取舍，只能降采样
        // 遍历每条激光线束
        for (int i = 0; i < N_SCAN; i++)
        {
            surfaceCloudScan->clear();

            // 将每条线束分成6段处理
            for (int j = 0; j < 6; j++)
            {
                // 计算当前段的起始和结束索引
                int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
                int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

                if (sp >= ep)
                    continue;

                // 按曲率值排序当前段的点
                std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());

                // 提取角点特征(曲率较大的点)
                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--)
                {
                    int ind = cloudSmoothness[k].ind;
                    // 检查是否未被选取且曲率超过阈值
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold)
                    {
                        largestPickedNum++;
                        if (largestPickedNum <= 20){  // 每段最多选取20个角点（降低ICP运算复杂度，提高速度）
                            cloudLabel[ind] = 1;  // 标记为角点
                            cornerCloud->push_back(extractedCloud->points[ind]);
                        } else {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;
                        // 标记邻近点，避免选取过于密集的特征（防止特征点过密）
                        for (int l = 1; l <= 5; l++)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                // 提取平面点特征(曲率较小的点)
                // int largestPickedNumS = 0;
                for (int k = sp; k <= ep; k++)
                {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold)
                    {
                        // largestPickedNumS++;
                        // if (largestPickedNumS <= 20){  // 每段最多选取20个平面点（降低ICP运算复杂度，提高速度）
                            cloudLabel[ind] = -1;  // 标记为平面点
                            // surfaceCloud->push_back(extractedCloud->points[ind]);
                            surfaceCloudScan->push_back(extractedCloud->points[ind]);
                        // } else {
                        //     break;
                        // }

                        // cloudLabel[ind] = -1;  // 标记为平面点
                        cloudNeighborPicked[ind] = 1;

                        // 标记邻近点（防止特征点过密）
                        for (int l = 1; l <= 5; l++) {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }
                
                // 收集当前段的平面点（问题：extractedCloud并不是平面点，是除角点外的全部点云）
                // 平面点云过多的问题：如果限制个数，导致快速达到平面点云上限，从而消失大量点云
                // 解决方法：平面点云提取没有上限，然后再降采样，这样可以保证平面点云的数量不会过多
                for (int k = sp; k <= ep; k++)
                {
                    if (cloudLabel[k] <= 0){  // 非角点(平面点或未分类点)
                        surfaceCloudScan->push_back(extractedCloud->points[k]);
                    }
                }
            }

            // 对平面点进行降采样
            surfaceCloudScanDS->clear();
            downSizeFilter.setInputCloud(surfaceCloudScan);
            downSizeFilter.filter(*surfaceCloudScanDS);

            *surfaceCloud += *surfaceCloudScanDS;  // 合并所有线束的平面点
        }
    }

    /*
     * 释放点云信息内存
     */
    void freeCloudInfoMemory()
    {
        cloudInfo.startRingIndex.clear();
        cloudInfo.endRingIndex.clear();
        cloudInfo.pointColInd.clear();
        cloudInfo.pointRange.clear();
    }

    /*
     * 发布特征点云
     */
    void publishFeatureCloud()
    {
        freeCloudInfoMemory();  // 释放内存
        
        // 发布角点和平面点云
        cloudInfo.cloud_corner  = publishCloud(&pubCornerPoints,  cornerCloud,  cloudHeader.stamp, robot_id + "/" + lidarFrame);
        cloudInfo.cloud_surface = publishCloud(&pubSurfacePoints, surfaceCloud, cloudHeader.stamp, robot_id + "/" + lidarFrame);
        
        // 发布带特征的点云信息
        pubLaserCloudInfo.publish(cloudInfo);
    }
};

/*
 * 主函数
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "disco_double");  // 初始化ROS节点

    FeatureExtraction FE;  // 创建特征提取对象

    ROS_INFO("\033[1;32m----> Feature Extraction Started.\033[0m");  // 打印启动信息

    // 单线程运行
    ros::spin();  // 进入ROS事件循环

    return 0;
}