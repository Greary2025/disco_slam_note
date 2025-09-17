# 设置C++标准为14并设为必需
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置CMake最低版本要求
cmake_minimum_required(VERSION 3.5)
# 定义项目名称
project(disco_double)

# 设置构建类型为Release
set(CMAKE_BUILD_TYPE "Release")
# 设置C++编译标志
set(CMAKE_CXX_FLAGS "-std=c++14")
# 设置Release模式的编译优化选项
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

# 查找catkin包及其组件
find_package(catkin REQUIRED COMPONENTS
  tf       # TF变换
  roscpp   # ROS C++接口
  rospy    # ROS Python接口
  cv_bridge # OpenCV桥接
  # pcl库相关
  pcl_conversions
  # ROS消息类型
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation # 消息生成
)

# 查找其他依赖包
find_package(OpenMP REQUIRED)   # OpenMP并行计算
find_package(PCL REQUIRED QUIET) # PCL点云库
find_package(OpenCV REQUIRED QUIET) # OpenCV
find_package(GTSAM REQUIRED QUIET) # GTSAM库
find_package(Eigen3 REQUIRED)   # Eigen数学库
find_package(libnabo REQUIRED)  # libnabo库

# 添加自定义消息文件
add_message_files(
  DIRECTORY msg
  FILES
  # cloud_info.msg    # 点云信息消息
  ring_cloud_info.msg
  context_info.msg  # 上下文信息消息
)

# 生成消息
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

# 定义catkin包属性
catkin_package(
  INCLUDE_DIRS include  # 包含目录
  DEPENDS PCL GTSAM     # 外部依赖
  
  CATKIN_DEPENDS        # catkin依赖
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime 
  message_generation
)

# 设置包含目录
include_directories(
	include  # 项目头文件目录
	${catkin_INCLUDE_DIRS}  # catkin包含目录
	${PCL_INCLUDE_DIRS}     # PCL包含目录
  ${OpenCV_INCLUDE_DIRS}  # OpenCV包含目录
	${GTSAM_INCLUDE_DIR}    # GTSAM包含目录
	src/DiSCo-SLAM/third_parties  # 第三方库目录
)

# 设置链接目录
link_directories(
	include  # 项目库目录
	${PCL_LIBRARY_DIRS}    # PCL库目录
  ${OpenCV_LIBRARY_DIRS} # OpenCV库目录
  ${GTSAM_LIBRARY_DIRS}  # GTSAM库目录
)

###########
## 查看版本 ##
###########

# 添加PCL版本输出（catkin build 会默认省略掉一部分信息）
# message(STATUS "PCL version: ${PCL_VERSION}")
# message(STATUS "PCL include dirs: ${PCL_INCLUDE_DIRS}")
# message(STATUS "PCL library dirs: ${PCL_LIBRARY_DIRS}")
# message(STATUS "PCL libraries: ${PCL_LIBRARIES}")

# 在find_package(PCL)之后添加
message(WARNING "==============================================")
message(WARNING "PCL version: ${PCL_VERSION}")
message(WARNING "==============================================")

###########
## 构建 ##
###########

# 用于融合
# 最大团求解器
add_library(fast_max-clique_finder
	src/DiSCo-SLAM/third_parties/fast_max-clique_finder/src/findClique.h
	src/DiSCo-SLAM/third_parties/fast_max-clique_finder/src/graphIO.h
	src/DiSCo-SLAM/third_parties/fast_max-clique_finder/src/findClique.cpp
	src/DiSCo-SLAM/third_parties/fast_max-clique_finder/src/findCliqueHeu.cpp
	src/DiSCo-SLAM/third_parties/fast_max-clique_finder/src/utils.cpp
	src/DiSCo-SLAM/third_parties/fast_max-clique_finder/src/graphIO.cpp
	)
target_compile_options(fast_max-clique_finder PRIVATE -w)

# 扫描上下文库
add_library(scan_context
	src/DiSCo-SLAM/third_parties/scanContext/scanContext.h
	src/DiSCo-SLAM/third_parties/scanContext/scanContext.cpp
)

# 点云投影节点
add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
add_dependencies(${PROJECT_NAME}_imageProjection 
                  ${catkin_EXPORTED_TARGETS} 
                  ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection 
                      ${catkin_LIBRARIES} 
                      ${PCL_LIBRARIES} 
                      ${OpenCV_LIBRARIES})

# 特征提取节点
add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
add_dependencies(${PROJECT_NAME}_featureExtraction 
                ${catkin_EXPORTED_TARGETS} 
                ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_featureExtraction 
                      ${catkin_LIBRARIES} 
                      ${PCL_LIBRARIES} 
                      ${OpenCV_LIBRARIES})

# 地图优化节点
add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmization.cpp)
add_dependencies(${PROJECT_NAME}_mapOptmization 
                ${catkin_EXPORTED_TARGETS} 
                ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE 
                      ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapOptmization 
                      ${catkin_LIBRARIES} 
                      ${PCL_LIBRARIES} 
                      ${OpenCV_LIBRARIES} 
                      ${OpenMP_CXX_FLAGS} 
                      gtsam)

# IMU预积分节点
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration 
${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

# 地图融合节点
add_executable(${PROJECT_NAME}_mapFusion src/DiSCo-SLAM/mapFusion.cpp)
add_dependencies(${PROJECT_NAME}_mapFusion 
                ${catkin_EXPORTED_TARGETS}  
                ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_mapFusion 
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES} libnabo::nabo
                      gtsam fast_max-clique_finder scan_context)

# PCD保存节点
add_executable(${PROJECT_NAME}_save_pcd src/save_pcd.cpp)
add_dependencies(${PROJECT_NAME}_save_pcd 
                ${catkin_EXPORTED_TARGETS} 
                ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_save_pcd 
                      ${catkin_LIBRARIES} 
                      ${PCL_LIBRARIES})

# ERASOR格式PCD保存节点
add_executable(${PROJECT_NAME}_save_pcd_erasor src/save_pcd_erasor.cpp)
add_dependencies(${PROJECT_NAME}_save_pcd_erasor 
                ${catkin_EXPORTED_TARGETS} 
                ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_save_pcd_erasor 
                      ${catkin_LIBRARIES} 
                      ${PCL_LIBRARIES})

# 安装Python脚本(已注释)
#catkin_install_python(PROGRAMS scripts/listener.py
#	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
