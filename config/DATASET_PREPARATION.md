# ERASOR 数据集准备指南

本文档详细说明了在使用ERASOR进行动态点云去除时，自己的数据集需要提供的内容和格式要求。

## 概述

ERASOR (Egocentric Ratio of Pseudo Occupancy-based Dynamic Object Removal) 是一个用于静态3D点云地图构建的动态物体去除算法。要在自己的数据集上运行ERASOR，需要准备特定格式的数据文件。

## 必需的数据文件

### 1. 点云数据文件夹 (`pcds/`)

- **格式**：PCD文件（Point Cloud Data）
- **命名规则**：按序号命名，使用6位数字格式
  ```
  000000.pcd
  000001.pcd
  000002.pcd
  ...
  ```
- **内容**：每个PCD文件包含一帧激光雷达点云数据
- **字段要求**：
  - 必须包含：x, y, z 坐标
  - 建议包含：intensity 强度信息
- **数据质量**：确保点云数据中没有过多的NaN值或异常点

### 2. 位姿文件 (`poses_lidar2body.csv`)

- **格式**：CSV文件，逗号分隔
- **结构**：
  ```csv
  frame_id,timestamp,x,y,z,qx,qy,qz,qw
  0,1234567890.123,1.0,2.0,3.0,0.0,0.0,0.0,1.0
  1,1234567890.223,1.1,2.1,3.1,0.0,0.0,0.1,0.99
  ...
  ```
- **字段说明**：
  - `frame_id`: 帧序号
  - `timestamp`: 时间戳
  - `x, y, z`: 位置坐标（索引2,3,4）
  - `qx, qy, qz, qw`: 四元数旋转（索引5,6,7,8）
- **重要说明**：
  - 第一行为标题行，程序会自动跳过
  - 位姿应相对于body frame
  - 确保第i个PCD文件对应poses文件中的第i行位姿

### 3. 全局地图文件 (`dense_global_map.pcd`)

- **格式**：PCD文件
- **内容**：完整的全局点云地图
- **用途**：作为ERASOR算法的参考地图，用于识别动态物体
- **要求**：应包含环境的完整静态结构

## 数据集目录结构

```
DATA_DIR/
├── dense_global_map.pcd     # 全局地图文件
├── poses_lidar2body.csv     # 位姿文件
└── pcds/                    # 点云数据文件夹
    ├── 000000.pcd
    ├── 000001.pcd
    ├── 000002.pcd
    ├── 000003.pcd
    └── ...
```

## 配置文件设置

### 修改 `config/your_own_env.yaml`

```yaml
# 数据路径配置
data_dir: "/path/to/your/dataset"  # 指向包含上述文件的目录

# 处理参数
voxel_size: 0.075    # 体素大小，影响处理精度和速度
init_idx: 0          # 起始帧索引
interval: 2          # 处理间隔，每隔几帧处理一次

# 坐标变换（lidar到body的变换）
tf:
    lidar2body: [0.0, 0.0, 0.7, 0, 0.0, 0.0, 1.0]  # [x,y,z,qx,qy,qz,qw]

# ERASOR算法参数
erasor:
    max_range: 9.5              # 最大处理距离 (米)
    num_rings: 8                # 径向网格数量
    num_sectors: 60             # 角度网格数量
    min_h: -1.6                 # 最小高度 (米) - 需根据机器人调整
    max_h: 1.3                  # 最大高度 (米) - 需根据机器人调整
    th_bin_max_h: -1.0          # 网格最大高度阈值
    scan_ratio_threshold: 0.2   # 扫描比率阈值，越大越激进
    minimum_num_pts: 5          # 最小点数要求
    rejection_ratio: 0          # 拒绝比率
    gf_dist_thr: 0.075         # 地面过滤距离阈值
    gf_iter: 3                 # 地面过滤迭代次数
    gf_num_lpr: 12             # 地面过滤LPR数量
    gf_th_seeds_height: 0.5    # 地面种子高度阈值
    version: 3                 # 算法版本 (2: R-GPF / 3: R-GPF w/ blocking)

# 其他参数
verbose: true                  # 是否显示详细信息
```

## 重要注意事项

### 1. 数据对应关系
- **严格对应**：确保第i个PCD文件对应poses文件中的第i行位姿
- **时间同步**：点云数据和位姿数据应该时间同步

### 2. 坐标系一致性
- **统一坐标系**：所有数据应在同一坐标系下，通常是body frame
- **变换矩阵**：正确设置lidar2body变换参数

### 3. 参数调整
- **高度参数**：`min_h` 和 `max_h` 需要根据你的机器人和环境特点调整
- **处理范围**：`max_range` 应根据激光雷达性能和环境大小设置
- **网格分辨率**：`num_rings` 和 `num_sectors` 影响算法精度和计算量

### 4. 数据质量要求
- **位姿精度**：位姿数据的精度直接影响动态物体检测的效果
- **点云密度**：确保点云数据有足够的密度
- **环境覆盖**：全局地图应覆盖所有需要处理的区域

## 数据准备流程

### 方法1：从rosbag转换
如果你有rosbag数据，可以使用项目中的转换脚本：

```bash
# 1. 启动roscore
roscore

# 2. 播放rosbag
rosbag play your_data.bag

# 3. 运行转换脚本
cd scripts/semantickitti2bag/
python3 to_erasor_data.py --lidar_topic /your/lidar/topic \
                          --world_frame world \
                          --lidar_frame lidar
```

### 方法2：直接准备数据
1. 将点云数据保存为PCD格式
2. 准备对应的位姿CSV文件
3. 生成或获取全局地图PCD文件
4. 按照上述目录结构组织文件

## 运行ERASOR

### 1. 配置参数
```bash
# 编辑配置文件
gedit config/your_own_env.yaml
```

### 2. 编译项目
```bash
# 在catkin工作空间根目录
cd ~/owen_erasor
catkin_make
source devel/setup.bash
```

### 3. 运行算法
```bash
# 启动ERASOR
roslaunch erasor run_erasor_in_your_env_vel16.launch
```

## 输出结果

运行完成后，ERASOR会生成：
- `staticmap_via_erasor.pcd`：去除动态物体后的静态地图
- 各种调试和可视化文件

## 故障排除

### 常见问题
1. **文件找不到**：检查data_dir路径是否正确
2. **位姿格式错误**：确认CSV文件格式和字段顺序
3. **坐标系不匹配**：检查lidar2body变换参数
4. **内存不足**：减小处理范围或降低网格分辨率

### 调试建议
1. 先用小数据集测试参数配置
2. 检查RViz中的可视化结果
3. 查看终端输出的调试信息
4. 逐步调整算法参数

## 参考资料

- [ERASOR论文](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9361109)
- [项目主页](https://github.com/LimHyungTae/ERASOR)
- [演示视频](https://www.youtube.com/watch?v=Nx27ZO8afm0)

---

**注意**：本指南基于ERASOR项目的实际代码分析，如有疑问请参考项目源码和官方文档。