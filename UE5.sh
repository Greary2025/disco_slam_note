#!/bin/bash

# 进入UE5目录
cd ~/UE5/

# 加载ROS环境设置
source devel/setup.bash

# 启动rosbridge TCP服务，设置bson_only_mode为True
roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=True
