# mrobot_frame_ws

本程序为一套可解耦的SLAM框架，该激光SLAM框架主要分为数据预处理模块、前端里程计模块、后端优化模块、回环检测模块以及建图模块。

## 0.前言

出于对激光SLAM可扩展性的考虑，对激光SLAM框架进行重构。程序整体的设计目的在于尽可能保留充足的可扩展性，各模块充分解耦、即插即用，做到程序二次开发简便、可扩展性高。

## 1.开始

本程序适用含有激光雷达扫描数据的bag包。

功能：提供机器人的位姿并且建立环境地图。(目前可在2D环境下运行)

部署前需要自行准备并更改的有：

- 更换launch文件中需要测试的bag包
- 更换launch文件中bag包中topic的名称

## 1.1安装

1. Clone the repository

```
git clone https://github.com/Billy2001-shadow/mrobot_frame_ws.git
```

2. Download ros dependencies

```
sudo apt update && sudo apt-get install -y 
```

3. Build  the package and source the `setup.bash` or `setup.zsh` file.

```
cd mrobot_frame_ws/
catkin_make -j6
source ./devel/setup.bash 
```



### 1.2 Quick start

#### **1.2.1 Run the  front_end process**

##### configure launch file

```xml
<launch>

    <!-- bag的地址与名称（需要用户自己选择需要测试的功能包） -->
    <arg name="bag_filename" default="/home/cw/Slam/bag_flies/basic_localization_stage_indexed.bag"/> 
    <!-- 使用bag的时间戳 -->
    <param name="use_sim_time" value="true" />
	 <!-- 需要加载到参数服务器中的参数（需要用户根据bag包中各topic的名称进行相应的更改） -->
    <param name="scan_topic" value="base_scan"/>
    <param name="cloud_topic" value="pretreat_cloud"/>
    <param name="laser_frame" value="base_laser"/>
    <param name="odom_frame" value="odom"/> 

    <!-- 启动节点 -->
    <node name="mrobot_frame_data_pretreat_node" pkg="mrobot_frame" type="mrobot_frame_data_pretreat_node" output="screen" />
    <node name="mrobot_frame_front_end_node" pkg="mrobot_frame" type="mrobot_frame_front_end_node" output="screen" />
    <!-- launch rviz -->
    <node name="rviz" pkg="rviz" type="rviz" required="false"
        args="-d $(find mrobot_frame)/rviz/front_end.rviz" />
    <!-- play bagfile -->
    <node name="playbag" pkg="rosbag" type="play" args="--clock $(arg bag_filename)" />
 
</launch>
```

##### configure init pose for lidar

```yaml
#激光雷达在里程计坐标系下的起始位姿（需要用户根据自己的bag包更改激光雷达的初始位姿）
pose: [0.275,0.0,0.150,0.0,0.0,0.0,1.0] #basic_localization_stage_indexed.bag
#pose: [-0.001,0.013,0.0,0.0,0.0,-0.707,0.707] #aces.bag
# 匹配
registration_method: NDT   # 选择点云匹配方法，目前支持：NDT


# 当前帧
# no_filter指不对点云滤波，在匹配中，理论上点云越稠密，精度越高，但是速度也越慢
# 所以提供这种不滤波的模式做为对比，以方便使用者去体会精度和效率随稠密度的变化关系
frame_filter: no_filter # 选择当前帧点云滤波方法，目前支持：voxel_filter、no_filter

# 局部地图
key_frame_distance: 0.25 # 关键帧距离0.25
key_frame_angular: 0.088 #5°
local_frame_num: 9       #9
local_map_filter: no_filter # 选择滑窗地图点云滤波方法，目前支持：voxel_filter、no_filter


# 各配置选项对应参数
## 匹配相关参数
NDT:
    res : 1.0
    step_size : 0.12
    trans_eps : 0.001
    max_iter : 100
## 滤波相关参数
voxel_filter:
    local_map:
        leaf_size: [0.6, 0.6, 0.6]
    frame:
        leaf_size: [1.3, 1.3, 1.3]
```

##### run launch file

```
roslaunch mrobot_frame front_end.launch
```

**2. Run the  mapping process**

TODO

```
roslaunch mrobot_frame mapping.launch
```



### 2.环境配置

- Ubuntu 20.04 LTS
- ROS1 Noetic
- PCL 1.10（ROS安装附带）
- Eigen3
- [g2o库](https://github.com/RainerKuemmerle/g2o/tree/9b41a4ea5ade8e1250b9c1b279f3a9c098811b5a)
- [Ceres](https://github.com/ceres-solver/ceres-solver)
- [glog](https://github.com/google/glog)









## 更新日志

---

### 2023.8.01

#### 更新

- 将前端里程计中激光雷达的初始位姿部分写入front.yaml文件，方便调整不同数据集中激光雷达不同的初始位姿情况
  - front.yaml文件中pose关键字下对应的列表中七个value分别表示x,y,z,q1,q2,q3,q4,其中q4为实部
  - 获取激光雷达初始位姿可以播放bag包，查看起始状态下odom与laser的相对位姿
- 为了适配不同bag下激光雷达消息的topic名称以及激光雷达坐标系的名称等，在程序中用外部输入的参数代替具体的topic名称，即从参数服务器中读取需要的参数，可在相应的launch文件中更改对应bag中的各话题名称
  - scan_topic：表示bag中激光雷达消息的名称
  - cloud_topic：表示经过预处理之后发布的PCL格式的点云数据的名称
  - laser_frame：表示激光雷达消息对应的激光雷达坐标系的名称
  - odom_frame：表示里程计坐标系的名称

- 注：目前可较为方便的替换bag数据集进行测试，只需要更改launch文件下对应的topic以及front.yaml中激光雷达的初始位姿

#### 还未解决的问题

- aces.bag退化场景下激光里程计表现很差，体现在长廊环境下，机器人在行进过程中，激光里程计的位姿一直停在原地不动，等走过长廊环境后激光里程计位姿又重新更新。
  - 导致的原因：目前单纯只用了点云数据来估计机器人的位姿，在点云匹配时输入的预测位姿是依据简单的匀速运动模型而没有融入其他传感器信息

---

### 2023.8.02

#### 更新

- 更新readme



---

