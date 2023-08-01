# mrobot_frame_ws
可解耦的SLAM框架



## 更新日志

---

### 2023.8.01

#### 完善的功能

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



