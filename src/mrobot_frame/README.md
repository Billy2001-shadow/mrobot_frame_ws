# m_robot_frame
激光SLAM框架的搭建





## 代码框架说明

#### 1.数据预处理部分

```
launch文件

节点名称：mrobot_frame_data_pretreat_node
节点内代码很简洁：
新建一个智能指针管理数据预处理类，然后循环执行类内run函数
```



2.







### 安装第三方库

[安装Ceres库](https://cxyzjd.com/article/Wadewhl/112648707)

[安装g2o库](https://github.com/RainerKuemmerle/g2o/tree/9b41a4ea5ade8e1250b9c1b279f3a9c098811b5a)



sudo ldconfig #更新缓存

g2o版本不能太高，不然容易报奇奇怪怪的错误

现在用的是slam十四讲里面的版本

```
roslaunch mrobot_frame data_pretreat.launch
```









## 实验结果

### lesson5.bag

#### 激光里程计

![image-20230408210204512](/home/chenw/ROS1/baseline_ws/src/mrobot_frame/README.assets/image-20230408210204512.png)





```
    position: 
          x: 2.0474998951
          y: 12.6241998672
          z: 0.0
    orientation: 
          x: 0.0
          y: 0.0
          z: 0.965952515602
          w: -0.258719444275
```

```text
rosservice call /optimize_map  //输入这个指令后，它会对当前所有的关键帧统一做一次优化，并用优化后的位姿重新产生一次全局地图，发在rviz上

rosservice call /save_map //地图生成完之后终端上会提示地图保存路径。
做了优化后，位姿不再有累计误差，得到的点云地图是闭合的

同时，每次执行优化操作以后，rviz上会出现一条绿色的轨迹，那就是优化之后的关键帧位姿，可以把它和轮式里程计位姿轨迹做对比显示。
```



了解以下优化后的关键帧，激光雷达点云+对应位姿，

对应的位姿是map坐标系下的吗-



跟踪器的设计需要我们这边出手、小陀螺检测器的设计

将目标送入跟踪器中得到目标在制定惯性系下的位置和速度，再经过小陀螺状态检测器的处理后，发布最终的目标位置和速度

惯性系：以云台中心为原点，IMU上电时的Yaw朝向为x轴的惯性系





PC到了之后，需要将cj的代码读懂，和电控想反陀螺的方案。
