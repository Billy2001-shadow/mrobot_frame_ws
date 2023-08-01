#ifndef MROBOT_FRAME_MAPPING_HPP_
#define MROBOT_FRAME_MAPPING_HPP_

#include <Eigen/Dense>
#include <string>
#include <yaml-cpp/yaml.h>

#include "mrobot_frame/models/cloud_filter/voxel_filter.hpp"
#include "mrobot_frame/publisher/gridmap_publisher.hpp"
#include "mrobot_frame/sensor_data/cloud_data.hpp"
#include "mrobot_frame/sensor_data/key_frame.hpp"
#include "mrobot_frame/sensor_data/pose_data.hpp"
#include "nav_msgs/GetMap.h"
#include <ros/ros.h>
typedef struct gridindex_ {
  int x;
  int y;

  void SetIndex(int x_, int y_) {
    x = x_;
    y = y_;
  }
} GridIndex;

typedef struct map_params {
  double log_occ, log_free;
  double log_max, log_min;   //每个栅格的最大最小值
  double resolution;         //地图分辨率
  double origin_x, origin_y; //世界坐标系下cell(0,0)的坐标
  int height, width;         //地图的长和宽
  int offset_x, offset_y; //地图中心（offset_x，offset_y）为地图原点
} MapParams;

namespace mrobot_frame {
class Mapping {
public:
  Mapping();

  bool SaveMap();
  nav_msgs::OccupancyGrid GetCurrentMap();
  // void OccupanyMapping(CloudData &cloud_data, PoseData &robot_poses);
  // //外部调用需要
  void OccupanyMapping(KeyFrame &current_keyframe); //外部调用需要
  // std::mutex map_mutex_;

private:
  //地图参数
  MapParams mapParams; //地图参数可以共用
  unsigned char *pMap; //指向unsigned char类型元素的指针

  unsigned long *pMapHits;
  unsigned long *pMapMisses;

  unsigned long *pMapW;
  double *pMapTSDF;

  bool InitWithConfig();
  bool InitParam(const YAML::Node &config_node);

  //建图用到的工具函数
  std::vector<GridIndex> TraceLine(int x0, int y0, int x1, int y1);
  GridIndex ConvertWorld2GridIndex(double x, double y);
  int GridIndexToLinearIndex(GridIndex index);
  bool isValidGridIndex(GridIndex index);
  void DestoryMap();
};
} // namespace mrobot_frame

#endif