#include "mrobot_frame/mapping/mapping/mapping.hpp"

#include "glog/logging.h"
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include "mrobot_frame/global_defination/global_defination.h"
#include "mrobot_frame/models/cloud_filter/voxel_filter.hpp"
#include "mrobot_frame/tools/file_manager.hpp"

namespace mrobot_frame {
Mapping::Mapping() {
  InitWithConfig(); //载入栅格地图参数
}

bool Mapping::InitWithConfig() {
  std::string config_file_path =
      WORK_SPACE_PATH + "/config/mapping/mapping.yaml";
  YAML::Node config_node = YAML::LoadFile(config_file_path);

  std::cout << "-----------------建图初始化-------------------" << std::endl;
  InitParam(config_node);

  return true;
}

bool Mapping::InitParam(const YAML::Node &config_node) {
  mapParams.width = config_node["gridmap_width"].as<int>();
  mapParams.height = config_node["gridmap_height"].as<int>();
  mapParams.resolution = config_node["gridmap_resolution"].as<double>();

  //每次被击中的log变化值，覆盖栅格建图算法需要的参数
  mapParams.log_free = config_node["gridmap_log_free"].as<double>();
  mapParams.log_occ = config_node["gridmap_log_occ"].as<double>();

  //每个栅格的最大最小值．
  mapParams.log_max = config_node["gridmap_log_max"].as<double>();
  mapParams.log_min = config_node["gridmap_log_min"].as<double>();

  mapParams.origin_x = config_node["gridmap_origin_x"].as<double>();
  mapParams.origin_y = config_node["gridmap_origin_y"].as<double>();

  //地图的原点，在地图的正中间
  mapParams.offset_x = config_node["gridmap_offset_x"].as<int>(); // 500
  mapParams.offset_y = config_node["gridmap_offset_y"].as<int>(); // 500

  pMap = new unsigned char
      [mapParams.width *
       mapParams.height]; // pMap为指向数组首个元素的指针(后面主要维护pMap)

  //计数建图算法需要的参数
  //每个栅格被激光击中的次数
  pMapHits = new unsigned long[mapParams.width * mapParams.height];
  //每个栅格被激光通过的次数
  pMapMisses = new unsigned long[mapParams.width * mapParams.height];

  // TSDF建图算法需要的参数
  pMapW = new unsigned long[mapParams.width * mapParams.height];
  pMapTSDF = new double[mapParams.width * mapParams.height];

  //初始化
  for (int i = 0; i < mapParams.width * mapParams.height; i++) {
    pMap[i] = 50;
    pMapHits[i] = 0;
    pMapMisses[i] = 0;
    pMapW[i] = 0;
    pMapTSDF[i] = -1;
  }
  return true;
}

void Mapping::OccupanyMapping(KeyFrame &current_keyframe) {

  LOG(INFO) << "开始建图，请稍后...";
  Eigen::Matrix4f robot_pose =
      current_keyframe.pose; //这个对应的是机器人的位姿还是激光雷达的位姿，
  CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
  std::string file_path = "";
  // std::string key_frames_path_ =
  // "/home/ncu/chenw/mrobot_frame_ws/src/mrobot_frame/slam_data/key_frames";
  std::string key_frames_path_ =
      "/home/cw/Slam/mrobot_frame_ws/src/mrobot_frame/slam_data/key_frames";
  file_path = key_frames_path_ + "/key_frame_" +
              std::to_string(current_keyframe.index) + ".pcd";
  pcl::io::loadPCDFile(file_path, *cloud_ptr); // cloud为点云指针

  //先获取机器人位姿
  Eigen::Matrix3f rotation_matrix =
      robot_pose.block<3, 3>(0, 0); //从(0,0)开始取三行三列
  Eigen::Vector3f eulerAngle = rotation_matrix.eulerAngles(
      2, 1, 0); // ZYX旋转顺序(只有一个θ，不用考虑旋转矩阵中旋转顺序的问题)

  if (eulerAngle(0) > 3.1415926)
    eulerAngle(0) -= 2 * 3.1415926;
  else if (eulerAngle(0) < -3.1415926)
    eulerAngle(0) += 2 * 3.1415926;

  Eigen::Vector3f robotPose(robot_pose(0, 3), robot_pose(1, 3),
                            eulerAngle(0)); // enlerAngle可能差2pai

  LOG(INFO) << "robotPose" << robotPose(0) << "  " << robotPose(1) << "  "
            << robotPose(2) << "  " << current_keyframe.index;

  GridIndex robotIndex = ConvertWorld2GridIndex(robotPose(0), robotPose(1));
  LOG(INFO) << "robotIndex.x = " << robotIndex.x
            << " robotIndex.y = " << robotIndex.y;
  //每一个激光束
  for (int id = 0; id < (*cloud_ptr).size(); id++) {
    // LOG(INFO) << "cloud_ptr.x = " << cloud_ptr->at(id).x
    //           << "cloud_ptr.y = " << cloud_ptr->at(id).y
    //           << "cloud_ptr.z = " << cloud_ptr->at(id).z;
    double theta = robotPose(2);

    double world_x = cos(theta) * cloud_ptr->at(id).x -
                     sin(theta) * cloud_ptr->at(id).x + robotPose(0);

    double world_y = sin(theta) * cloud_ptr->at(id).y +
                     cos(theta) * cloud_ptr->at(id).y + robotPose(1);
    // std::cout << "robotPose_x=  " << robotPose(0) << "  robotPose_y=   " <<
    // robotPose(1) << std::endl; std::cout << "world_x=  " << world_x << "
    // world_y=   " << world_y << std::endl;

    GridIndex beamPointIndex = ConvertWorld2GridIndex(world_x, world_y);
    // std::cout << "beamPointIndex.x=   " << beamPointIndex.x << "
    // beamPointIndex.y=    " << beamPointIndex.y << std::endl;
    std::vector<GridIndex> beamTraceindexes = TraceLine(
        robotIndex.x, robotIndex.y, beamPointIndex.x, beamPointIndex.y);
    for (auto index : beamTraceindexes) {
      // std::cout << "Index_x=  " << index.x << "   Index_y：  " << index.y <<
      // std::endl;

      if (isValidGridIndex(index)) {
        int tmpLinearIndex =
            GridIndexToLinearIndex(index); // tmpLinearIndex一直为1？
        if (pMap[tmpLinearIndex] == 0)
          continue;
        pMap[tmpLinearIndex] += mapParams.log_free;
        // std::cout << "tmpLinearIndex=  " << tmpLinearIndex << "
        // pMap值(路经点)：  " << pMap[tmpLinearIndex] << std::endl;
      } else {
        // std::cerr << "index if invalid!!!" << std::endl;
      }
    } // for

    if (isValidGridIndex(beamPointIndex)) {
      int tmpLinearIndex = GridIndexToLinearIndex(beamPointIndex);
      pMap[tmpLinearIndex] += mapParams.log_occ;
      // std::cout << "tmpLinearIndex=  " << tmpLinearIndex << "
      // pMap值(占据点)：  " << pMap[tmpLinearIndex] << std::endl;
      if (pMap[tmpLinearIndex] >= 100)
        pMap[tmpLinearIndex] = 100;
    } else {
      // std::cerr << "beamPointIndex if invalid!!!" << std::endl;
    }
  } //一束激光雷达数据中的数据点遍历
    // double world_x = (*cloud_data.cloud_ptr).points
}

/**
 * Increments all the grid cells from (x0, y0) to (x1, y1);
 * //不包含(x1,y1)
 * 2D画线算法　来进行计算两个点之间的grid cell
 * @param x0
 * @param y0
 * @param x1
 * @param y1 robotIndex.x,robotIndex.y,beamPointIndex.x,beamPointIndex.y
 */
std::vector<GridIndex> Mapping::TraceLine(int x0, int y0, int x1, int y1) {
  GridIndex tmpIndex;
  std::vector<GridIndex> gridIndexVector;

  bool steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) // k>1或k<-1
  {
    std::swap(x0, y0);
    std::swap(x1, y1);
  }
  if (x0 > x1) {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }

  int deltaX = x1 - x0;
  int deltaY = abs(y1 - y0);
  int error = 0;
  int ystep;
  int y = y0;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  int pointX;
  int pointY;
  for (int x = x0; x <= x1; x++) {
    if (steep) {
      pointX = y;
      pointY = x;
    } else {
      pointX = x;
      pointY = y;
    }

    error += deltaY;

    if (2 * error >= deltaX) {
      y += ystep;
      error -= deltaX;
    }

    //不包含最后一个点．
    if (pointX == x1 && pointY == y1)
      continue;

    //保存所有的点
    tmpIndex.SetIndex(pointX, pointY);

    gridIndexVector.push_back(tmpIndex);
  }

  return gridIndexVector;
}

//从世界坐标系转换到栅格坐标系
GridIndex Mapping::ConvertWorld2GridIndex(double x, double y) {
  GridIndex index;
  //地图的正中间为栅格坐标系的中心，（mapParams.offset_x，mapParams.offset_y）
  index.x = std::ceil((x - mapParams.origin_x) / mapParams.resolution) +
            mapParams.offset_x;
  index.y = std::ceil((y - mapParams.origin_y) / mapParams.resolution) +
            mapParams.offset_y;

  return index;
}

int Mapping::GridIndexToLinearIndex(GridIndex index) {
  int linear_index;
  linear_index = index.x + index.y * mapParams.width;
  return linear_index;
}

//判断index是否有效
bool Mapping::isValidGridIndex(GridIndex index) {
  if (index.x >= 0 && index.x < mapParams.width && index.y >= 0 &&
      index.y < mapParams.height)
    return true;

  return false;
}

void Mapping::DestoryMap() {
  if (pMap != NULL)
    delete pMap;
}

nav_msgs::OccupancyGrid Mapping::GetCurrentMap() {
  // std::cout << "开始建图，请稍后..." << std::endl;
  nav_msgs::OccupancyGrid Gridmap;

  Gridmap.info.resolution =
      mapParams.resolution; //这个通过yaml文件给mapParams.resolution置位
  Gridmap.info.origin.position.x = 0.0;
  Gridmap.info.origin.position.y = 0.0;
  Gridmap.info.origin.position.z = 0.0;
  Gridmap.info.origin.orientation.x = 0.0;
  Gridmap.info.origin.orientation.y = 0.0;
  Gridmap.info.origin.orientation.z = 0.0;
  Gridmap.info.origin.orientation.w = 1.0;

  Gridmap.info.origin.position.x =
      mapParams.origin_x; //这个通过yaml文件给mapParams.resolution置位
  Gridmap.info.origin.position.y =
      mapParams.origin_y; //这些参数应该在mapping类下搞定的
  Gridmap.info.width = mapParams.width;
  Gridmap.info.height = mapParams.height;
  Gridmap.data.resize(Gridmap.info.width * Gridmap.info.height);

  // 0~100
  // int cnt0, cnt1, cnt2;
  // cnt0 = cnt1 = cnt2 = 100;
  for (int i = 0; i < Gridmap.info.width * Gridmap.info.height; i++) {
    if (pMap[i] == 50) //？
    {
      Gridmap.data[i] = -1.0; // Unknown is -1.
    } else {
      Gridmap.data[i] =
          pMap[i]; // unsigned char *pMap; //指向unsigned char类型元素的指针
    }
  }

  return Gridmap;
}

//保存地图的服务
bool Mapping::SaveMap() {

  std::cout << "地图保存完成，地址是：" << std::endl;

  return true;
}

} // namespace mrobot_frame