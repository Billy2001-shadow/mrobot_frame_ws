#ifndef MROBOT_FRAME_MAPPING_MAPPING_FLOW_HPP_
#define MROBOT_FRAME_MAPPING_MAPPING_FLOW_HPP_

#include <deque>
#include <ros/ros.h>
// subscriber
#include "mrobot_frame/subscriber/cloud_subscriber.hpp"
#include "mrobot_frame/subscriber/cloud_subscriber2.hpp"
#include "mrobot_frame/subscriber/odometry_subscriber.hpp"
#include "mrobot_frame/subscriber/key_frame_subscriber.hpp"
#include "mrobot_frame/subscriber/key_frames_subscriber.hpp"
// publisher
#include "mrobot_frame/publisher/odometry_publisher.hpp"
#include "mrobot_frame/publisher/cloud_publisher.hpp"
#include "mrobot_frame/publisher/gridmap_publisher.hpp"
// 
#include "mrobot_frame/mapping/mapping/mapping.hpp"

namespace mrobot_frame {
class MappingFlow {
  public:
    MappingFlow(ros::NodeHandle& nh, std::string cloud_topic);
    bool Run();
    bool SaveGridmap(); //建立珊格地图(后面可以和viewer中保存点云地图的函数名区分开来) 供node文件调用

  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool PublishData();

  private:

  CloudData current_cloud_data_;
  PoseData current_transformed_odom_;
  KeyFrame current_keyframe;

  std::deque<CloudData> cloud_data_buff_;
  std::deque<PoseData> transformed_odom_buff_;
  std::deque<KeyFrame> key_frame_buff_;
  // subscriber
  std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
  //publisher
  std::shared_ptr<GridmapPublisher>occupancygrid_pub_ptr_;

  std::shared_ptr<Mapping> mapping_ptr_; //调用核心类

    
};
}

#endif