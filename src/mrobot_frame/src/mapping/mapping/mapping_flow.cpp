#include "mrobot_frame/mapping/mapping/mapping_flow.hpp"
#include "glog/logging.h"
#include "mrobot_frame/global_defination/global_defination.h"

namespace mrobot_frame {
MappingFlow::MappingFlow(ros::NodeHandle &nh, std::string cloud_topic) {

  // //只根据历史关键帧建图
  key_frame_sub_ptr_ = std::make_shared<KeyFrameSubscriber>(
      nh, "/key_frame", 100000); //订阅最新的关键帧

  occupancygrid_pub_ptr_ =
      std::make_shared<GridmapPublisher>(nh, "occupancygrid", "odom", 1);

  mapping_ptr_ = std::make_shared<Mapping>(); //调用核心功能类
}

bool MappingFlow::Run() {
  if (!ReadData())
    return false;

  while (HasData()) {
    if (ValidData()) {
      mapping_ptr_->OccupanyMapping(current_keyframe);
      PublishData(); //每过一个关键帧更新一次地图
    }
  }

  return true;
}

bool MappingFlow::ReadData() {

  key_frame_sub_ptr_->ParseData(key_frame_buff_);

  return true;
}

bool MappingFlow::HasData() {
  if (key_frame_buff_.size() == 0)
    return false;

  return true;
}

bool MappingFlow::ValidData() {
  current_keyframe = key_frame_buff_.front();
  key_frame_buff_.pop_front();

  return true;
}

bool MappingFlow::PublishData() {
  occupancygrid_pub_ptr_->Publish(mapping_ptr_->GetCurrentMap());
  return true;
}

bool MappingFlow::SaveGridmap() { return true; }
} // namespace mrobot_frame