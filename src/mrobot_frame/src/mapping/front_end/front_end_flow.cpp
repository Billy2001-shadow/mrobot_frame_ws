#include "mrobot_frame/mapping/front_end/front_end_flow.hpp"
// #include "glog/logging.h"

namespace mrobot_frame {
FrontEndFlow::FrontEndFlow(ros::NodeHandle &nh, std::string cloud_topic,
                           std::string laser_odom_topic) {

  std::string laser_frame, odom_frame;
  nh.param<std::string>("laser_frame", laser_frame, "laser_link");
  nh.param<std::string>("odom_frame", odom_frame, "odom");
  //订阅点云信息
  cloud_sub_ptr_ = std::make_shared<CloudSubscriber2>(
      nh, cloud_topic, 100000); //已经转换到odom坐标系下的点云数据

  laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, laser_odom_topic, odom_frame, laser_frame, 100);

  front_end_ptr_ = std::make_shared<FrontEnd>();
}

bool FrontEndFlow::Run() {
  if (!ReadData())
    return false;

  while (HasData()) {
    if (!ValidData())
      continue;

    if (UpdateLaserOdometry()) {
      PublishData();
    }
  }

  return true;
}

bool FrontEndFlow::ReadData() {
  cloud_sub_ptr_->ParseData(cloud_data_buff_);
  return true;
}

bool FrontEndFlow::HasData() { return cloud_data_buff_.size() > 0; }

bool FrontEndFlow::ValidData() {
  current_cloud_data_ = cloud_data_buff_.front();
  cloud_data_buff_.pop_front();

  return true;
}

//更新位姿
/*

*/
bool FrontEndFlow::UpdateLaserOdometry() {
  //根据当前点云数据更新激光里程计计算得到的位姿
  return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
}

//
bool FrontEndFlow::PublishData() {
  laser_odom_pub_ptr_->Publish(laser_odometry_,
                               current_cloud_data_.time); //激光里程计
  return true;
}

} // namespace mrobot_frame
