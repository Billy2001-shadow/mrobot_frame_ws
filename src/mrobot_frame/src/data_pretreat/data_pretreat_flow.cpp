#include "mrobot_frame/data_pretreat/data_pretreat_flow.hpp"
#include <pcl/common/transforms.h>

namespace mrobot_frame {
DataPretreatFlow::DataPretreatFlow(ros::NodeHandle &nh, std::string scan_topic,
                                   std::string cloud_topic) {

  std::string laser_frame, odom_frame, lidarMsg_type;
  nh.param<std::string>("laser_frame", laser_frame, "laser_link");
  nh.param<std::string>("odom_frame", odom_frame, "odom");
  //在数据预处理部分默认是2d即Laserscan
  nh.param<std::string>("lidarMsg_type", lidarMsg_type, "Laserscan");
  cloud_sub_ptr_ =
      std::make_shared<CloudSubscriber>(nh, lidarMsg_type, scan_topic, 100000);
  tf_pose_ptr_ = std::make_shared<TFListener>(nh, odom_frame, laser_frame);
  //后面这里可以加一个gnss的位姿展示(仅在3d下)

  // publisher
  cloud_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, cloud_topic, laser_frame, 100);
  odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "/odom_pose", odom_frame, laser_frame, 100);
}

bool DataPretreatFlow::Run() {
  if (!ReadData())
    return false;

  while (HasData()) {
    if (!ValidData())
      continue;
    TransformDataToMap();
    PublishData();
  }

  return true;
}

bool DataPretreatFlow::ReadData() {
  cloud_sub_ptr_->ParseData(cloud_data_buff_);

  if (cloud_data_buff_.size() == 0)
    return false;

  return true;
}

bool DataPretreatFlow::HasData() { return cloud_data_buff_.size() > 0; }

bool DataPretreatFlow::ValidData() {
  current_cloud_data_ = cloud_data_buff_.front();

  cloud_data_buff_.pop_front();

  return true;
}

bool DataPretreatFlow::TransformDataToMap() {
  tf_pose_ptr_->LookupData(tf_pose_);

  // pcl::transformPointCloud(*current_cloud_data_.cloud_ptr,
  //                          *current_cloud_data_.cloud_ptr,
  //                          tf_pose_); //将点云转换到odom坐标系下

  return true;
}

bool DataPretreatFlow::PublishData() {
  cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr,
                          current_cloud_data_.time);
  odom_pub_ptr_->Publish(tf_pose_, current_cloud_data_.time); //轮式里程计

  return true;
}
} // namespace mrobot_frame