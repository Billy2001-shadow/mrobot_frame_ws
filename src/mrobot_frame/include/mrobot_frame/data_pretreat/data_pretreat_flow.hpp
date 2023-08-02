#ifndef MROBOT_FRAME_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_
#define MROBOT_FRAME_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_

#include "mrobot_frame/publisher/cloud_publisher.hpp"
#include "mrobot_frame/publisher/odometry_publisher.hpp"
#include "mrobot_frame/sensor_data/cloud_data.hpp"
#include "mrobot_frame/subscriber/cloud_subscriber.hpp"
#include "mrobot_frame/subscriber/cloud_subscriber2.hpp"
#include "mrobot_frame/subscriber/tf_listener.hpp"

#include <memory>
#include <ros/ros.h>

namespace mrobot_frame {
class DataPretreatFlow {
public:
  DataPretreatFlow(ros::NodeHandle &nh, std::string scan_topic,
                   std::string cloud_topic);
  bool Run();

private:
  bool ReadData();
  bool HasData();
  bool ValidData();
  bool TransformDataToMap();
  bool PublishData();

private:
  // subscriber
  std::shared_ptr<CloudSubscriber2> cloud_sub_ptr_2;
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<TFListener> tf_pose_ptr_;

  // publisher
  std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
  std::shared_ptr<OdometryPublisher> odom_pub_ptr_;

  std::deque<CloudData> cloud_data_buff_;

  CloudData current_cloud_data_;

  Eigen::Matrix4f tf_pose_ = Eigen::Matrix4f::Identity();
};
} // namespace mrobot_frame

#endif