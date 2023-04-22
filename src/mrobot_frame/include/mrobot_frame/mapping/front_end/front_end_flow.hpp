#ifndef MROBOT_FRAME_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_
#define MROBOT_FRAME_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_

#include <ros/ros.h>

#include "mrobot_frame/subscriber/cloud_subscriber2.hpp"
#include "mrobot_frame/publisher/odometry_publisher.hpp"
#include "mrobot_frame/mapping/front_end/front_end.hpp"
#include "mrobot_frame/subscriber/tf_listener.hpp"
namespace mrobot_frame {
class FrontEndFlow {
  public:
    FrontEndFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string odom_topic);

    bool Run();

  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool UpdateLaserOdometry();
    bool PublishData();

  private:
  
    std::shared_ptr<CloudSubscriber2> cloud_sub_ptr_;
    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
    std::shared_ptr<FrontEnd> front_end_ptr_;

    std::deque<CloudData> cloud_data_buff_;

    CloudData current_cloud_data_;

    Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f tf_pose_ = Eigen::Matrix4f::Identity();
    std::shared_ptr<TFListener> tf_pose_ptr_;
};
}

#endif