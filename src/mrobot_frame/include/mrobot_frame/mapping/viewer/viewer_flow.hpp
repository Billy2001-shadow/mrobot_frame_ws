#ifndef MROBOT_FRAME_MAPPING_VIEWER_VIEWER_FLOW_HPP_
#define MROBOT_FRAME_MAPPING_VIEWER_VIEWER_FLOW_HPP_

#include <deque>
#include <ros/ros.h>
// subscriber
#include "mrobot_frame/subscriber/cloud_subscriber2.hpp"
#include "mrobot_frame/subscriber/odometry_subscriber.hpp"
#include "mrobot_frame/subscriber/key_frame_subscriber.hpp"
#include "mrobot_frame/subscriber/key_frames_subscriber.hpp"
// publisher
#include "mrobot_frame/publisher/odometry_publisher.hpp"
#include "mrobot_frame/publisher/cloud_publisher.hpp"
// viewer
#include "mrobot_frame/mapping/viewer/viewer.hpp"

namespace mrobot_frame {
class ViewerFlow {
  public:
    ViewerFlow(ros::NodeHandle& nh, std::string cloud_topic);

    bool Run();
    bool SaveMap();

  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool PublishGlobalData();
    bool PublishLocalData();

  private:
    // subscriber
    std::shared_ptr<CloudSubscriber2> cloud_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> transformed_odom_sub_ptr_;
    std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
    std::shared_ptr<KeyFramesSubscriber> optimized_key_frames_sub_ptr_;
    // publisher
    std::shared_ptr<OdometryPublisher> optimized_odom_pub_ptr_;
    std::shared_ptr<CloudPublisher> current_scan_pub_ptr_;
    std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
    std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
    // viewer
    std::shared_ptr<Viewer> viewer_ptr_;

    std::deque<CloudData> cloud_data_buff_;
    std::deque<PoseData> transformed_odom_buff_;
    std::deque<KeyFrame> key_frame_buff_;
    std::deque<KeyFrame> optimized_key_frames_;
    std::deque<KeyFrame> all_key_frames_;

    CloudData current_cloud_data_;
    PoseData current_transformed_odom_;
};
}

#endif