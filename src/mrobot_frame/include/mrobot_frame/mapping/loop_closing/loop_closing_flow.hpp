#ifndef MROBOT_FRAME_MAPPING_LOOP_CLOSING_LOOP_CLOSING_FLOW_HPP_
#define MROBOT_FRAME_MAPPING_LOOP_CLOSING_LOOP_CLOSING_FLOW_HPP_

#include <deque>
#include <ros/ros.h>
// subscriber
#include "mrobot_frame/subscriber/key_frame_subscriber.hpp"
// publisher
#include "mrobot_frame/publisher/loop_pose_publisher.hpp"
// loop closing
#include "mrobot_frame/mapping/loop_closing/loop_closing.hpp"

namespace mrobot_frame {
class LoopClosingFlow {
  public:
    LoopClosingFlow(ros::NodeHandle& nh);

    bool Run();

  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool PublishData();

  private:
    // subscriber
    std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
    // publisher
    std::shared_ptr<LoopPosePublisher> loop_pose_pub_ptr_;
    // loop closing
    std::shared_ptr<LoopClosing> loop_closing_ptr_;

    std::deque<KeyFrame> key_frame_buff_;

    KeyFrame current_key_frame_;
};
}

#endif