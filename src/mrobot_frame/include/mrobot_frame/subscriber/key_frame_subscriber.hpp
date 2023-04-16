#ifndef MROBOT_FRAME_SUBSCRIBER_KEY_FRAME_SUBSCRIBER_HPP_
#define MROBOT_FRAME_SUBSCRIBER_KEY_FRAME_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "mrobot_frame/sensor_data/key_frame.hpp"

namespace mrobot_frame {
class KeyFrameSubscriber {
  public:
    KeyFrameSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    KeyFrameSubscriber() = default;
    void ParseData(std::deque<KeyFrame>& key_frame_buff);

  private:
    void msg_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& key_frame_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<KeyFrame> new_key_frame_; 
};
}
#endif