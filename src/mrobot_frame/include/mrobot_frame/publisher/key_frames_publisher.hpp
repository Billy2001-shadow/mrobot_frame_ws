#ifndef MROBOT_FRAME_PUBLISHER_KEY_FRAMES_PUBLISHER_HPP_
#define MROBOT_FRAME_PUBLISHER_KEY_FRAMES_PUBLISHER_HPP_

#include <string>
#include <deque>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "mrobot_frame/sensor_data/key_frame.hpp"

namespace mrobot_frame {
class KeyFramesPublisher {
  public:
    KeyFramesPublisher(ros::NodeHandle& nh, 
                      std::string topic_name, 
                      std::string frame_id,
                      int buff_size);
    KeyFramesPublisher() = default;

    void Publish(const std::deque<KeyFrame>& key_frames);

    bool HasSubscribers();

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_ = "";
};
}
#endif