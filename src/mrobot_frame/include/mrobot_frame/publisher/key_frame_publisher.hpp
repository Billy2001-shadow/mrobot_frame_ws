#ifndef MROBOT_FRAME_PUBLISHER_KEY_FRAME_PUBLISHER_HPP_
#define MROBOT_FRAME_PUBLISHER_KEY_FRAME_PUBLISHER_HPP_

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "mrobot_frame/sensor_data/key_frame.hpp"

namespace mrobot_frame {
class KeyFramePublisher {
  public:
    KeyFramePublisher(ros::NodeHandle& nh, 
                      std::string topic_name, 
                      std::string frame_id,
                      int buff_size);
    KeyFramePublisher() = default;

    void Publish(KeyFrame& key_frame);

    bool HasSubscribers();

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_ = "";
};
}
#endif