#ifndef MROBOT_FRAME_PUBLISHER_LOOP_POSE_PUBLISHER_HPP_
#define MROBOT_FRAME_PUBLISHER_LOOP_POSE_PUBLISHER_HPP_

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "mrobot_frame/sensor_data/loop_pose.hpp"

namespace mrobot_frame {
class LoopPosePublisher {
  public:
    LoopPosePublisher(ros::NodeHandle& nh, 
                      std::string topic_name, 
                      std::string frame_id,
                      int buff_size);
    LoopPosePublisher() = default;

    void Publish(LoopPose& loop_pose);

    bool HasSubscribers();

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_ = "";
};
}
#endif