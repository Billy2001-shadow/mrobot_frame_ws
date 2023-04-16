#ifndef MROBOT_FRAME_CLOUD_SUBSCRIBER_HPP_
#define MROBOT_FRAME_CLOUD_SUBSCRIBER_HPP_

#include "mrobot_frame/sensor_data/cloud_data.hpp"
#include "mrobot_frame/data_pretreat/trans_data_methods/transcloud.hpp"

#include <deque>
#include <mutex>
#include <thread>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace mrobot_frame {
class CloudSubscriber {
  public:
    CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    CloudSubscriber() = default;
    void ParseData(std::deque<CloudData>& cloud_data_buff);

  private:
    void msg_callback(const sensor_msgs::LaserScan::ConstPtr& cloud_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<CloudData> new_cloud_data_;

    std::mutex buff_mutex_;
};
}


#endif