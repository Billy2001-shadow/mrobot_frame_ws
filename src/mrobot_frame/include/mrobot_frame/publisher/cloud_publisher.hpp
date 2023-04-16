#ifndef MROBOT_FRAME_CLOUD_PUBLISHER_HPP_
#define MROBOT_FRAME_CLOUD_PUBLISHER_HPP_

#include "mrobot_frame/sensor_data/cloud_data.hpp"
#include "mrobot_frame/data_pretreat/trans_data_methods/transcloud.hpp"

#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <string>

namespace mrobot_frame {
class CloudPublisher {
  public:
    CloudPublisher(ros::NodeHandle& nh,
                   std::string topic_name,
                   std::string frame_id,
                   size_t buff_size);
    CloudPublisher() = default;

    void Publish(CloudData::CLOUD_PTR& cloud_ptr_input, double time);
    void Publish(CloudData::CLOUD_PTR& cloud_ptr_input);

    bool HasSubscribers();


  private:
    void PublishData(CloudData::CLOUD_PTR& cloud_ptr_input, ros::Time time);

    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};
}


#endif