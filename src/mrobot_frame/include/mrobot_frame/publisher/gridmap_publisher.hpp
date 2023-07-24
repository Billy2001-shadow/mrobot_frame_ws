#ifndef MROBOT_FRAME_GRIDMAP_PUBLISHER_HPP_
#define MROBOT_FRAME_GRIDMAP_PUBLISHER_HPP_

#include <ros/ros.h>
#include <string>
#include "nav_msgs/GetMap.h"

namespace mrobot_frame {
class GridmapPublisher {
  public:
    GridmapPublisher(ros::NodeHandle& nh,
                   std::string topic_name,
                   std::string frame_id,
                   size_t buff_size);
    GridmapPublisher() = default;

    void Publish(const nav_msgs::OccupancyGrid& rosMap);

    bool HasSubscribers();


  private:

    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_ = "map";
};
}


#endif