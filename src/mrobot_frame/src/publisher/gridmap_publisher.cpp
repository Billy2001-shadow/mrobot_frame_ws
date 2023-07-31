#include "mrobot_frame/publisher/gridmap_publisher.hpp"

namespace mrobot_frame {
GridmapPublisher::GridmapPublisher(ros::NodeHandle &nh, std::string topic_name,
                                   std::string frame_id, size_t buff_size)
    : nh_(nh), frame_id_(frame_id) { // frame_id一般为map坐标系
  publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>(
      topic_name, buff_size); //发布占据栅格地图
}

void GridmapPublisher::Publish(const nav_msgs::OccupancyGrid &rosMap) {
  nav_msgs::OccupancyGrid Gridmap;
  map_mutex_.lock();
  Gridmap = rosMap;
  map_mutex_.unlock();
  Gridmap.header.stamp = ros::Time::now();
  Gridmap.header.frame_id = frame_id_;
  // std::cout << "建图完毕..." << std::endl;
  publisher_.publish(Gridmap);
}

} // namespace mrobot_frame