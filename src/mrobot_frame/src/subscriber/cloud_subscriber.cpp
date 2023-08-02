
#include "mrobot_frame/subscriber/cloud_subscriber.hpp"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"

namespace mrobot_frame {

CloudSubscriber::CloudSubscriber(ros::NodeHandle &nh, std::string msg_type,
                                 std::string topic_name, size_t buff_size)
    : nh_(nh), scan_filter_sub_(NULL), scan_filter_(NULL) {

  nh.param<std::string>("odom_frame", odom_frame_, "odom");
  if (msg_type == "PointCloud2") {
    cloud_filter_sub_ =
        new message_filters::Subscriber<sensor_msgs::PointCloud2>(
            nh, topic_name, 5);
    cloud_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(
        *cloud_filter_sub_, tf_, odom_frame_, 5);
    cloud_filter_->registerCallback(
        boost::bind(&CloudSubscriber::msg_pointCloud2_callback, this, _1));
  } else if (msg_type == "Laserscan") {
    scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(
        nh, topic_name, 5); //用message_filters库来订阅"/base_scan"
    // tf::MessageFilter，订阅激光数据同时和odom_frame之间转换时间同步
    scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(
        *scan_filter_sub_, tf_, odom_frame_,
        5); //创建一个tf::MessageFilter对象，用于将激光数据转换到odom_frame_坐标系下。
    // scan_filter_注册回调函数laserCallback
    scan_filter_->registerCallback(
        boost::bind(&CloudSubscriber::msg_laserScan_callback, this, _1)); //
  }
}

CloudSubscriber::~CloudSubscriber() {
  if (scan_filter_)
    delete scan_filter_;
  if (scan_filter_sub_)
    delete scan_filter_sub_;
}

void CloudSubscriber::msg_laserScan_callback(
    const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
  buff_mutex_.lock();
  CloudData cloud_data;
  Transcloud::LaserscanToPcl(scan_msg, cloud_data);

  new_cloud_data_.push_back(cloud_data);
  buff_mutex_.unlock();
}

void CloudSubscriber::msg_pointCloud2_callback(
    const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr) {
  buff_mutex_.lock();
  CloudData cloud_data;
  // cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
  pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));

  new_cloud_data_.push_back(cloud_data);
  buff_mutex_.unlock();
}

void CloudSubscriber::ParseData(std::deque<CloudData> &cloud_data_buff) {
  buff_mutex_.lock();

  if (new_cloud_data_.size() > 0) {

    cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(),
                           new_cloud_data_.end());
    new_cloud_data_.clear();
  }

  buff_mutex_.unlock();
}
} // namespace mrobot_frame
