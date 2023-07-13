
#include "mrobot_frame/subscriber/cloud_subscriber.hpp"

namespace mrobot_frame {

CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh),
    scan_filter_sub_(NULL),
    scan_filter_(NULL)
{
    //subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);

    scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, topic_name, 5);//用message_filters库来订阅scan_topic_
    //tf::MessageFilter，订阅激光数据同时和odom_frame之间转换时间同步
    scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5); //创建一个tf::MessageFilter对象，用于将激光数据转换到odom_frame_坐标系下。
    //scan_filter_注册回调函数laserCallback
    scan_filter_->registerCallback(boost::bind(&CloudSubscriber::msg_callback, this, _1)); //

}

CloudSubscriber::~CloudSubscriber()
{
    if(scan_filter_)
        delete scan_filter_;
    if(scan_filter_sub_)
        delete scan_filter_sub_;
}

void CloudSubscriber::msg_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    buff_mutex_.lock();
    CloudData cloud_data;
    Transcloud::LaserscanToPcl(scan_msg, cloud_data);

    new_cloud_data_.push_back(cloud_data);
    buff_mutex_.unlock();
}


void CloudSubscriber::ParseData(std::deque<CloudData>& cloud_data_buff) {
    buff_mutex_.lock();

    if (new_cloud_data_.size() > 0) {
        
        cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
        new_cloud_data_.clear();
    }

    buff_mutex_.unlock();

}
}


