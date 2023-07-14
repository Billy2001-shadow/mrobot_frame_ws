#include "mrobot_frame/data_pretreat/data_pretreat_flow.hpp"
#include <pcl/common/transforms.h>


namespace mrobot_frame {
DataPretreatFlow::DataPretreatFlow(ros::NodeHandle& nh, std::string cloud_topic) {

    //ACES.bag
    // subscriber
    // cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/scan", 100000);
    // tf_pose_ptr_ = std::make_shared<TFListener>(nh, "/odom", "/base_link");
    
    // // // publisher
    // cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, cloud_topic, "odom",100);
    // odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/odom_pose", "odom", "/base_link", 100);  //轮式里程计？

    //basic_localization_stage_indexed.bag
    // subscriber
    // cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/base_scan", 100000);
    // tf_pose_ptr_ = std::make_shared<TFListener>(nh, "/odom", "/base_link");
    
    // // publisher
    // cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, cloud_topic, "map",100);
    // odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/odom_pose", "map", "/base_link", 100);

    // // subscriber
    // cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/scan", 100000);
    // tf_pose_ptr_ = std::make_shared<TFListener>(nh, "/odom", "/base_link");
    
    // // publisher
    // cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, cloud_topic, "map",100);
    // odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/odom_pose", "map", "/base_link", 100);
    
    //subscriber
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/base_scan", 100000);
    tf_pose_ptr_ = std::make_shared<TFListener>(nh, "/odom", "/base_footprint");
    
    
    // publisher
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, cloud_topic, "odom",100);  //用于将激光数据转换到odom_frame_坐标系下。
    odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/odom_pose", "odom", "/base_footprint", 100);
}

bool DataPretreatFlow::Run() {
    if (!ReadData())
        return false;


    while(HasData()) {
        if (!ValidData())
            continue;
        TransformDataToMap();
        PublishData();
    }

    return true;
}

bool DataPretreatFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);


    if (cloud_data_buff_.size() == 0)
        return false;


    return true;
}


bool DataPretreatFlow::HasData() {
    return cloud_data_buff_.size() > 0;
}

bool DataPretreatFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();

    cloud_data_buff_.pop_front();

    

    return true;
}

bool DataPretreatFlow::TransformDataToMap() {
    tf_pose_ptr_->LookupData(tf_pose_);

    //pcl::transformPointCloud(*current_cloud_data_.cloud_ptr, *current_cloud_data_.cloud_ptr, tf_pose_);  //将点云转换到odom坐标系下

    return true;
}

bool DataPretreatFlow::PublishData() {
    cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr, current_cloud_data_.time);
    odom_pub_ptr_->Publish(tf_pose_, current_cloud_data_.time); //轮式里程计

    return true;
}
}