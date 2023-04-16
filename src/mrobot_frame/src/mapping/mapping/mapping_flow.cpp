#include "mrobot_frame/mapping/mapping/mapping_flow.hpp"
#include "glog/logging.h"
#include "mrobot_frame/global_defination/global_defination.h"

namespace mrobot_frame {
MappingFlow::MappingFlow(ros::NodeHandle& nh, std::string cloud_topic) {
    //订阅
    //cloud_sub_ptr_ = std::make_shared<CloudSubscriber2>(nh, cloud_topic, 100000);
    //cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "laser_scan", 100000);
    optimized_key_frames_sub_ptr_ = std::make_shared<KeyFramesSubscriber>(nh, "/optimized_key_frames", 100000); //只根据历史关键帧建图
    //transformed_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/transformed_odom", 100000);
    //transformed_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/odom", 100000); 
    //发布
    occupancygrid_pub_ptr_ = std::make_shared<GridmapPublisher>(nh,"occupancygrid","map",100);

    // 
    mapping_ptr_ = std::make_shared<Mapping>(); //调用核心功能类
}

bool MappingFlow::Run() {
    if(!ReadData())
        return false;

    while(HasData()){
        if(ValidData()){
            mapping_ptr_->OccupanyMapping(current_keyframe);
            PublishData(); //每过一个关键帧更新一次地图
        }
    }

    
    return true;
}

bool MappingFlow::ReadData() {
    // cloud_sub_ptr_->ParseData(cloud_data_buff_);
    // transformed_odom_sub_ptr_->ParseData(transformed_odom_buff_);
    optimized_key_frames_sub_ptr_->ParseData(optimized_key_frames_buff_);

    return true;
}

bool MappingFlow::HasData() {
    if (optimized_key_frames_buff_.size() == 0)
        return false;
    // if (cloud_data_buff_.size() == 0)
    //     return false;
    // if (transformed_odom_buff_.size() == 0)
    //     return false;

    return true;
}

bool MappingFlow::ValidData() {
    current_keyframe = optimized_key_frames_buff_.front();
    optimized_key_frames_buff_.pop_front();
    // current_cloud_data_ = cloud_data_buff_.front();
    // current_transformed_odom_ = transformed_odom_buff_.front();

    // double diff_odom_time = current_cloud_data_.time - current_transformed_odom_.time;

    // if (diff_odom_time < -0.05) {
    //     cloud_data_buff_.pop_front();
    //     return false;
    // }

    // if (diff_odom_time > 0.05) {
    //     transformed_odom_buff_.pop_front();
    //     return false;
    // }

    // cloud_data_buff_.pop_front();
    // transformed_odom_buff_.pop_front();

    return true;
}

bool MappingFlow::PublishData(){
    occupancygrid_pub_ptr_->Publish(mapping_ptr_->GetCurrentMap());
    return true;
}

bool MappingFlow::SaveGridmap() {

    return true;
}
}