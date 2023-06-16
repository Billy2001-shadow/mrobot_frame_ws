#include "mrobot_frame/mapping/viewer/viewer_flow.hpp"
#include "glog/logging.h"
#include "mrobot_frame/global_defination/global_defination.h"

namespace mrobot_frame {
ViewerFlow::ViewerFlow(ros::NodeHandle& nh, std::string cloud_topic) {
    // subscriber
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber2>(nh, cloud_topic, 100000);
    key_frame_sub_ptr_ = std::make_shared<KeyFrameSubscriber>(nh, "/key_frame", 100000);
    transformed_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/transformed_odom", 100000);
    optimized_key_frames_sub_ptr_ = std::make_shared<KeyFramesSubscriber>(nh, "/optimized_key_frames", 100000);
    // publisher
    optimized_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/optimized_odom", "map", "/lidar", 100);
    current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/current_scan", "map", 100);
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/global_map", "map", 100);
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/local_map", "map", 100);
    // viewer
    viewer_ptr_ = std::make_shared<Viewer>();
}

bool ViewerFlow::Run() {
    if (!ReadData())
        return false;

    while(HasData()) {
        if (ValidData()) {
            viewer_ptr_->UpdateWithNewKeyFrame(key_frame_buff_, current_transformed_odom_, current_cloud_data_);
            PublishLocalData();
        }
    }

    //数据全部接受完毕才进行这一步，才读最后的优化结果
    if (optimized_key_frames_.size() > 0) {
        ROS_INFO_STREAM("\033[1;32m----> Before Map <----\033[0m");
        viewer_ptr_->UpdateWithOptimizedKeyFrames(optimized_key_frames_);
        PublishGlobalData();
    }

    return true;
}

bool ViewerFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    transformed_odom_sub_ptr_->ParseData(transformed_odom_buff_);
    key_frame_sub_ptr_->ParseData(key_frame_buff_);
    optimized_key_frames_sub_ptr_->ParseData(optimized_key_frames_);

    return true;
}

bool ViewerFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (transformed_odom_buff_.size() == 0)
        return false;

    return true;
}

bool ViewerFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_transformed_odom_ = transformed_odom_buff_.front();

    double diff_odom_time = current_cloud_data_.time - current_transformed_odom_.time;

    if (diff_odom_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_odom_time > 0.05) {
        transformed_odom_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    transformed_odom_buff_.pop_front();

    return true;
}

bool ViewerFlow::PublishGlobalData() {
    if (viewer_ptr_->HasNewGlobalMap() && global_map_pub_ptr_->HasSubscribers()) {
        CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
        viewer_ptr_->GetGlobalMap(cloud_ptr);
        global_map_pub_ptr_->Publish(cloud_ptr);
    }

    return true;
}

bool ViewerFlow::PublishLocalData() {
    optimized_odom_pub_ptr_->Publish(viewer_ptr_->GetCurrentPose()); //优化后的位姿
    current_scan_pub_ptr_->Publish(viewer_ptr_->GetCurrentScan());

    if (viewer_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers()) {
        CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
        viewer_ptr_->GetLocalMap(cloud_ptr);
        local_map_pub_ptr_->Publish(cloud_ptr);
    }

    return true;
}

bool ViewerFlow::SaveMap() {
    return viewer_ptr_->SaveMap();
}
}