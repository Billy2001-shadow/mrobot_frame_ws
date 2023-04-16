#include "mrobot_frame/mapping/back_end/back_end_flow.hpp"

#include "glog/logging.h"

#include "mrobot_frame/tools/file_manager.hpp"


namespace mrobot_frame {
BackEndFlow::BackEndFlow(ros::NodeHandle& nh) {
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber2>(nh, "/pretreat_cloud", 100000); //
    laser_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/laser_odom_pose", 100000); //激光里程计位姿(构建节点和相邻边)
    loop_pose_sub_ptr_ = std::make_shared<LoopPoseSubscriber>(nh, "/loop_pose", 100000);//回环位姿(构建回环边)
    //发布
    transformed_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/transformed_odom", "/map", "/lidar", 100);//优化后的位姿？
    key_frame_pub_ptr_ = std::make_shared<KeyFramePublisher>(nh, "/key_frame", "/map", 100);//关键帧(最新的)
    key_frames_pub_ptr_ = std::make_shared<KeyFramesPublisher>(nh, "/optimized_key_frames", "/map", 100);//历史关键帧

    back_end_ptr_ = std::make_shared<BackEnd>();
}

bool BackEndFlow::Run() {
    if (!ReadData())
        return false;

    MaybeInsertLoopPose();  //插入回环帧

    while(HasData()) {
        if (!ValidData())
            continue;

        UpdateBackEnd();

        PublishData();
    }

    return true;
}

bool BackEndFlow::ForceOptimize() {
    back_end_ptr_->ForceOptimize();
    if (back_end_ptr_->HasNewOptimized()) {
        std::deque<KeyFrame> optimized_key_frames;
        back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames);
        key_frames_pub_ptr_->Publish(optimized_key_frames);
    }
    return true;
}

bool BackEndFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    laser_odom_sub_ptr_->ParseData(laser_odom_data_buff_);
    loop_pose_sub_ptr_->ParseData(loop_pose_data_buff_);

    return true;
}

bool BackEndFlow::MaybeInsertLoopPose() {
    while (loop_pose_data_buff_.size() > 0) {
        back_end_ptr_->InsertLoopPose(loop_pose_data_buff_.front());
        loop_pose_data_buff_.pop_front();
    }
    return true;
}

bool BackEndFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    
    if (laser_odom_data_buff_.size() == 0)
        return false;

    return true;
}

bool BackEndFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_laser_odom_data_ = laser_odom_data_buff_.front();

    double diff_laser_time = current_cloud_data_.time - current_laser_odom_data_.time;

    if (diff_laser_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_laser_time > 0.05) {
        laser_odom_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    laser_odom_data_buff_.pop_front();

    return true;
}

bool BackEndFlow::UpdateBackEnd() {
    //可根据需要在此调整点云坐标系
    return back_end_ptr_->Update(current_cloud_data_, current_laser_odom_data_);//根据当前点云数据和当前激光里程计位姿更新
}

bool BackEndFlow::PublishData() {
    transformed_odom_pub_ptr_->Publish(current_laser_odom_data_.pose, current_laser_odom_data_.time); //感觉current_laser_odom_data_没有变化

    if (back_end_ptr_->HasNewKeyFrame()) {
        KeyFrame key_frame;
        back_end_ptr_->GetLatestKeyFrame(key_frame);
        key_frame_pub_ptr_->Publish(key_frame);
    }

    if (back_end_ptr_->HasNewOptimized()) {
        std::deque<KeyFrame> optimized_key_frames;
        back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames);
        key_frames_pub_ptr_->Publish(optimized_key_frames);
    }

    return true;
}
}