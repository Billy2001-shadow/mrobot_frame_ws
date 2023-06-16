#include "mrobot_frame/mapping/loop_closing/loop_closing_flow.hpp"
#include "glog/logging.h"

namespace mrobot_frame {
LoopClosingFlow::LoopClosingFlow(ros::NodeHandle& nh) {
    // subscriber
    key_frame_sub_ptr_ = std::make_shared<KeyFrameSubscriber>(nh, "/key_frame", 100000);
    // publisher
    loop_pose_pub_ptr_ = std::make_shared<LoopPosePublisher>(nh, "/loop_pose", "map", 100);
    // loop closing
    loop_closing_ptr_ = std::make_shared<LoopClosing>();
}

bool LoopClosingFlow::Run() {
    if (!ReadData())
        return false;

    while(HasData()) {
        if (!ValidData())
            continue;
        
        loop_closing_ptr_->Update(current_key_frame_);
        
        PublishData();
    }

    return true;
}

bool LoopClosingFlow::ReadData() {
    key_frame_sub_ptr_->ParseData(key_frame_buff_);

    return true;
}

bool LoopClosingFlow::HasData() {
    if (key_frame_buff_.size() == 0)
        return false;

    return true;
}

bool LoopClosingFlow::ValidData() {
    current_key_frame_ = key_frame_buff_.front();

    key_frame_buff_.pop_front();

    return true;
}

bool LoopClosingFlow::PublishData() {
    if (loop_closing_ptr_->HasNewLoopPose()) 
        loop_pose_pub_ptr_->Publish(loop_closing_ptr_->GetCurrentLoopPose());

    return true;
}
}