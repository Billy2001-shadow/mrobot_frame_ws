#include "mrobot_frame/mapping/front_end/front_end_flow.hpp"
// #include "glog/logging.h"


namespace mrobot_frame {
FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string odom_topic) {
    //订阅点云信息
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber2>(nh, cloud_topic, 100000);
    //发布位姿信息
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, odom_topic, "/map", "/lidar", 100);

    front_end_ptr_ = std::make_shared<FrontEnd>();
}

bool FrontEndFlow::Run() {
    if (!ReadData())
        return false;

    while(HasData()) {
        if (!ValidData())
            continue;
        
        if (UpdateLaserOdometry()) {
            PublishData();
        }
    }

    return true;
}

bool FrontEndFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    return true;
}

bool FrontEndFlow::HasData() {
    return cloud_data_buff_.size() > 0;
}

bool FrontEndFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    cloud_data_buff_.pop_front();

    return true;
}

//更新位姿
bool FrontEndFlow::UpdateLaserOdometry() {
    static bool odometry_inited = false;
    if (!odometry_inited) {
        odometry_inited = true;
        odometry_inited = true;

        Eigen::Matrix4f odom_pose_init = Eigen::Matrix4f::Identity();
        //front_end_ptr_->SetInitPose(Eigen::Matrix4f::Identity()); //初始位姿

        odom_pose_init(0,3) = (float)2.0474998951;
        odom_pose_init(1,3) = (float)12.6241998672;
        odom_pose_init(2,3) = (float)0.0;
        Eigen::Quaterniond quaternion4(-0.258719463674, 0.0, 0.0, 0.965952503551);
        quaternion4.normalize();
        Eigen::Matrix3d rotation_matrix3d = quaternion4.matrix();
        Eigen::Matrix3f rotation_matrix3f = rotation_matrix3d.cast<float>();
        odom_pose_init.block<3,3>(0,0) = rotation_matrix3f;
        front_end_ptr_->SetInitPose(odom_pose_init); //初始位姿
      
        return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
    }

    return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
}

//
bool FrontEndFlow::PublishData() {
    laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time);

    return true;
}

}

