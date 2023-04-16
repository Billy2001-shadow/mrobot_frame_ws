#ifndef MROBOT_FRAME_MAPPING_FRONT_END_FRONT_END_HPP_
#define MROBOT_FRAME_MAPPING_FRONT_END_FRONT_END_HPP_

#include <deque>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "mrobot_frame/sensor_data/cloud_data.hpp"
#include "mrobot_frame/models/registration/registration_interface.hpp"
#include "mrobot_frame/models/cloud_filter/cloud_filter_interface.hpp"

namespace mrobot_frame {
class FrontEnd {
  public:
    struct Frame { 
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        CloudData cloud_data;
    };

  public:
    FrontEnd();
    
    //得到一帧点云，就返回一个位姿
    bool Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose);
    bool SetInitPose(const Eigen::Matrix4f& init_pose);

  private:
    bool InitWithConfig();
    bool InitParam(const YAML::Node& config_node);
    bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
    bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);
    bool UpdateWithNewFrame(const Frame& new_key_frame);

  private:
    std::string data_path_ = "";

    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
    std::shared_ptr<RegistrationInterface> registration_ptr_; 

    std::deque<Frame> local_map_frames_;

    CloudData::CLOUD_PTR local_map_ptr_;
    Frame current_frame_;
    bool first_scan_;

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();

    float key_frame_distance_ = 2.0;
    int local_frame_num_ = 20;
};
}

#endif