#ifndef MROBOT_FRAME_MODELS_REGISTRATION_INTERFACE_HPP_
#define MROBOT_FRAME_MODELS_REGISTRATION_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "mrobot_frame/sensor_data/cloud_data.hpp"

namespace mrobot_frame {
class RegistrationInterface {
  public:
    virtual ~RegistrationInterface() = default;

    //添加target
    virtual bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) = 0;
    //添加source，提供predict_pose，得到全局的位姿，非相对位姿
    virtual bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                          const Eigen::Matrix4f& predict_pose, 
                          CloudData::CLOUD_PTR& result_cloud_ptr,
                          Eigen::Matrix4f& result_pose) = 0;
    //计算source和target之间的误差
    virtual float GetFitnessScore() = 0;
};
} 

#endif