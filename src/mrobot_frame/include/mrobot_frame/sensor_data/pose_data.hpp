#ifndef MROBOT_FRAME_SENSOR_DATA_POSE_DATA_HPP_
#define MROBOT_FRAME_SENSOR_DATA_POSE_DATA_HPP_

#include <Eigen/Dense>

namespace mrobot_frame {
class PoseData {
  public:
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    double time = 0.0;

  public:
    Eigen::Quaternionf GetQuaternion();
};
}

#endif