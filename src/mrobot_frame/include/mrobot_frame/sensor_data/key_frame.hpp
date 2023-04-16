#ifndef MROBOT_FRAME_SENSOR_DATA_KEY_FRAME_HPP_
#define MROBOT_FRAME_SENSOR_DATA_KEY_FRAME_HPP_

#include <Eigen/Dense>

namespace mrobot_frame {
class KeyFrame {
  public:
    double time = 0.0;
    unsigned int index = 0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

  public:
    Eigen::Quaternionf GetQuaternion();
};
}

#endif