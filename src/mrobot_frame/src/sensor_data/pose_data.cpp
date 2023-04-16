#include "mrobot_frame/sensor_data/pose_data.hpp"

namespace mrobot_frame {
Eigen::Quaternionf PoseData::GetQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);

    return q;
}
}