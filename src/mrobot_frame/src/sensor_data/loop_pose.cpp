#include "mrobot_frame/sensor_data/loop_pose.hpp"

namespace mrobot_frame {
Eigen::Quaternionf LoopPose::GetQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);

    return q;
}
}