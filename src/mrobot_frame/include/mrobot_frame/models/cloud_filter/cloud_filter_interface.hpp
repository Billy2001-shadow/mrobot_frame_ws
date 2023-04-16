#ifndef MROBOT_FRAME_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_
#define MROBOT_FRAME_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include "mrobot_frame/sensor_data/cloud_data.hpp"

namespace mrobot_frame {
class CloudFilterInterface {
  public:
    virtual ~CloudFilterInterface() = default;

    virtual bool Filter(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) = 0;
};
}

#endif