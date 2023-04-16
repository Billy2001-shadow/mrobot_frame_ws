#ifndef MROBOT_FRAME_MODELS_CLOUD_FILTER_NO_FILTER_HPP_
#define MROBOT_FRAME_MODELS_CLOUD_FILTER_NO_FILTER_HPP_

#include "mrobot_frame/models/cloud_filter/cloud_filter_interface.hpp"

namespace mrobot_frame {
class NoFilter: public CloudFilterInterface {
  public:
    NoFilter();

    bool Filter(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;
};
}
#endif