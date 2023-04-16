#include "mrobot_frame/models/cloud_filter/no_filter.hpp"
// #include "glog/logging.h"

namespace mrobot_frame {
NoFilter::NoFilter() {
}

bool NoFilter::Filter(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) {
    filtered_cloud_ptr.reset(new CloudData::CLOUD(*input_cloud_ptr));
    return true;
}
} 