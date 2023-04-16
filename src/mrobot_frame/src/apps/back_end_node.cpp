#include <ros/ros.h>
#include "glog/logging.h"

#include <mrobot_frame/optimizeMap.h>
#include "mrobot_frame/mapping/back_end/back_end_flow.hpp"

using namespace mrobot_frame;

std::shared_ptr<BackEndFlow> _back_end_flow_ptr;
bool _need_optimize_map = false;

bool optimize_map_callback(optimizeMap::Request &request, optimizeMap::Response &response) {
    _need_optimize_map = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "mrobot_frame_back_end_node");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("optimize_map", optimize_map_callback); //需不需要优化图
    std::shared_ptr<BackEndFlow> back_end_flow_ptr = std::make_shared<BackEndFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        back_end_flow_ptr->Run();

        if (_need_optimize_map) {
            back_end_flow_ptr->ForceOptimize();
            _need_optimize_map = false;
        }

        rate.sleep();
    }

    return 0;
}