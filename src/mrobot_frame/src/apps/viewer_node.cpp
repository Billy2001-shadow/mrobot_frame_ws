#include <ros/ros.h>
#include "glog/logging.h"

#include <mrobot_frame/saveMap.h>
#include "mrobot_frame/global_defination/global_defination.h"
#include "mrobot_frame/mapping/viewer/viewer_flow.hpp"

using namespace mrobot_frame;

std::shared_ptr<ViewerFlow> _viewer_flow_ptr;
bool _need_save_map = false;

bool save_map_callback(saveMap::Request &request, saveMap::Response &response) {
    _need_save_map = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char *argv[]) {
    

    ros::init(argc, argv, "mrobot_frame_viewer_node");
    ros::NodeHandle nh;

    std::string cloud_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/pretreat_cloud");
    std::shared_ptr<ViewerFlow> _viewer_flow_ptr = std::make_shared<ViewerFlow>(nh, cloud_topic);

    ros::ServiceServer service = nh.advertiseService("save_map", save_map_callback);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        _viewer_flow_ptr->Run();
        if (_need_save_map) {
            _need_save_map = false;
            _viewer_flow_ptr->SaveMap();
        }

        rate.sleep();
    }

    return 0;
}