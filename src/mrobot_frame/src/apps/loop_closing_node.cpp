#include <ros/ros.h>
#include "glog/logging.h"

#include "mrobot_frame/mapping/loop_closing/loop_closing_flow.hpp"

using namespace mrobot_frame;

int main(int argc, char *argv[]) {
    

    ros::init(argc, argv, "mrobot_frame_loop_closing_node");
    ros::NodeHandle nh;

    std::shared_ptr<LoopClosingFlow> loop_closing_flow_ptr = std::make_shared<LoopClosingFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        loop_closing_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}