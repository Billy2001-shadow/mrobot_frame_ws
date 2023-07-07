#include <ros/ros.h>
// #include "glog/logging.h"

#include "mrobot_frame/mapping/front_end/front_end_flow.hpp"

using namespace mrobot_frame;

int main(int argc, char *argv[]) {
    

    ros::init(argc, argv, "mrobot_frame_front_end_node");
    ros::NodeHandle nh;

    std::string cloud_topic, odom_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/pretreat_cloud");
    nh.param<std::string>("odom_topic", odom_topic, "/laser_odom_pose");

    std::shared_ptr<FrontEndFlow> front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh, cloud_topic, odom_topic);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        front_end_flow_ptr->Run();

        rate.sleep();  //订阅不那么快？ 控制节点的频率？
    }

    return 0;
}