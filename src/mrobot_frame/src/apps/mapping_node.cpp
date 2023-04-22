#include <ros/ros.h>
#include "glog/logging.h"

#include <mrobot_frame/saveGridmap.h>
#include "mrobot_frame/global_defination/global_defination.h"
#include "mrobot_frame/mapping/mapping/mapping_flow.hpp"

using namespace mrobot_frame;

std::shared_ptr<MappingFlow> _mapping_flow_ptr;

bool _need_save_Gridmap = false;
bool save_Gridmap_callback(saveGridmap::Request &request, saveGridmap::Response &response) {
    _need_save_Gridmap = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char *argv[]) {

    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/log";
    FLAGS_alsologtostderr = 1;
    FLAGS_colorlogtostderr = true;  //是否启用不同颜色显示

 
    // google::SetLogDestination(google::GLOG_INFO,"/home/ncu/chenw/mrobot_frame_ws/src/mrobot_frame/log/info/");  //设置日志级别
    // google::SetLogDestination(google::GLOG_WARNING,"/home/ncu/chenw/mrobot_frame_ws/src/mrobot_frame/log/warning/");
    // google::SetLogDestination(google::GLOG_ERROR,"/home/ncu/chenw/mrobot_frame_ws/src/mrobot_frame/log/error/");

    ros::init(argc, argv, "mrobot_frame_mapping_node");
    ros::NodeHandle nh;
    std::string cloud_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/pretreat_cloud");
    
    std::shared_ptr<MappingFlow> _mapping_flow_ptr = std::make_shared<MappingFlow>(nh,cloud_topic);

    ros::ServiceServer service = nh.advertiseService("save_Gridmap", save_Gridmap_callback); //需不需要建图

    ros::Rate rate(100);

    LOG(INFO) << "[MAIN] mapping_node start!";


    while (ros::ok()) {
        ros::spinOnce();

        _mapping_flow_ptr->Run();

        if (_need_save_Gridmap) {
            _mapping_flow_ptr->SaveGridmap();
            _need_save_Gridmap = false;
        }

        rate.sleep();
    }

    return 0;
}