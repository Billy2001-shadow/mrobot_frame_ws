#include <ros/ros.h>
#include "mrobot_frame/data_pretreat/data_pretreat_flow.hpp"

using namespace mrobot_frame;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mrobot_frame_data_pretreat_node"); // 节点的名字
    ros::NodeHandle nh;

    std::shared_ptr<DataPretreatFlow> data_pretreat_flow_ptr = std::make_shared<DataPretreatFlow>(nh, "/pretreat_cloud");
    

    ros::Rate rate(100);
    while (ros::ok()) {
        
        ros::spinOnce();

        data_pretreat_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}
