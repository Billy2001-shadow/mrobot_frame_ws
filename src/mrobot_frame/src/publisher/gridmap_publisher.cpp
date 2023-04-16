#include "mrobot_frame/publisher/gridmap_publisher.hpp"

namespace mrobot_frame {
GridmapPublisher::GridmapPublisher(ros::NodeHandle& nh,
                                    std::string topic_name,
                                    std::string frame_id,
                                    size_t buff_size)
    :nh_(nh), frame_id_(frame_id){ //frame_id一般为map坐标系
    publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>(topic_name, buff_size);  //发布占据栅格地图        

}
                   
void GridmapPublisher::Publish(const nav_msgs::OccupancyGrid& rosMap) {
    nav_msgs::OccupancyGrid Gridmap;
    
    Gridmap = rosMap;
    // Gridmap.info.resolution = rosMap.info.resolution; //这个通过yaml文件给mapParams.resolution置位
    // Gridmap.info.origin.position.x = 0.0;
    // Gridmap.info.origin.position.y = 0.0;
    // Gridmap.info.origin.position.z = 0.0;
    // Gridmap.info.origin.orientation.x = 0.0;
    // Gridmap.info.origin.orientation.y = 0.0;
    // Gridmap.info.origin.orientation.z = 0.0;
    // Gridmap.info.origin.orientation.w = 1.0;

    // Gridmap.info.origin.position.x = rosMap.info.origin.position.x;//这个通过yaml文件给mapParams.resolution置位
    // Gridmap.info.origin.position.y = rosMap.info.origin.position.y;//这些参数应该在mapping类下搞定的
    // Gridmap.info.width = rosMap.info.width;
    // Gridmap.info.height = rosMap.info.height;
    // Gridmap.data.resize(rosMap.info.width * rosMap.info.height);

    // //0~100(重点搞清楚pMap的来源)
    // // int cnt0, cnt1, cnt2;
    // // cnt0 = cnt1 = cnt2 = 100;
    // for (int i = 0; i < Gridmap.info.width * Gridmap.info.height; i++)
    // {
    //     if (Gridmap.data[i] == 50) //？
    //     {
    //         Gridmap.data[i] = -1.0; //Unknown is -1.
    //         std::cout << Gridmap.data[i] << std::endl;
    //     }
    //     else
    //     {

    //         Gridmap.data[i] = rosMap.data[i]; //unsigned char *pMap; //指向unsigned char类型元素的指针
    //         //std::cout << Gridmap.data[i] << std::endl;
    //     }
    // }

    Gridmap.header.stamp = ros::Time::now();
    Gridmap.header.frame_id = frame_id_;
    //std::cout << "建图完毕..." << std::endl;
    publisher_.publish(Gridmap);
    
}

}