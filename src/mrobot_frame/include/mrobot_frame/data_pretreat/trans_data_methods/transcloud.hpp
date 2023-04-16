#ifndef MROBOT_FRAME_TRANSCLOUD_HPP_
#define MROBOT_FRAME_TRANSCLOUD_HPP_
#include "mrobot_frame/sensor_data/cloud_data.hpp"

#include <iostream>

#include <ros/ros.h>
// #include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

namespace mrobot_frame {
class Transcloud {
    public:
        static bool LaserscanToPcl(const sensor_msgs::LaserScan::ConstPtr& scan, CloudData& pclcloud);
        static bool LasersacanToPoint2(const sensor_msgs::LaserScan::ConstPtr& scan, sensor_msgs::PointCloud2& cloud);
        static bool PclToPoint2(CloudData::CLOUD_PTR& pclcloud_ptr, sensor_msgs::PointCloud2Ptr& cloud);
        static bool Point2ToPcl(const sensor_msgs::PointCloud2& cloud, CloudData& pclcloud);
        static bool Point2ToPcl(const sensor_msgs::PointCloud2::ConstPtr& cloud_ptr, CloudData& pclcloud);
    
    private:
        static laser_geometry::LaserProjection* projector_;
        unsigned int seq_number_;
};
}

#endif

