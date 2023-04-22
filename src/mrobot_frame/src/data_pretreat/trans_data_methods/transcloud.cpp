#include "mrobot_frame/data_pretreat/trans_data_methods/transcloud.hpp"

namespace mrobot_frame {
bool Transcloud::LaserscanToPcl(const sensor_msgs::LaserScan::ConstPtr& scan, CloudData& pclcloud){
    laser_geometry::LaserProjection* projector_ = new laser_geometry::LaserProjection();
    sensor_msgs::PointCloud2 cloud;

    //普通转换
    projector_->projectLaser(*scan, cloud);  //转换的过程中会把inf等异常值去除掉

    //使用tf的转换
    // projector_->transformLaserScanToPointCloud("laser", *scan, cloud, tfListener_);
    pclcloud.time = cloud.header.stamp.toSec();

    pcl::fromROSMsg(cloud, *(pclcloud.cloud_ptr));

    return true;
}

bool Transcloud::LasersacanToPoint2(const sensor_msgs::LaserScan::ConstPtr& scan, sensor_msgs::PointCloud2& cloud) {
    laser_geometry::LaserProjection* projector_ = new laser_geometry::LaserProjection();
    projector_->projectLaser(*scan, cloud);     
    return true;

}

bool Transcloud::PclToPoint2(CloudData::CLOUD_PTR& pclcloud_ptr, sensor_msgs::PointCloud2Ptr& cloud) {
    pcl::toROSMsg(*pclcloud_ptr, *cloud);
    return true;
}

bool Transcloud::Point2ToPcl(const sensor_msgs::PointCloud2& cloud, CloudData& pclcloud) {
    pclcloud.time = cloud.header.stamp.toSec();
    pcl::fromROSMsg(cloud, *(pclcloud.cloud_ptr));

    return true;
}

bool Transcloud::Point2ToPcl(const sensor_msgs::PointCloud2::ConstPtr& cloud_ptr, CloudData& pclcloud) {
    pclcloud.time = cloud_ptr->header.stamp.toSec();
    pcl::fromROSMsg(*cloud_ptr, *(pclcloud.cloud_ptr));

    return true;
}
}
