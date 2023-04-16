#include "mrobot_frame/mapping/front_end/front_end.hpp"

#include <fstream>
#include <iostream>
#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"
#include "mrobot_frame/global_defination/global_defination.h"

#include "mrobot_frame/models/registration/ndt_registration.hpp"
#include "mrobot_frame/models/cloud_filter/voxel_filter.hpp"
#include "mrobot_frame/models/cloud_filter/no_filter.hpp"

namespace mrobot_frame {
FrontEnd::FrontEnd()
    :local_map_ptr_(new CloudData::CLOUD()) {
    
    InitWithConfig();
}

//初始化参数，匹配方法，滤波
bool FrontEnd::InitWithConfig() {
    std::string config_file_path = WORK_SPACE_PATH + "/config/mapping/front_end.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    first_scan_ = true;

    std::cout << "-----------------前端初始化-------------------" << std::endl;

    InitParam(config_node);
    InitRegistration(registration_ptr_, config_node);
    InitFilter("local_map", local_map_filter_ptr_, config_node);
    InitFilter("frame", frame_filter_ptr_, config_node);

    return true;
}

bool FrontEnd::InitParam(const YAML::Node& config_node) {
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    local_frame_num_ = config_node["local_frame_num"].as<int>();

    return true;
}

bool FrontEnd::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node) {
    std::string registration_method = config_node["registration_method"].as<std::string>();
    std::cout << "前端选择的点云匹配方式为：" << registration_method << std::endl;

    if (registration_method == "NDT") {
        //此处要make_shared要添具体的子类，而不是父类RegistrationInterface，体现多态
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    } else {
        // LOG(ERROR) << "没找到与 " << registration_method << " 相对应的点云匹配方式!";
        std::cout << "没找到与 " << registration_method << " 相对应的点云匹配方式!";
        return false;
    }

    return true;
}

bool FrontEnd::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
    std::cout << "前端" << filter_user << "选择的滤波方法为：" << filter_mothod << std::endl;

    if (filter_mothod == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
    } else if (filter_mothod == "no_filter") {
        filter_ptr = std::make_shared<NoFilter>();
    } else {
        // LOG(ERROR) << "没有为 " << filter_user << " 找到与 " << filter_mothod << " 相对应的滤波方法!";
        std::cout << "没有为 " << filter_user << " 找到与 " << filter_mothod << " 相对应的滤波方法!";
        return false;
    }

    return true;
}

//得到一帧点云，就返回一个位姿
bool FrontEnd::Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose) {
    current_frame_.cloud_data.time = cloud_data.time;
    std::vector<int> indices;
    //将原始数据进行处理，过滤后的数据在原始数据中的下标保存在indices中
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, indices);

    //滤波后保存
    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
    frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr, filtered_cloud_ptr);

    //类的所有对象调用这个函数都可以使用这些局部静态变量
    //只有第一次才会进行这些初始化语句，不会反复初始化的
    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = init_pose_;
    
    // if (first_scan_) {
    //     first_scan_ = false;
    //     current_frame_.pose = init_pose_;
    //     UpdateWithNewFrame(current_frame_);
    //     cloud_pose = current_frame_.pose;
    //     return true;
    // }

    // 局部地图容器中没有关键帧，代表是第一帧数据
    // 此时把当前帧数据作为第一个关键帧，并更新局部地图容器和全局地图容器
    if (local_map_frames_.size() == 0) {
        current_frame_.pose = init_pose_;
        UpdateWithNewFrame(current_frame_);
        cloud_pose = current_frame_.pose;
        return true;
    }

    // 不是第一帧，就正常匹配
    CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
    //UpdateWithNewFrame()中设置了targrt,即局部地图，ScanMatch给预测位姿，和匹配点云，返回计算出的位姿
    registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr, current_frame_.pose);
    cloud_pose = current_frame_.pose;

    // 更新相邻两帧的相对运动
    step_pose = last_pose.inverse() * current_frame_.pose;
    predict_pose = current_frame_.pose * step_pose;
    last_pose = current_frame_.pose;

    // UpdateWithNewFrame(current_frame_);

    // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
    if (fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) + 
        fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) +
        fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3)) > key_frame_distance_) {
        UpdateWithNewFrame(current_frame_);
        last_key_frame_pose = current_frame_.pose; 
    }

    return true;
}

bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose) {
    init_pose_ = init_pose;
    return true;
}

bool FrontEnd::UpdateWithNewFrame(const Frame& new_key_frame) {
    Frame key_frame = new_key_frame;
    // 这一步的目的是为了把关键帧的点云保存下来
    // 由于用的是共享指针，所以直接复制只是复制了一个指针而已
    // 此时无论你放多少个关键帧在容器里，这些关键帧点云指针都是指向的同一个点云
    //为了让key_frame不和输入new_key_frame指向同一个点云，所以要new一个点云，让key_frame的指针指向它
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());
    
    // pcl::transformPointCloud(*key_frame.cloud_data.cloud_ptr, 
    //                              *transformed_cloud_ptr, 
    //                              key_frame.pose);
    // registration_ptr_->SetInputTarget(transformed_cloud_ptr);

    // 更新局部地图
    local_map_frames_.push_back(key_frame);
    while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_)) {
        local_map_frames_.pop_front();
    }
    local_map_ptr_.reset(new CloudData::CLOUD());
    //local_map_ptr_是局部帧叠加在一起的局部地图
    //因为点云是基于雷达坐标的，所以要把每帧点云变换到以第一帧为标准的参考系，才能在空间上正确叠加
    for (size_t i = 0; i < local_map_frames_.size(); ++i) {
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr, 
                                 *transformed_cloud_ptr, 
                                 local_map_frames_.at(i).pose);

        *local_map_ptr_ += *transformed_cloud_ptr;
        // *local_map_ptr_ += *local_map_frames_.at(i).cloud_data.cloud_ptr;
    }

    // 更新ndt匹配的目标点云
    // 关键帧数量还比较少的时候不滤波，因为点云本来就不多，太稀疏影响匹配效果
    // registration_ptr_->SetInputTarget(local_map_ptr_);
    if (local_map_frames_.size() < 10) {
        registration_ptr_->SetInputTarget(local_map_ptr_);
    } else {
        CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
        local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);
        registration_ptr_->SetInputTarget(filtered_local_map_ptr);
    }

    return true;
}
}