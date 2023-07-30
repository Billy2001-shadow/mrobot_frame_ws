#include "mrobot_frame/mapping/back_end/back_end.hpp"

#include "glog/logging.h"
#include "mrobot_frame/global_defination/global_defination.h"
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>

#include "mrobot_frame/tools/file_manager.hpp"

namespace mrobot_frame {
BackEnd::BackEnd() { InitWithConfig(); }

bool BackEnd::InitWithConfig() {
  std::string config_file_path =
      WORK_SPACE_PATH + "/config/mapping/back_end.yaml";
  YAML::Node config_node = YAML::LoadFile(config_file_path);

  std::cout << "-----------------后端初始化-------------------" << std::endl;
  InitParam(config_node);
  InitGraphOptimizer(config_node);
  InitDataPath(config_node);

  return true;
}

bool BackEnd::InitParam(const YAML::Node &config_node) {
  key_frame_distance_ = config_node["key_frame_distance"].as<float>();
  key_frame_angular_ = config_node["key_frame_angular"].as<float>();
  return true;
}

bool BackEnd::InitGraphOptimizer(const YAML::Node &config_node) {
  std::string graph_optimizer_type =
      config_node["graph_optimizer_type"].as<std::string>();
  if (graph_optimizer_type == "g2o") {
    graph_optimizer_ptr_ = std::make_shared<G2oGraphOptimizer>("lm_var");
  } else {
    std::cout << "没有找到与 " << graph_optimizer_type
              << " 对应的图优化模式,请检查配置文件";
    return false;
  }
  std::cout << "后端优化选择的优化器为：" << graph_optimizer_type << std::endl
            << std::endl;

  graph_optimizer_config_.use_loop_close =
      config_node["use_loop_close"].as<bool>();

  graph_optimizer_config_.optimize_step_with_key_frame =
      config_node["optimize_step_with_key_frame"].as<int>();
  graph_optimizer_config_.optimize_step_with_loop =
      config_node["optimize_step_with_loop"].as<int>();

  for (int i = 0; i < 6; ++i) {
    graph_optimizer_config_.odom_edge_noise(i) =
        config_node[graph_optimizer_type + "_param"]["odom_edge_noise"][i]
            .as<double>();
    graph_optimizer_config_.close_loop_noise(i) =
        config_node[graph_optimizer_type + "_param"]["close_loop_noise"][i]
            .as<double>();
  }

  return true;
}

bool BackEnd::Update(const CloudData &cloud_data, const PoseData &laser_odom) {
  ResetParam();

  if (MaybeNewKeyFrame(cloud_data, laser_odom)) {
    AddNodeAndEdge();
    MaybeOptimized();
  }

  return true;
}

bool BackEnd::InsertLoopPose(const LoopPose &loop_pose) {
  if (!graph_optimizer_config_.use_loop_close)
    return false;

  Eigen::Isometry3d isometry;
  isometry.matrix() = loop_pose.pose.cast<double>();
  graph_optimizer_ptr_->AddSe3Edge(loop_pose.index0, loop_pose.index1, isometry,
                                   graph_optimizer_config_.close_loop_noise);

  new_loop_cnt_++;
  // std::cout << "插入闭环：" << loop_pose.index0 << "," << loop_pose.index1;

  return true;
}

void BackEnd::ResetParam() {
  has_new_key_frame_ = false;
  has_new_optimized_ = false;
}

bool BackEnd::InitDataPath(const YAML::Node &config_node) {
  std::string data_path = config_node["data_path"].as<std::string>();
  if (data_path == "./") {
    data_path = WORK_SPACE_PATH;
  }

  if (!FileManager::CreateDirectory(data_path + "/slam_data"))
    return false;

  key_frames_path_ = data_path + "/slam_data/key_frames";
  trajectory_path_ = data_path + "/slam_data/trajectory";

  if (!FileManager::InitDirectory(key_frames_path_, "关键帧点云"))
    return false;
  // if (!FileManager::InitDirectory(trajectory_path_, "轨迹文件"))
  //     return false;

  // if (!FileManager::CreateFile(ground_truth_ofs_, trajectory_path_ +
  // "/ground_truth.txt"))
  //     return false;
  // if (!FileManager::CreateFile(laser_odom_ofs_, trajectory_path_ +
  // "/laser_odom.txt"))
  //     return false;

  return true;
}

bool BackEnd::MaybeNewKeyFrame(const CloudData &cloud_data,
                               const PoseData &laser_odom) {
  static Eigen::Matrix4f last_key_pose = laser_odom.pose;
  if (key_frames_deque_.size() == 0) {
    has_new_key_frame_ = true;
    last_key_pose = laser_odom.pose;
  }

  // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
  if ((fabs(laser_odom.pose(0, 3) - last_key_pose(0, 3)) +
           fabs(laser_odom.pose(1, 3) - last_key_pose(1, 3)) +
           fabs(laser_odom.pose(2, 3) - last_key_pose(2, 3)) >
       key_frame_distance_) ||
      (fabs(Matrix4fToYaw(laser_odom.pose) - Matrix4fToYaw(last_key_pose)) >
       key_frame_angular_)) {
    has_new_key_frame_ = true;
    last_key_pose = laser_odom.pose;
  }

  if (has_new_key_frame_) {
    // 把关键帧点云存储到硬盘里
    std::string file_path = key_frames_path_ + "/key_frame_" +
                            std::to_string(key_frames_deque_.size()) + ".pcd";
    pcl::io::savePCDFileBinary(file_path, *cloud_data.cloud_ptr);

    KeyFrame key_frame;
    key_frame.time = laser_odom.time;
    key_frame.index = (unsigned int)key_frames_deque_.size();
    key_frame.pose = laser_odom.pose; //激光里程计算出来的位姿
    key_frames_deque_.push_back(key_frame);

    current_key_frame_ = key_frame;
  }

  return has_new_key_frame_;
}

float BackEnd::Matrix4fToYaw(const Eigen::Matrix4f &MatrixPose) {
  Eigen::Matrix3f rotationMatrix =
      MatrixPose.block<3, 3>(0, 0); // 提取旋转矩阵部分
  Eigen::Vector3f euler =
      rotationMatrix.eulerAngles(2, 1, 0); // 将旋转矩阵转换为欧拉角
  float yaw = euler[0];                    // 获取偏航角（yaw angle）
  return yaw;
}

bool BackEnd::AddNodeAndEdge() {
  Eigen::Isometry3d isometry;
  // 添加关键帧节点
  isometry.matrix() = current_key_frame_.pose.cast<double>();
  graph_optimizer_ptr_->AddSe3Node(isometry, false);
  new_key_frame_cnt_++;

  // 添加激光里程计对应的边
  static KeyFrame last_key_frame = current_key_frame_;
  int node_num = graph_optimizer_ptr_->GetNodeNum();
  if (node_num > 1) {
    Eigen::Matrix4f relative_pose =
        last_key_frame.pose.inverse() * current_key_frame_.pose;
    isometry.matrix() = relative_pose.cast<double>();
    graph_optimizer_ptr_->AddSe3Edge(node_num - 2, node_num - 1, isometry,
                                     graph_optimizer_config_.odom_edge_noise);
  }
  last_key_frame = current_key_frame_;

  return true;
}

bool BackEnd::MaybeOptimized() {
  bool need_optimize = false;

  if (new_loop_cnt_ >= graph_optimizer_config_.optimize_step_with_loop)
    need_optimize = true;
  if (new_key_frame_cnt_ >=
      graph_optimizer_config_.optimize_step_with_key_frame)
    need_optimize = true;

  if (!need_optimize)
    return false;

  new_loop_cnt_ = 0;
  new_key_frame_cnt_ = 0;

  if (graph_optimizer_ptr_->Optimize())
    has_new_optimized_ = true;

  return true;
}

bool BackEnd::ForceOptimize() {
  if (graph_optimizer_ptr_->Optimize())
    has_new_optimized_ = true;
  return has_new_optimized_;
}

void BackEnd::GetOptimizedKeyFrames(std::deque<KeyFrame> &key_frames_deque) {
  key_frames_deque.clear();
  if (graph_optimizer_ptr_->GetNodeNum() > 0) {
    std::deque<Eigen::Matrix4f> optimized_pose;
    graph_optimizer_ptr_->GetOptimizedPose(optimized_pose);
    KeyFrame key_frame;
    for (size_t i = 0; i < optimized_pose.size(); ++i) {
      key_frame.pose = optimized_pose.at(i);
      key_frame.index = (unsigned int)i;
      key_frames_deque.push_back(key_frame);
    }
  }
}

bool BackEnd::HasNewKeyFrame() { return has_new_key_frame_; }

bool BackEnd::HasNewOptimized() { return has_new_optimized_; }

void BackEnd::GetLatestKeyFrame(KeyFrame &key_frame) {
  key_frame = current_key_frame_;
}
} // namespace mrobot_frame