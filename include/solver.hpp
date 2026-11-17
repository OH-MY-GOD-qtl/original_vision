#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "armor.hpp"

namespace vision
{

// 解算：像素角点 + 云台姿态 → 世界系坐标
// 云台系：x 右、y 下、z 前（右手系，与相机系同构，对准时手眼为单位阵）
// 角度：yaw = atan2(x, z) 正为向右；pitch = atan2(y, z) 正为向下
// 方向向量 = (sin yaw·cos pitch, sin pitch, cos yaw·cos pitch)
class Solver
{
  public:
    explicit Solver(const std::string &config_path);

    void set_R_gimbal2world(const Eigen::Quaterniond &q); // 用该帧时间戳的姿态，不用 now()

    void solve(Armor &armor); // 成功填 xyz 并置 solved，失败保持 false

  private:
    cv::Mat camera_matrix_;          // 3×3 内参，像素
    cv::Mat distort_coeffs_;         // k1 k2 p1 p2 k3
    std::vector<cv::Point3f> model_; // 装甲板四角，单位 m，左上 右上 右下 左下

    Eigen::Matrix3d R_camera2gimbal_ = Eigen::Matrix3d::Identity(); // 相机 → 云台
    Eigen::Vector3d t_camera2gimbal_ = Eigen::Vector3d::Zero();     // 光心相对云台原点，m

    Eigen::Quaterniond R_gimbal2world_ = Eigen::Quaterniond::Identity();
};

} // namespace vision
