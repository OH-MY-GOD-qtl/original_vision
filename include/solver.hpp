#pragma once

#include <array>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "armor.hpp"

namespace vision
{

// 解算：像素角点 + 云台姿态 → 世界系坐标 + 板面朝向
// 云台系：x 右、y 下、z 前（右手系，与相机系同构，对准时手眼为单位阵）
// 角度：yaw = atan2(x, z) 正为向右；pitch = asin(y / |p|) 正为向下，即仰角
// 方向向量 = (sin yaw·cos pitch, sin pitch, cos yaw·cos pitch)
class Solver
{
  public:
    explicit Solver(const std::string &config_path);

    void set_attitude(double yaw, double pitch); // 相机光轴的世界 yaw / pitch，rad

    void solve(Armor &armor); // 成功填 xyz 与板面 yaw 并置 solved，失败保持 false

    // 世界系 → 像素：把装甲板四角投回图像，姿态按正面朝相机处理，仅供可视化
    std::array<cv::Point2f, 4> project(const Eigen::Vector3d &xyz) const;

  private:
    cv::Mat camera_matrix_;          // 3×3 内参，像素
    cv::Mat distort_coeffs_;         // k1 k2 p1 p2 k3
    std::vector<cv::Point3f> model_; // 装甲板四角，单位 m，左上 右上 右下 左下

    Eigen::Matrix3d R_camera2gimbal_ = Eigen::Matrix3d::Identity(); // 相机 → 云台
    Eigen::Vector3d t_camera2gimbal_ = Eigen::Vector3d::Zero();     // 光心相对云台原点，m

    Eigen::Quaterniond R_gimbal2world_ = Eigen::Quaterniond::Identity();
};

} // namespace vision
