#pragma once

#include <string>

#include <Eigen/Geometry>

#include "armor.hpp"

namespace vision
{

// 解算：像素角点 + 云台姿态 → 云台系坐标
class Solver
{
  public:
    explicit Solver(const std::string &config_path);

    void set_R_gimbal2world(const Eigen::Quaterniond &q); // 用该帧时间戳的姿态，不用 now()

    void solve(Armor &armor); // 成功填 xyz 并置 solved，失败保持 false

  private:
    Eigen::Quaterniond R_gimbal2world_ = Eigen::Quaterniond::Identity();
};

} // namespace vision
