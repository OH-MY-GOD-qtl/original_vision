#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "armor.hpp"
#include "target.hpp"

namespace vision
{

// 跟踪：多帧装甲板 → 唯一目标，顺带滤波预测
class Tracker
{
  public:
    explicit Tracker(const std::string &config_path);

    std::optional<Target> track(
        const std::vector<Armor> &armors, std::chrono::steady_clock::time_point t);

  private:
    using State = Eigen::Matrix<double, 6, 1>;      // [x y z vx vy vz]
    using Covariance = Eigen::Matrix<double, 6, 6>;

    void predict(double dt);               // 按等速模型外推到当前时刻
    void update(const Eigen::Vector3d &z); // 用观测修正位置与速度

    State x_ = State::Zero();
    Covariance P_ = Covariance::Identity();

    std::chrono::steady_clock::time_point last_{}; // 上一帧时刻
    bool inited_ = false;                          // 是否已用第一帧初始化
    int lost_ = 0;                                 // 连续丢失帧数

    double process_noise_;  // 过程噪声谱密度，越大越信观测
    double measure_noise_;  // 观测噪声方差，m²
    int max_lost_;          // 连续丢失多少帧后放弃
};

} // namespace vision
