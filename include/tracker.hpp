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

// 跟踪：多帧装甲板 → 唯一目标。装甲板固定挂在一个圆上，所以跟踪的是圆心与转角
class Tracker
{
  public:
    explicit Tracker(const std::string &config_path);

    std::optional<Target> track(
        const std::vector<Armor> &armors, std::chrono::steady_clock::time_point t);

  private:
    using State = Eigen::Matrix<double, 8, 1>; // [cx cy cz vx vy vz theta omega]
    using Covariance = Eigen::Matrix<double, 8, 8>;

    void predict(double dt);            // 圆心等速、转角随角速度外推
    void update(const Armor &armor);    // 用位置和板面角度修正
    Target current() const;             // 状态转成对外结果

    State x_ = State::Zero();
    Covariance P_ = Covariance::Identity();

    std::chrono::steady_clock::time_point last_{}; // 上一帧时刻
    bool inited_ = false;                          // 是否已用第一帧初始化
    int lost_ = 0;                                 // 连续丢失帧数

    double process_noise_;  // 圆心过程噪声谱密度，越大越信观测
    double measure_noise_;  // 观测噪声方差，m²
    double yaw_noise_;      // 板面角度观测噪声方差，rad²
    double rotation_noise_; // 角速度谱密度，越大越跟得上急转
    double radius_;         // 装甲板到中心的水平距离，m，固定值
    int armor_count_;       // 装甲板数量，相邻两块间隔 360°/armor_count
    int max_lost_;          // 连续丢失多少帧后放弃
};

} // namespace vision
