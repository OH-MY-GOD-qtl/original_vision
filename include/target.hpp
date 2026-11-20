#pragma once

#include <cmath>

#include <Eigen/Core>

namespace vision
{

// 跟踪结果：目标绕竖直轴旋转，装甲板绕中心做圆周运动，全为世界系
struct Target
{
    Eigen::Vector3d center = Eigen::Vector3d::Zero();   // 旋转中心
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero(); // 中心速度
    double radius = 0.0;                                // 板到中心的水平距离，m
    double theta = 0.0;                                 // 板的方位角，rad，正为向右
    double omega = 0.0;                                 // 旋转角速度，rad/s，正为向右转

    Eigen::Vector3d armor(double t) const;          // t 秒后装甲板的位置
    Eigen::Vector3d armor_velocity(double t) const; // t 秒后装甲板的速度
    double armor_yaw(double t) const;               // t 秒后板面法线的世界偏航角
};

inline Eigen::Vector3d Target::armor(double t) const
{
    const double a = theta + omega * t;
    return center + velocity * t + radius * Eigen::Vector3d(std::sin(a), 0.0, std::cos(a));
}

inline Eigen::Vector3d Target::armor_velocity(double t) const
{
    const double a = theta + omega * t;
    return velocity + radius * omega * Eigen::Vector3d(std::cos(a), 0.0, -std::sin(a));
}

inline double Target::armor_yaw(double t) const { return theta + omega * t; }

} // namespace vision
