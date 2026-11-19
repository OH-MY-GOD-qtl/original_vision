#pragma once

#include <cmath>

#include <Eigen/Core>

namespace vision
{

// 项目统一的角度约定：yaw = atan2(x, z) 正为向右，pitch = asin(y / |p|) 正为向下

// 世界系角度 → 单位方向
inline Eigen::Vector3d direction_of(double yaw, double pitch)
{
    return {std::sin(yaw) * std::cos(pitch), std::sin(pitch), std::cos(yaw) * std::cos(pitch)};
}

// 世界系点 → yaw 与 pitch，与 direction_of 互逆
inline Eigen::Vector2d angles_of(const Eigen::Vector3d &p)
{
    return {std::atan2(p.x(), p.z()), std::asin(p.y() / p.norm())};
}

} // namespace vision
