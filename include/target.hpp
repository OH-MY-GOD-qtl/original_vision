#pragma once

#include <Eigen/Core>

namespace vision
{

// 跟踪结果：云台系位置与速度
struct Target
{
    Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
};

} // namespace vision
