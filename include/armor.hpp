#pragma once

#include <array>

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace vision
{

// 装甲板：Detector 填像素角点，Solver 填世界系坐标
struct Armor
{
    std::array<cv::Point2f, 4> corners;            // 像素角点，左上→右上→右下→左下，取灯条中轴线端点
    Eigen::Vector3d xyz = Eigen::Vector3d::Zero(); // 世界系坐标，solved 为 true 时有效
    bool solved = false;                           // Solver 解算成功才置 true
};

} // namespace vision
