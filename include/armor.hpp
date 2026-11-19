#pragma once

#include <array>

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace vision
{

// 装甲板：Detector 填像素角点，Solver 填世界系坐标与板面角度
struct Armor
{
    std::array<cv::Point2f, 4> corners;            // 像素角点，左上→右上→右下→左下，取灯条中轴线端点
    Eigen::Vector3d xyz = Eigen::Vector3d::Zero(); // 世界系坐标，solved 为 true 时有效
    double yaw = 0.0;                              // 板面法线的世界系偏航角，rad，正为向右
    bool solved = false;                           // Solver 解算成功才置 true
};

// 四角点求中心，像素角点和投影结果都走它
inline cv::Point2f center_of(const std::array<cv::Point2f, 4> &corners)
{
    return (corners[0] + corners[1] + corners[2] + corners[3]) * 0.25f;
}

} // namespace vision
