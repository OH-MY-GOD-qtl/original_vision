#pragma once

#include <array>

#include <opencv2/imgproc.hpp>

namespace vision
{

// 画面配色：绿检出 / 深绿滤波后 / 红击打位置 / 灰检出未解算
inline const cv::Scalar color_detected(0, 255, 0);
inline const cv::Scalar color_tracked(0, 128, 0);
inline const cv::Scalar color_hit(0, 0, 255);
inline const cv::Scalar color_unsolved(128, 128, 128);

// 按角点顺序连成闭合四边形
inline void draw_quad(cv::Mat &img, const std::array<cv::Point2f, 4> &corners,
                      const cv::Scalar &color)
{
    for (size_t i = 0; i < 4; ++i)
        cv::line(img, corners[i], corners[(i + 1) % 4], color, 2);
}

} // namespace vision
