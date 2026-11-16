#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "armor.hpp"

namespace vision
{

// 检测：图像 → 装甲板像素角点，不做坐标解算
class Detector
{
  public:
    explicit Detector(const std::string &config_path);

    std::vector<Armor> detect(const cv::Mat &img) const;
};

} // namespace vision
