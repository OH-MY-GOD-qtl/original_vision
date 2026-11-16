#include "detector.hpp"

namespace vision
{

Detector::Detector(const std::string &) {} // TODO: 读色差与亮度阈值

std::vector<Armor> Detector::detect(const cv::Mat &) const
{
    return {}; // TODO: 色差分离 → 灯条 → 配对 → 四角点
}

} // namespace vision
