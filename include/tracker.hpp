#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <vector>

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
};

} // namespace vision
