#include "tracker.hpp"

namespace vision
{

Tracker::Tracker(const std::string &) {} // TODO: 读滤波参数

std::optional<Target> Tracker::track(
    const std::vector<Armor> &, std::chrono::steady_clock::time_point)
{
    return std::nullopt; // TODO: 选板 + EKF，跳过 solved 为 false 的装甲板
}

} // namespace vision
