#pragma once

#include <chrono>
#include <optional>
#include <string>

#include "command.hpp"
#include "target.hpp"

namespace vision
{

// 瞄准：目标 → 云台指令；无目标时先原位停留，再缓慢归零
class Aimer
{
  public:
    explicit Aimer(const std::string &config_path);

    Command aim(
        const std::optional<Target> &target, double bullet_speed,
        std::chrono::steady_clock::time_point t);

  private:
    Command solve(
        const Target &target, double bullet_speed, std::chrono::steady_clock::time_point t) const;

    double lost_hold_s_;      // 丢失目标后原位停留时长
    double return_rad_per_s_; // 归位角速度
    Command last_command_{};  // 最近一次发出的指令
    std::chrono::steady_clock::time_point last_target_{}; // 最后一次看到目标的时刻
    std::chrono::steady_clock::time_point last_call_{};   // 上一次调用时刻，用来算步长
};

} // namespace vision
