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
        const std::optional<Target> &target, double bullet_speed, double gimbal_yaw,
        double gimbal_pitch, std::chrono::steady_clock::time_point t);

  private:
    Command solve(const Target &target, double bullet_speed) const;
    int best_plate(const Target &target, double t) const; // t 时刻最正面朝向相机的板

    double gravity_;          // 重力加速度，m/s²，方向竖直向下
    double lost_hold_s_;      // 丢失目标后原位停留时长
    double return_rad_per_s_; // 归位角速度
    double latency_s_;        // 曝光中点到云台到位的延迟，s
    bool started_ = false;    // 首帧已把当前云台姿态当作原位
    Command last_command_{};  // 最近一次发出的指令
    std::chrono::steady_clock::time_point last_target_{}; // 最后一次看到目标的时刻
    std::chrono::steady_clock::time_point last_call_{};   // 上一次调用时刻，用来算步长
};

} // namespace vision
