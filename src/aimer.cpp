#include "aimer.hpp"

#include <cmath>

#include <yaml-cpp/yaml.h>

namespace
{

constexpr double deg2rad = 3.14159265358979323846 / 180.0;
constexpr double gravity = 9.8; // 重力加速度，m/s²，方向竖直向下

// 朝 0 走一步，单步不超过 step
double toward_zero(double value, double step)
{
    if (value > step)
        return value - step;
    if (value < -step)
        return value + step;
    return 0.0;
}

} // namespace

namespace vision
{

Aimer::Aimer(const std::string &config_path)
{
    const YAML::Node yaml = YAML::LoadFile(config_path); // 缺少配置项或类型错误会抛异常
    lost_hold_s_ = yaml["lost_hold_ms"].as<double>() / 1000.0;
    return_rad_per_s_ = yaml["return_deg_per_s"].as<double>() * deg2rad;
}

Command Aimer::aim(
    const std::optional<Target> &target, double bullet_speed, double gimbal_yaw,
    double gimbal_pitch, std::chrono::steady_clock::time_point t)
{
    const double dt = std::chrono::duration<double>(t - last_call_).count();
    last_call_ = t;

    if (!started_) // 首帧把云台当前姿态当作原位，否则会直接下发 0,0 把云台拽走
    {
        last_command_.yaw = gimbal_yaw;
        last_command_.pitch = gimbal_pitch;
        last_target_ = t;
        started_ = true;
    }

    if (target)
    {
        last_target_ = t;
        last_command_ = solve(*target, bullet_speed, t);
        last_command_.is_empty = false;
        return last_command_;
    }

    last_command_.is_empty = true; // 目标丢了，但停留和归零仍由视觉接管

    if (std::chrono::duration<double>(t - last_target_).count() < lost_hold_s_)
        return last_command_; // 停留期：保持原位

    const double step = return_rad_per_s_ * dt; // 每次调用只走这么多，所以是缓慢归零
    last_command_.yaw = toward_zero(last_command_.yaw, step);
    last_command_.pitch = toward_zero(last_command_.pitch, step);
    last_command_.distance = 0.0;
    return last_command_;
}

Command Aimer::solve(
    const Target &target, double bullet_speed, std::chrono::steady_clock::time_point) const
{
    // 预瞄点越远飞行时间越长，时间越长预瞄点又越远，先粗算一轮再修正
    const double now_range = target.armor(0.0).norm();
    const Eigen::Vector3d rough = target.armor(now_range / bullet_speed);
    const double flight = rough.norm() / bullet_speed;

    Eigen::Vector3d aim = target.armor(flight); // 装甲板沿圆周走到那儿
    aim.y() -= 0.5 * gravity * flight * flight; // 重力下坠，y 朝下所以减 y 等于抬高

    const double distance = aim.norm();
    Command command;
    command.yaw = std::atan2(aim.x(), aim.z());
    command.pitch = std::asin(aim.y() / distance); // 仰角，与 Solver 的 Ry*Rx 云台模型一致
    command.distance = distance;
    return command;
}

} // namespace vision
