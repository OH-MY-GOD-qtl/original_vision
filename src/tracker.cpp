#include "tracker.hpp"

#include <Eigen/LU> // 3x3 求逆的实现在这里
#include <yaml-cpp/yaml.h>

namespace vision
{

Tracker::Tracker(const std::string &config_path)
{
    const YAML::Node yaml = YAML::LoadFile(config_path); // 缺少配置项或类型错误会抛异常
    process_noise_ = yaml["track_process_noise"].as<double>();
    measure_noise_ = yaml["track_measure_noise"].as<double>();
    max_lost_ = yaml["track_max_lost"].as<int>();
}

std::optional<Target> Tracker::track(
    const std::vector<Armor> &armors, std::chrono::steady_clock::time_point t)
{
    if (!inited_)
    {
        const Armor *first = nullptr; // 第一帧没有历史，取第一块解算成功的板
        for (const auto &armor : armors)
        {
            if (!armor.solved)
                continue;
            first = &armor;
            break;
        }
        if (!first)
            return std::nullopt;

        x_.setZero();
        x_.head<3>() = first->xyz;
        P_.setZero();
        P_.diagonal() << 1, 1, 1, 100, 100, 100; // 初始位置 1 m²，速度 100 (m/s)²，都先不信
        last_ = t;
        inited_ = true;
        lost_ = 0;

        Target target;
        target.xyz = x_.head<3>();
        return target; // velocity 保持 0，等第二帧才有速度
    }

    predict(std::chrono::duration<double>(t - last_).count());
    last_ = t;

    const Armor *nearest = nullptr; // 离预测位置最近的那块就是同一块板
    double nearest_sq = 0;
    for (const auto &armor : armors)
    {
        if (!armor.solved)
            continue;
        const double sq = (armor.xyz - x_.head<3>()).squaredNorm();
        if (!nearest || sq < nearest_sq)
        {
            nearest = &armor;
            nearest_sq = sq;
        }
    }

    if (!nearest)
    {
        if (++lost_ > max_lost_)
        {
            inited_ = false; // 丢太久，下块板重新起步
            return std::nullopt;
        }
        Target target; // 短暂丢失就靠预测撑过去
        target.xyz = x_.head<3>();
        target.velocity = x_.tail<3>();
        return target;
    }

    lost_ = 0;
    update(nearest->xyz);

    Target target;
    target.xyz = x_.head<3>();
    target.velocity = x_.tail<3>();
    return target;
}

void Tracker::predict(double dt)
{
    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    Covariance F = Covariance::Identity();
    F.block<3, 3>(0, 3) = dt * I;

    // 白噪声加速度模型：q 是加速度的谱密度
    Covariance Q = Covariance::Zero();
    Q.block<3, 3>(0, 0) = dt * dt * dt * dt / 4 * I;
    Q.block<3, 3>(0, 3) = dt * dt * dt / 2 * I;
    Q.block<3, 3>(3, 0) = dt * dt * dt / 2 * I;
    Q.block<3, 3>(3, 3) = dt * dt * I;

    x_ = F * x_;
    P_ = F * P_ * F.transpose() + process_noise_ * Q;
}

void Tracker::update(const Eigen::Vector3d &z)
{
    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    const Eigen::Matrix3d S = H * P_ * H.transpose() + measure_noise_ * Eigen::Matrix3d::Identity();
    const Eigen::Matrix<double, 6, 3> K = P_ * H.transpose() * S.inverse();

    x_ += K * (z - H * x_);
    P_ = (Covariance::Identity() - K * H) * P_;
}

} // namespace vision
