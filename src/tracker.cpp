#include "tracker.hpp"

#include <cmath>

#include <Eigen/LU> // 3x3 求逆的实现在这里
#include <yaml-cpp/yaml.h>

namespace vision
{

Tracker::Tracker(const std::string &config_path)
{
    const YAML::Node yaml = YAML::LoadFile(config_path); // 缺少配置项或类型错误会抛异常
    process_noise_ = yaml["track_process_noise"].as<double>();
    measure_noise_ = yaml["track_measure_noise"].as<double>();
    yaw_noise_ = yaml["track_yaw_noise"].as<double>();
    rotation_noise_ = yaml["track_rotation_noise"].as<double>();
    radius_ = yaml["armor_radius"].as<double>() / 1000.0; // 配置写 mm，算的时候用 m
    armor_count_ = yaml["armor_count"].as<int>();
    max_lost_ = yaml["track_max_lost"].as<int>();
}

Target Tracker::current() const
{
    Target target;
    target.center = x_.head<3>();
    target.velocity = x_.segment<3>(3);
    target.radius = radius_;
    target.theta = x_(6);
    target.omega = x_(7);
    return target;
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
        x_(6) = first->yaw;                                // 板面角度直接给出装甲板在圆上的方位
        x_.head<3>() = first->xyz - current().armor(0.0); // 圆心在板子身后一个半径
        P_.setZero();
        P_.diagonal() << 1, 1, 1, 100, 100, 100, 0.1, 25; // 速度、转速都先不信
        last_ = t;
        inited_ = true;
        lost_ = 0;

        return current(); // omega 保持 0，等第二帧才有转速
    }

    predict(std::chrono::duration<double>(t - last_).count());
    last_ = t;

    const Eigen::Vector3d predicted = current().armor(0.0); // 预测的那块板在哪
    const Armor *nearest = nullptr;
    double nearest_sq = 0;
    for (const auto &armor : armors)
    {
        if (!armor.solved)
            continue;
        const double sq = (armor.xyz - predicted).squaredNorm();
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
        return current(); // 短暂丢失就靠预测撑过去
    }

    lost_ = 0;
    update(*nearest);
    return current();
}

void Tracker::predict(double dt)
{
    // 圆心等速，转角按角速度推进
    Covariance F = Covariance::Identity();
    F.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
    F(6, 7) = dt;

    // 圆心用白噪声加速度模型，转角用白噪声角加速度模型
    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Covariance Q = Covariance::Zero();
    Q.block<3, 3>(0, 0) = dt * dt * dt * dt / 4 * I;
    Q.block<3, 3>(0, 3) = dt * dt * dt / 2 * I;
    Q.block<3, 3>(3, 0) = dt * dt * dt / 2 * I;
    Q.block<3, 3>(3, 3) = dt * dt * I;
    Q *= process_noise_;
    Q(6, 6) = rotation_noise_ * dt * dt * dt / 3;
    Q(6, 7) = rotation_noise_ * dt * dt / 2;
    Q(7, 6) = rotation_noise_ * dt * dt / 2;
    Q(7, 7) = rotation_noise_ * dt;

    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q;
}

void Tracker::update(const Armor &armor)
{
    // 板面角度是 atan2 出来的，落在 ±π；状态里的转角一直在累加，先折到同一分支
    double diff = std::remainder(armor.yaw - x_(6), 2 * CV_PI);
    if (std::abs(diff) > CV_PI / armor_count_)
    {
        x_(6) += diff; // 换板：转角直接对齐新板，转速不受影响
        diff = 0.0;    // 这一帧已经对齐，不再用它修正转角
    }
    const double yaw = x_(6) + diff;

    const double theta = x_(6);
    const double s = std::sin(theta);
    const double c = std::cos(theta);

    // 观测是装甲板位置和板面角度，位置对转角有耦合
    Eigen::Matrix<double, 4, 8> H = Eigen::Matrix<double, 4, 8>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    H(0, 6) = radius_ * c;
    H(2, 6) = -radius_ * s;
    H(3, 6) = 1.0;

    Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
    R.diagonal() << measure_noise_, measure_noise_, measure_noise_, yaw_noise_;

    Eigen::Vector4d z;
    z << armor.xyz, yaw;

    Eigen::Vector4d h;
    h << current().armor(0.0), x_(6);

    const Eigen::Matrix4d S = H * P_ * H.transpose() + R;
    const Eigen::Matrix<double, 8, 4> K = P_ * H.transpose() * S.inverse();

    x_ += K * (z - h);
    P_ = (Covariance::Identity() - K * H) * P_;
}

} // namespace vision
