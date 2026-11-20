#include "solver.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/calib3d.hpp>
#include <yaml-cpp/yaml.h>

namespace
{

constexpr double deg2rad = 3.14159265358979323846 / 180.0;

// 装甲板四角：灯条中轴线端点，单位 m，y 朝下，左上 右上 右下 左下
std::vector<cv::Point3f> model_points(double gap_m, double length_m)
{
    const auto hx = static_cast<float>(gap_m / 2);
    const auto hy = static_cast<float>(length_m / 2);
    return {{-hx, -hy, 0}, {hx, -hy, 0}, {hx, hy, 0}, {-hx, hy, 0}};
}

} // namespace

namespace vision
{

Solver::Solver(const std::string &config_path)
{
    const YAML::Node yaml = YAML::LoadFile(config_path); // 缺少配置项或类型错误会抛异常

    auto k = yaml["camera_matrix"].as<std::vector<double>>();
    auto d = yaml["distort_coeffs"].as<std::vector<double>>();
    camera_matrix_ = cv::Mat(3, 3, CV_64F, k.data()).clone();
    distort_coeffs_ = cv::Mat(1, static_cast<int>(d.size()), CV_64F, d.data()).clone();

    const double gap_m = yaml["lightbar_gap"].as<double>() / 1000.0;
    const double length_m = yaml["lightbar_length"].as<double>() / 1000.0;
    model_ = model_points(gap_m, length_m);

    // 手眼：先绕光轴 roll，再低头 pitch，最后偏右 yaw
    const double yaw = yaml["handeye_yaw_deg"].as<double>() * deg2rad;
    const double pitch = yaml["handeye_pitch_deg"].as<double>() * deg2rad;
    const double roll = yaml["handeye_roll_deg"].as<double>() * deg2rad;
    R_camera2gimbal_ = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitX()) *
                       Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ());

    const auto offset = yaml["handeye_offset_mm"].as<std::vector<double>>();
    t_camera2gimbal_ = Eigen::Vector3d(offset[0], offset[1], offset[2]) / 1000.0;
}

void Solver::set_attitude(double yaw, double pitch)
{
    R_gimbal2world_ = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitX()); // pitch 正为向下，绕 +x 取负
}

void Solver::solve(Armor &armor)
{
    const std::vector<cv::Point2f> image_points(armor.corners.begin(), armor.corners.end());

    cv::Mat rvec, tvec;
    if (!cv::solvePnP(model_, image_points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
                      cv::SOLVEPNP_IPPE))
        return; // 解算失败，solved 保持 false

    const Eigen::Vector3d p_camera(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    const Eigen::Vector3d p_pitch = R_camera2gimbal_ * p_camera + t_camera2gimbal_; // 相机系 → 俯仰系
    armor.xyz = R_gimbal2world_ * p_pitch; // 俯仰系 → 世界系，云台转动时静止目标坐标不变

    // 板面法线：模型 +z 指向板内，取负才是朝外；转到世界系后取偏航角
    cv::Mat rotation;
    cv::Rodrigues(rvec, rotation);
    const Eigen::Vector3d n_camera(-rotation.at<double>(0, 2), -rotation.at<double>(1, 2),
                                   -rotation.at<double>(2, 2));
    const Eigen::Vector3d n_world = R_gimbal2world_ * (R_camera2gimbal_ * n_camera);
    armor.yaw = std::atan2(n_world.x(), n_world.z());

    armor.solved = true;
}

std::array<cv::Point2f, 4> Solver::project(const Eigen::Vector3d &xyz, double yaw) const
{
    // 板面绕世界竖直轴转到实际朝向；yaw 是板面法线的世界偏航角
    const Eigen::Matrix3d R_plate2world =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()).toRotationMatrix();

    std::vector<cv::Point3f> points;
    for (const auto &p : model_)
    {
        const Eigen::Vector3d p_world = xyz + R_plate2world * Eigen::Vector3d(p.x, p.y, p.z);
        // 世界系 → 俯仰系 → 相机系，hand-eye 的逆变换
        const Eigen::Vector3d p_camera =
            R_camera2gimbal_.transpose() *
            (R_gimbal2world_.inverse() * p_world - t_camera2gimbal_);
        points.emplace_back(static_cast<float>(p_camera.x()), static_cast<float>(p_camera.y()),
                            static_cast<float>(p_camera.z()));
    }

    std::vector<cv::Point2f> pixels;
    const cv::Vec3d zero(0, 0, 0); // 点已在相机系，旋转和平移都填 0
    cv::projectPoints(points, zero, zero, camera_matrix_, distort_coeffs_, pixels);

    std::array<cv::Point2f, 4> corners;
    std::copy(pixels.begin(), pixels.end(), corners.begin());
    return corners;
}

} // namespace vision
