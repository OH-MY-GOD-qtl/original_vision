#include "detector.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace
{

// 灯条长轴的两个端点，按 y 分出上下
void endpoints(const cv::RotatedRect &bar, cv::Point2f &top, cv::Point2f &bottom)
{
    cv::Point2f p[4];
    bar.points(p);

    const bool long_is_01 = cv::norm(p[1] - p[0]) > cv::norm(p[2] - p[1]); // 长边是 p0p1 还是 p1p2
    const cv::Point2f e1 = long_is_01 ? (p[0] + p[3]) * 0.5f : (p[0] + p[1]) * 0.5f;
    const cv::Point2f e2 = long_is_01 ? (p[1] + p[2]) * 0.5f : (p[2] + p[3]) * 0.5f;

    top = e1.y < e2.y ? e1 : e2;
    bottom = e1.y < e2.y ? e2 : e1;
}

// 灯条长度取长边，与 OpenCV 的 width/height 约定无关
double length(const cv::RotatedRect &bar)
{
    return std::max(bar.size.width, bar.size.height);
}

// 灯条倾角：长轴相对竖直方向，顺时针为正，范围 (-90, 90)
double tilt(const cv::RotatedRect &bar)
{
    cv::Point2f top, bottom;
    endpoints(bar, top, bottom);
    return std::atan2(bottom.x - top.x, bottom.y - top.y) * 180.0 / CV_PI;
}

// 四角：左上 → 右上 → 右下 → 左下
vision::Armor make_armor(
    const cv::Point2f &left_top, const cv::Point2f &right_top, const cv::Point2f &right_bottom,
    const cv::Point2f &left_bottom)
{
    vision::Armor armor;
    armor.corners = {left_top, right_top, right_bottom, left_bottom};
    return armor;
}

// 从配置读出一套检测参数
vision::DetectorParams load_detector_params(const std::string &config_path)
{
    const YAML::Node yaml = YAML::LoadFile(config_path); // 缺少配置项或类型错误会抛异常

    vision::DetectorParams params;
    params.is_red = yaml["enemy_color"].as<std::string>() == "red";
    params.color_threshold = yaml["color_threshold"].as<double>();
    params.brightness_threshold = yaml["brightness_threshold"].as<double>();
    params.morph_size = yaml["morph_size"].as<int>();
    params.min_lightbar_ratio = yaml["min_lightbar_ratio"].as<double>();
    params.min_lightbar_area = yaml["min_lightbar_area"].as<double>();
    params.min_height_ratio = yaml["min_height_ratio"].as<double>();
    params.max_tilt_diff_deg = yaml["max_tilt_diff_deg"].as<double>();
    params.max_center_offset = yaml["max_center_offset"].as<double>();
    params.min_armor_ratio = yaml["min_armor_ratio"].as<double>();
    params.max_armor_ratio = yaml["max_armor_ratio"].as<double>();
    return params;
}

} // namespace

namespace vision
{

Detector::Detector(const std::string &config_path)
{
    set_params(load_detector_params(config_path));
}

void Detector::set_params(const DetectorParams &params)
{
    params_ = params;
    kernel_ = cv::Mat::ones(params_.morph_size, params_.morph_size, CV_8U); // 方形核，点阵屏缝隙各向分布
}

std::vector<Armor> Detector::detect(const cv::Mat &img) const
{
    return pair(find_lightbars(close(lightbar_mask(img))));
}

cv::Mat Detector::close(const cv::Mat &mask) const
{
    cv::Mat closed;
    cv::morphologyEx(mask, closed, cv::MORPH_CLOSE, kernel_); // 点阵屏断开，闭运算连成一片且不撑大外框
    return closed;
}

// 灯条像素：敌方通道够色，且灰度也够亮
cv::Mat Detector::lightbar_mask(const cv::Mat &img) const
{
    std::vector<cv::Mat> bgr;
    cv::split(img, bgr);

    const cv::Mat diff = params_.is_red ? bgr[2] - bgr[0] : bgr[0] - bgr[2]; // 敌方通道减对方通道
    cv::Mat color_mask;
    cv::threshold(diff, color_mask, params_.color_threshold, 255, cv::THRESH_BINARY);

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::Mat bright_mask;
    cv::threshold(gray, bright_mask, params_.brightness_threshold, 255, cv::THRESH_BINARY);

    cv::Mat mask;
    cv::bitwise_and(color_mask, bright_mask, mask);
    return mask;
}

// 灯条：轮廓的最小外接旋转矩形，去掉太小和不够细长的
std::vector<cv::RotatedRect> Detector::find_lightbars(const cv::Mat &mask) const
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::RotatedRect> bars;
    for (const auto &contour : contours)
    {
        const cv::RotatedRect bar = cv::minAreaRect(contour);
        if (bar.size.width * bar.size.height < params_.min_lightbar_area)
            continue; // 面积太小，多半是噪点
        const double short_side = std::min(bar.size.width, bar.size.height);
        if (length(bar) < short_side * params_.min_lightbar_ratio)
            continue; // 不够细长
        bars.push_back(bar);
    }
    return bars;
}

// 左右配对：高度相近、倾角一致、中心对齐、间距像装甲板
std::vector<Armor> Detector::pair(const std::vector<cv::RotatedRect> &bars) const
{
    std::vector<Armor> armors;

    std::vector<double> tilts(bars.size());
    for (size_t i = 0; i < bars.size(); ++i)
        tilts[i] = tilt(bars[i]);

    for (size_t i = 0; i < bars.size(); ++i)
    {
        for (size_t j = i + 1; j < bars.size(); ++j)
        {
            const cv::RotatedRect &a = bars[i];
            const cv::RotatedRect &b = bars[j];

            const double h = std::min(length(a), length(b));
            if (h < std::max(length(a), length(b)) * params_.min_height_ratio)
                continue; // 高度差太大

            if (std::abs(tilts[i] - tilts[j]) > params_.max_tilt_diff_deg)
                continue; // 两灯条不平行，不是同一块装甲板

            const double dy = std::abs(a.center.y - b.center.y);
            if (dy > h * params_.max_center_offset)
                continue; // 上下没对齐

            const double gap = std::abs(a.center.x - b.center.x);
            if (gap < h * params_.min_armor_ratio || gap > h * params_.max_armor_ratio)
                continue; // 间距不像装甲板

            const cv::RotatedRect &left = a.center.x < b.center.x ? a : b;
            const cv::RotatedRect &right = a.center.x < b.center.x ? b : a;

            cv::Point2f left_top, left_bottom, right_top, right_bottom;
            endpoints(left, left_top, left_bottom);
            endpoints(right, right_top, right_bottom);
            armors.push_back(make_armor(left_top, right_top, right_bottom, left_bottom));
        }
    }
    return armors;
}

} // namespace vision
