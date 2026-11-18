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

} // namespace

namespace vision
{

Detector::Detector(const std::string &config_path)
{
    const YAML::Node yaml = YAML::LoadFile(config_path); // 缺少配置项或类型错误会抛异常
    is_red_ = yaml["enemy_color"].as<std::string>() == "red";
    color_threshold_ = yaml["color_threshold"].as<double>();
    brightness_threshold_ = yaml["brightness_threshold"].as<double>();
    dilate_size_ = yaml["dilate_size"].as<int>();
    kernel_ = cv::Mat::ones(dilate_size_, dilate_size_, CV_8U); // 方形核，点阵屏缝隙各向分布
    min_lightbar_ratio_ = yaml["min_lightbar_ratio"].as<double>();
    min_lightbar_area_ = yaml["min_lightbar_area"].as<double>();
    min_height_ratio_ = yaml["min_height_ratio"].as<double>();
    max_tilt_diff_deg_ = yaml["max_tilt_diff_deg"].as<double>();
    max_center_offset_ = yaml["max_center_offset"].as<double>();
    min_armor_ratio_ = yaml["min_armor_ratio"].as<double>();
    max_armor_ratio_ = yaml["max_armor_ratio"].as<double>();
}

std::vector<Armor> Detector::detect(const cv::Mat &img) const
{
    const cv::Mat mask = lightbar_mask(img);

    cv::Mat thick;
    cv::dilate(mask, thick, kernel_); // 点阵屏灯条是断开的点，先连成一片

    return pair(find_lightbars(thick));
}

// 灯条像素：敌方通道够色，且灰度也够亮
cv::Mat Detector::lightbar_mask(const cv::Mat &img) const
{
    std::vector<cv::Mat> bgr;
    cv::split(img, bgr);

    const cv::Mat diff = is_red_ ? bgr[2] - bgr[0] : bgr[0] - bgr[2]; // 敌方通道减对方通道
    cv::Mat color_mask;
    cv::threshold(diff, color_mask, color_threshold_, 255, cv::THRESH_BINARY);

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::Mat bright_mask;
    cv::threshold(gray, bright_mask, brightness_threshold_, 255, cv::THRESH_BINARY);

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
        if (bar.size.width * bar.size.height < min_lightbar_area_)
            continue; // 面积太小，多半是噪点
        const double short_side = std::min(bar.size.width, bar.size.height);
        if (length(bar) < short_side * min_lightbar_ratio_)
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
            if (h < std::max(length(a), length(b)) * min_height_ratio_)
                continue; // 高度差太大

            if (std::abs(tilts[i] - tilts[j]) > max_tilt_diff_deg_)
                continue; // 两灯条不平行，不是同一块装甲板

            const double dy = std::abs(a.center.y - b.center.y);
            if (dy > h * max_center_offset_)
                continue; // 上下没对齐

            const double gap = std::abs(a.center.x - b.center.x);
            if (gap < h * min_armor_ratio_ || gap > h * max_armor_ratio_)
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
