#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "armor.hpp"

namespace vision
{

// 检测：图像 → 装甲板像素角点，不做坐标解算
class Detector
{
  public:
    explicit Detector(const std::string &config_path);

    std::vector<Armor> detect(const cv::Mat &img) const;

  private:
    cv::Mat lightbar_mask(const cv::Mat &img) const;                          // 颜色 + 亮度 → 二值图
    std::vector<cv::RotatedRect> find_lightbars(const cv::Mat &mask) const;   // 连通域 → 旋转矩形
    std::vector<Armor> pair(const std::vector<cv::RotatedRect> &bars) const;  // 左右灯条 → 装甲板

    bool is_red_;                  // 敌方颜色
    double color_threshold_;       // 敌方通道减去对方通道的阈值
    double brightness_threshold_;  // 灰度阈值
    double min_lightbar_ratio_;    // 灯条高/宽 下限
    double min_lightbar_area_;     // 灯条面积下限，像素²
    double min_height_ratio_;      // 两灯条高度 短/长 下限
    double max_tilt_diff_deg_;     // 两灯条倾角差 上限，度
    double max_center_offset_;     // 中心上下偏移/灯条高 上限
    double min_armor_ratio_;       // 两灯条间距/灯条高 下限
    double max_armor_ratio_;       // 两灯条间距/灯条高 上限
};

} // namespace vision
