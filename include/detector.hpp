#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "armor.hpp"

namespace vision
{

// 检测参数：Detector 与调参工具共用这一份定义，避免两边各写一套
struct DetectorParams
{
    bool is_red;               // 敌方颜色，true 红 false 蓝
    double color_threshold;    // 敌方通道减去对方通道的阈值
    double brightness_threshold; // 灰度阈值
    int morph_size;            // 闭运算核边长，像素，1 表示不处理
    double min_lightbar_area;  // 灯条面积下限，像素²
    double min_lightbar_ratio; // 灯条高/宽 下限
    double min_height_ratio;   // 两灯条高度 短/长 下限
    double max_tilt_diff_deg;  // 两灯条倾角差 上限，度
    double max_center_offset;  // 中心上下偏移/灯条高 上限
    double min_armor_ratio;    // 两灯条间距/灯条高 下限
    double max_armor_ratio;    // 两灯条间距/灯条高 上限
};

// 检测：图像 → 装甲板像素角点，不做坐标解算
class Detector
{
  public:
    explicit Detector(const std::string &config_path);

    const DetectorParams &params() const { return params_; } // 调参工具拿来起步
    void set_params(const DetectorParams &params);            // 调参工具热更新用

    std::vector<Armor> detect(const cv::Mat &img) const;

    // 拆开的三步，调参工具要单独看中间结果
    cv::Mat lightbar_mask(const cv::Mat &img) const;                         // 颜色 + 亮度 → 二值图
    cv::Mat close(const cv::Mat &mask) const;                                // 闭运算，点阵屏连成一片
    std::vector<cv::RotatedRect> find_lightbars(const cv::Mat &mask) const;  // 连通域 → 旋转矩形
    std::vector<Armor> pair(const std::vector<cv::RotatedRect> &bars) const; // 左右灯条 → 装甲板

  private:
    DetectorParams params_;
    cv::Mat kernel_; // 形态学核，由 morph_size 构造
};

} // namespace vision
