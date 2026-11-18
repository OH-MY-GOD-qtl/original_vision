// 标定：检测参数，滑动条实时调，退出时打印可直接粘贴回 auto_aim.yaml 的配置
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include "camera.hpp"

// 滑动条只能绑 int，小数参数用放大倍数存
struct Params
{
    int enemy_red;       // 0 蓝 / 1 红
    int color;           // 敌方通道差值阈值
    int brightness;      // 灰度阈值
    int dilate_size;     // 膨胀核边长，像素
    int min_area;        // 灯条面积下限，像素²
    int lightbar_ratio;  // 灯条长/宽 下限 ×10
    int height_ratio;    // 两灯条 短/长 下限 ×100
    int tilt_diff;       // 两灯条倾角差 上限，度 ×10
    int center_offset;   // 中心偏移/灯条长 上限 ×100
    int armor_ratio_min; // 间距/灯条长 下限 ×100
    int armor_ratio_max; // 间距/灯条长 上限 ×100
};

static Params params; // 滑动条直接改这几个 int

// 滑动条：名字、绑定的参数、上限
struct Slider
{
    const char *name;
    int *value;
    int max;
};

static void add_sliders(const std::string &win, const std::vector<Slider> &sliders)
{
    for (const auto &slider : sliders)
    {
        cv::createTrackbar(slider.name, win, nullptr, slider.max); // 不用已废弃的 value 指针
        cv::setTrackbarPos(slider.name, win, *slider.value);
    }
}

static void pull_sliders(const std::string &win, const std::vector<Slider> &sliders)
{
    for (const auto &slider : sliders)
        *slider.value = cv::getTrackbarPos(slider.name, win);
}

// 灯条像素：敌方通道够色，且灰度也够亮，与 detector.cpp 同一套算法
static cv::Mat lightbar_mask(const cv::Mat &img)
{
    std::vector<cv::Mat> bgr;
    cv::split(img, bgr);

    const cv::Mat diff = params.enemy_red ? bgr[2] - bgr[0] : bgr[0] - bgr[2];
    cv::Mat color_mask;
    cv::threshold(diff, color_mask, params.color, 255, cv::THRESH_BINARY);

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::Mat bright_mask;
    cv::threshold(gray, bright_mask, params.brightness, 255, cv::THRESH_BINARY);

    cv::Mat mask;
    cv::bitwise_and(color_mask, bright_mask, mask);
    return mask;
}

static double length(const cv::RotatedRect &bar) // 灯条长度取长边，与 detector.cpp 同名
{
    return std::max(bar.size.width, bar.size.height);
}

// 灯条倾角：长轴相对竖直方向，顺时针为正，与 detector.cpp 同一套算法
static double tilt(const cv::RotatedRect &bar)
{
    cv::Point2f p[4];
    bar.points(p);

    const bool long_is_01 = cv::norm(p[1] - p[0]) > cv::norm(p[2] - p[1]);
    const cv::Point2f e1 = long_is_01 ? (p[0] + p[3]) * 0.5f : (p[0] + p[1]) * 0.5f;
    const cv::Point2f e2 = long_is_01 ? (p[1] + p[2]) * 0.5f : (p[2] + p[3]) * 0.5f;

    const cv::Point2f top = e1.y < e2.y ? e1 : e2;
    const cv::Point2f bottom = e1.y < e2.y ? e2 : e1;
    return std::atan2(bottom.x - top.x, bottom.y - top.y) * 180.0 / CV_PI;
}

// 灯条：面积够大且足够细长
static std::vector<cv::RotatedRect> find_lightbars(const cv::Mat &mask)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::RotatedRect> bars;
    for (const auto &contour : contours)
    {
        const cv::RotatedRect bar = cv::minAreaRect(contour);
        if (bar.size.width * bar.size.height < params.min_area)
            continue; // 面积太小
        const double short_side = std::min(bar.size.width, bar.size.height);
        if (length(bar) < short_side * params.lightbar_ratio / 10.0)
            continue; // 不够细长
        bars.push_back(bar);
    }
    return bars;
}

// 配对：高度相近、倾角一致、中心对齐、间距像装甲板，返回下标对
static std::vector<std::pair<size_t, size_t>> find_pairs(const std::vector<cv::RotatedRect> &bars)
{
    std::vector<std::pair<size_t, size_t>> pairs;

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
            if (h < std::max(length(a), length(b)) * params.height_ratio / 100.0)
                continue; // 高度差太大

            if (std::abs(tilts[i] - tilts[j]) > params.tilt_diff / 10.0)
                continue; // 两灯条不平行

            if (std::abs(a.center.y - b.center.y) > h * params.center_offset / 100.0)
                continue; // 上下没对齐

            const double gap = std::abs(a.center.x - b.center.x);
            if (gap < h * params.armor_ratio_min / 100.0 || gap > h * params.armor_ratio_max / 100.0)
                continue; // 间距不像装甲板

            pairs.emplace_back(i, j);
        }
    }
    return pairs;
}

// 绿框灯条，红线连配对
static void draw(cv::Mat &img, const std::vector<cv::RotatedRect> &bars,
                 const std::vector<std::pair<size_t, size_t>> &pairs)
{
    for (const auto &bar : bars)
    {
        cv::Point2f p[4];
        bar.points(p);
        for (int i = 0; i < 4; ++i)
            cv::line(img, p[i], p[(i + 1) % 4], {0, 255, 0}, 1);
    }
    for (const auto &[i, j] : pairs)
        cv::line(img, bars[i].center, bars[j].center, {0, 0, 255}, 2);
}

int main(int, char *argv[])
{
    const std::string path = argv[1];
    const YAML::Node yaml = YAML::LoadFile(path);

    // 从现有配置起步，滑动条在此基础上微调
    params.enemy_red = yaml["enemy_color"].as<std::string>() == "red";
    params.color = static_cast<int>(yaml["color_threshold"].as<double>());
    params.brightness = static_cast<int>(yaml["brightness_threshold"].as<double>());
    params.dilate_size = yaml["dilate_size"].as<int>();
    params.min_area = static_cast<int>(yaml["min_lightbar_area"].as<double>());
    params.lightbar_ratio = static_cast<int>(yaml["min_lightbar_ratio"].as<double>() * 10);
    params.height_ratio = static_cast<int>(yaml["min_height_ratio"].as<double>() * 100);
    params.tilt_diff = static_cast<int>(yaml["max_tilt_diff_deg"].as<double>() * 10);
    params.center_offset = static_cast<int>(yaml["max_center_offset"].as<double>() * 100);
    params.armor_ratio_min = static_cast<int>(yaml["min_armor_ratio"].as<double>() * 100);
    params.armor_ratio_max = static_cast<int>(yaml["max_armor_ratio"].as<double>() * 100);

    vision::Camera camera(path);

    const std::string view_win = "detector tuning";
    const std::string mask_win = "lightbar mask";

    const std::vector<Slider> sliders = {
        {"enemy_red 0blue 1red", &params.enemy_red, 1},
        {"color_threshold", &params.color, 255},
        {"brightness_threshold", &params.brightness, 255},
        {"dilate_size", &params.dilate_size, 21},
        {"min_lightbar_area", &params.min_area, 2000},
        {"min_lightbar_ratio x10", &params.lightbar_ratio, 50},
        {"min_height_ratio x100", &params.height_ratio, 100},
        {"max_tilt_diff_deg x10", &params.tilt_diff, 900},
        {"max_center_offset x100", &params.center_offset, 100},
        {"min_armor_ratio x100", &params.armor_ratio_min, 500},
        {"max_armor_ratio x100", &params.armor_ratio_max, 1000},
    };

    cv::namedWindow(view_win, cv::WINDOW_NORMAL);
    cv::resizeWindow(view_win, 960, 720);
    add_sliders(view_win, sliders);
    cv::namedWindow(mask_win, cv::WINDOW_NORMAL);
    cv::resizeWindow(mask_win, 960, 720);

    vision::Frame frame;
    int tick = 0;
    while (cv::waitKey(1) != 27) // ESC 退出
    {
        camera.read(frame);
        pull_sliders(view_win, sliders);

        const cv::Mat mask = lightbar_mask(frame.img);
        cv::Mat thick;
        cv::dilate(mask, thick, cv::Mat::ones(params.dilate_size, params.dilate_size, CV_8U)); // 点阵屏缝隙
        const std::vector<cv::RotatedRect> bars = find_lightbars(thick);
        const std::vector<std::pair<size_t, size_t>> pairs = find_pairs(bars);

        draw(frame.img, bars, pairs); // read() 每帧重新转码，直接画在 frame.img 上
        cv::imshow(view_win, frame.img);
        cv::imshow(mask_win, thick); // 显示膨胀后的图，就是进 findContours 的那张

        if (++tick % 30 == 0) // 每 30 帧打一次，别刷屏
            std::printf("灯条 %zu  装甲板 %zu  mask 像素 %d\n", bars.size(), pairs.size(),
                        cv::countNonZero(mask));
    }

    std::printf("\n# 复制到 auto_aim.yaml\n");
    std::printf("enemy_color: \"%s\"\n", params.enemy_red ? "red" : "blue");
    std::printf("color_threshold: %d\n", params.color);
    std::printf("brightness_threshold: %d\n", params.brightness);
    std::printf("dilate_size: %d\n", params.dilate_size);
    std::printf("min_lightbar_area: %d\n", params.min_area);
    std::printf("min_lightbar_ratio: %.1f\n", params.lightbar_ratio / 10.0);
    std::printf("min_height_ratio: %.2f\n", params.height_ratio / 100.0);
    std::printf("max_tilt_diff_deg: %.1f\n", params.tilt_diff / 10.0);
    std::printf("max_center_offset: %.2f\n", params.center_offset / 100.0);
    std::printf("min_armor_ratio: %.2f\n", params.armor_ratio_min / 100.0);
    std::printf("max_armor_ratio: %.2f\n", params.armor_ratio_max / 100.0);

    return 0;
}
